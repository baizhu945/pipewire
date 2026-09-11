/* Spa experimental aptX Adaptive A2DP codec bridge */
/* SPDX-License-Identifier: MIT */

#include "config.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <spa/param/audio/format.h>
#include <spa/param/audio/format-utils.h>
#include <spa/utils/string.h>

#include <aptxadaptive.h>

#include "a2dp-codec-caps.h"
#include "media-codecs.h"
#include "rtp.h"

/*
 * This bridge is opt-in at runtime.  PipeWire talks to a long-lived helper
 * over two pipes; the helper is expected to contain a user-supplied,
 * licensed Qualcomm Hexagon encoder.  No proprietary library is linked into
 * this plugin.
 */
#define APTX_ADAPTIVE_HELPER_ENV "PIPEWIRE_APTX_ADAPTIVE_HELPER"
#define APTX_ADAPTIVE_QEMU_ENV "PIPEWIRE_APTX_ADAPTIVE_QEMU"
#define APTX_ADAPTIVE_SYSROOT_ENV "PIPEWIRE_APTX_ADAPTIVE_SYSROOT"
#define APTX_ADAPTIVE_MODE_ENV "PIPEWIRE_APTX_ADAPTIVE_MODE"
#define APTX_ADAPTIVE_PROFILE_ENV "APTX_ADAPTIVE_PROFILE"
#define APTX_ADAPTIVE_STREAM_OVERRIDE_ENV \
	"APTX_ADAPTIVE_CONFIG_STREAM_OVERRIDE_HEX"
#define APTX_ADAPTIVE_LOSSLESS_ENV "APTX_ADAPTIVE_LOSSLESS"
#define APTX_ADAPTIVE_QHS_ENV "APTX_ADAPTIVE_QHS_SUPPORT"
#define APTX_ADAPTIVE_ABR_ENV "APTX_ADAPTIVE_ABR"
#define APTX_ADAPTIVE_ADVERTISE_R2_2_ENV "APTX_ADAPTIVE_ADVERTISE_R2_2"

#define APTX_ADAPTIVE_CHANNELS 2u
#define APTX_ADAPTIVE_HELPER_BITS_PER_SAMPLE 32u
#define APTX_ADAPTIVE_CODEC_FRAMES 672u
/* The direct R3 pipeline consumes 720 samples per frame (measured at 44.1/48/96
 * kHz): with 672-sample blocks the encoder's window drains 48 samples per call
 * and half the calls return EAGAIN. */
#define APTX_ADAPTIVE_R3_CODEC_FRAMES 720u
#define APTX_ADAPTIVE_CODEC_BYTES \
	(APTX_ADAPTIVE_CHANNELS * (APTX_ADAPTIVE_HELPER_BITS_PER_SAMPLE / 8u) * \
	 APTX_ADAPTIVE_CODEC_FRAMES)
#define APTX_ADAPTIVE_MAX_PACKET_SIZE 4096u
#define APTX_ADAPTIVE_MAX_SOURCE_FRAMES (APTX_ADAPTIVE_CODEC_FRAMES * 2u)
#define APTX_ADAPTIVE_RTP_HEADER_SIZE ((size_t)sizeof(struct rtp_header))

#define APTX_ADAPTIVE_ABR_LEVELS 5u
#define APTX_ADAPTIVE_ABR_CONFIRMATIONS 3u

/* Upper bound for a single helper reply.  The helper answers in well under a
 * millisecond; this only protects the data thread from a stuck emulator. */
#define APTX_ADAPTIVE_HELPER_READ_TIMEOUT_MS 1000
/* The same bound applies to writes.  One PCM block is at most ~18 KiB and the
 * pipe holds 64 KiB, so a healthy helper never makes the writer wait; without
 * a deadline a helper that stops reading stdin would block the data thread
 * forever once the pipe is full. */
#define APTX_ADAPTIVE_HELPER_WRITE_TIMEOUT_MS 1000
/* Smallest transport MTU that can carry an ordinary R2 packet: RTP header plus
 * the 8-byte wrapper plus the 656-byte codec frame.  A smaller MTU cannot
 * carry the stream at all, so say so instead of dropping every packet. */
#define APTX_ADAPTIVE_MIN_MTU 700
/* A staged, self-contained format block is large; keep the helper protocol
 * from being handed an absurd length. */
#define APTX_ADAPTIVE_MAX_HELPER_REQUEST (1u << 20)

static int64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* A causal 2:1 half-band filter.  It is used only for 88.2->44.1 and
 * 192->96 graph formats, because the available Qualcomm CAPI build accepts
 * 44.1, 48 and 96 kHz as codec-native input rates. */
#define APTX_ADAPTIVE_DOWNSAMPLE_TAPS 15u
#define APTX_ADAPTIVE_DOWNSAMPLE_HISTORY (APTX_ADAPTIVE_DOWNSAMPLE_TAPS - 1u)
static const int32_t downsample2_coeffs[APTX_ADAPTIVE_DOWNSAMPLE_TAPS] = {
	0, 188, 0, -1595, 0, 9590, 0, 16384,
	0, 9590, 0, -1595, 0, 188, 0,
};

static struct spa_log *log_;

struct adaptive_rate {
	uint32_t graph_rate;
	uint32_t codec_rate;
	uint8_t codec_frequency;
};

static const struct adaptive_rate adaptive_rates[] = {
	{ 48000, 48000, APTX_ADAPTIVE_SAMPLING_FREQ_48000 },
	{ 44100, 44100, APTX_ADAPTIVE_SAMPLING_FREQ_44100 },
	{ 88200, 44100, APTX_ADAPTIVE_SAMPLING_FREQ_44100 },
	{ 96000, 96000, APTX_ADAPTIVE_SAMPLING_FREQ_96000 },
	{ 192000, 96000, APTX_ADAPTIVE_SAMPLING_FREQ_96000 },
};

enum adaptive_pcm_format {
	ADAPTIVE_PCM_S16,
	ADAPTIVE_PCM_S24_32,
	ADAPTIVE_PCM_S32,
};

static const uint8_t adaptive_ttp[6] = {
	APTX_ADAPTIVE_TTP_LL_0,
	APTX_ADAPTIVE_TTP_LL_1,
	APTX_ADAPTIVE_TTP_HQ_0,
	APTX_ADAPTIVE_TTP_HQ_1,
	APTX_ADAPTIVE_TTP_TWS_0,
	APTX_ADAPTIVE_TTP_TWS_1,
};

static const uint8_t adaptive_setup_pref[4] = { 2, 3, 3, 3 };

static uint32_t adaptive_read_features(const a2dp_aptx_adaptive_t *caps)
{
	return (uint32_t)caps->supported_features[0] |
			(uint32_t)caps->supported_features[1] << 8 |
			(uint32_t)caps->supported_features[2] << 16 |
			(uint32_t)caps->supported_features[3] << 24;
}

static void adaptive_write_features(a2dp_aptx_adaptive_t *caps,
		uint32_t features)
{
	caps->supported_features[0] = features & 0xff;
	caps->supported_features[1] = (features >> 8) & 0xff;
	caps->supported_features[2] = (features >> 16) & 0xff;
	caps->supported_features[3] = (features >> 24) & 0xff;
}

static uint8_t adaptive_sampling_freq(const a2dp_aptx_adaptive_t *caps)
{
	return caps->sampling_freq_source_type & APTX_ADAPTIVE_SAMPLING_FREQ_MASK;
}

static uint8_t adaptive_source_type(const a2dp_aptx_adaptive_t *caps)
{
	return caps->sampling_freq_source_type & APTX_ADAPTIVE_SOURCE_TYPE_MASK;
}

static void adaptive_set_sampling_freq(a2dp_aptx_adaptive_t *caps,
		uint8_t sampling_freq)
{
	caps->sampling_freq_source_type =
			(sampling_freq & APTX_ADAPTIVE_SAMPLING_FREQ_MASK) |
			(adaptive_source_type(caps) & APTX_ADAPTIVE_SOURCE_TYPE_MASK);
}

static void adaptive_init_caps(a2dp_aptx_adaptive_t *caps,
		uint32_t features, uint8_t sampling_freq, uint8_t source_type)
{
	/* Diagnostic override: match a working peer configuration exactly.
	 * APTX_ADAPTIVE_SOURCE_TYPE_1 (0x00) is what an Android source
	 * advertises; the stock default here is SOURCE_TYPE_2 (0x02). */
	const char *st = getenv("APTX_ADAPTIVE_SOURCE_TYPE");
	if (st != NULL && *st != '\0')
		source_type = (uint8_t)strtoul(st, NULL, 0);
	memset(caps, 0, sizeof(*caps));
	caps->info.vendor_id = APTX_ADAPTIVE_VENDOR_ID;
	caps->info.codec_id = APTX_ADAPTIVE_CODEC_ID;
	caps->sampling_freq_source_type =
			(sampling_freq & APTX_ADAPTIVE_SAMPLING_FREQ_MASK) |
			(source_type & APTX_ADAPTIVE_SOURCE_TYPE_MASK);
	caps->channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_STEREO_SOURCE;
	memcpy(caps->ttp, adaptive_ttp, sizeof(caps->ttp));
	caps->reserved_15thbyte = APTX_ADAPTIVE_RESERVED_15THBYTE;
	caps->cap_ext_ver_num = APTX_ADAPTIVE_CAP_EXT_VER_NUM;
	adaptive_write_features(caps, features);
	memcpy(caps->setup_pref, adaptive_setup_pref,
			sizeof(caps->setup_pref));
	caps->eoc[0] = APTX_ADAPTIVE_EOC0;
	caps->eoc[1] = APTX_ADAPTIVE_EOC1;
}

static void adaptive_log_caps(const char *label,
		const a2dp_aptx_adaptive_t *caps)
{
	if (log_ == NULL)
		return;
	spa_log_debug(log_,
			"aptX Adaptive %s: freq-source=0x%02x channel=0x%02x "
			"ext=%u features=0x%08x setup=%02x%02x%02x%02x eoc=%02x%02x",
			label, caps->sampling_freq_source_type, caps->channel_mode,
			caps->cap_ext_ver_num, adaptive_read_features(caps),
			caps->setup_pref[0], caps->setup_pref[1], caps->setup_pref[2],
			caps->setup_pref[3], caps->eoc[0], caps->eoc[1]);
}

SPA_STATIC_ASSERT(sizeof(a2dp_aptx_adaptive_t) == 40);

struct impl {
	pid_t pid;
	int input_fd;
	int output_fd;
	bool sink;

	uint32_t source_rate;
	uint32_t codec_rate;
	uint32_t source_frames;
	uint32_t codec_frames;
	size_t helper_bytes;
	int block_size;
	int mtu;
	enum aptx_adaptive_helper_mode mode;
	uint32_t profile;
	bool downsample2;
	enum adaptive_pcm_format pcm_format;
	uint32_t source_bytes;
	enum aptx_adaptive_helper_lossless_mode lossless_mode;
	bool qhs_supported;
	uint8_t r2_stream[APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE];

	int32_t downsample_history[APTX_ADAPTIVE_CHANNELS]
		[APTX_ADAPTIVE_DOWNSAMPLE_HISTORY];
	int32_t source_pcm[APTX_ADAPTIVE_MAX_SOURCE_FRAMES * APTX_ADAPTIVE_CHANNELS];
	int32_t codec_pcm[APTX_ADAPTIVE_MAX_SOURCE_FRAMES * APTX_ADAPTIVE_CHANNELS];
	uint8_t packet[APTX_ADAPTIVE_MAX_PACKET_SIZE];

	bool abr_enabled;
	unsigned int abr_level;
	unsigned int abr_pending_level;
	unsigned int abr_pending_count;
};

static const struct adaptive_rate *find_rate(uint32_t rate)
{
	for (size_t i = 0; i < SPA_N_ELEMENTS(adaptive_rates); ++i) {
		if (adaptive_rates[i].graph_rate == rate)
			return &adaptive_rates[i];
	}
	return NULL;
}

/* The R2 CAPI wrapper emits one OTA packet per fixed-duration frame.  Measured
 * against the CPH2749 module: 1102.8 samples at 44.1 kHz (25 ms), 1200 at
 * 48 kHz (25 ms) and 1923.7 at 96 kHz (20 ms).  The block handed to the helper
 * must track the frame length, otherwise the wrapper only produces a packet
 * every other call and the RTP timestamp drifts against the codec frames. */
static uint32_t r2_frames_for_rate(uint32_t rate)
{
	switch (rate) {
	case 44100:
		return 1102;
	case 48000:
		return 1200;
	case 96000:
		return 1920;
	default:
		return APTX_ADAPTIVE_CODEC_FRAMES;
	}
}

static enum aptx_adaptive_helper_mode get_helper_mode(uint32_t source_rate)
{
	const char *value = getenv(APTX_ADAPTIVE_MODE_ENV);
	if (value != NULL) {
		if (spa_streq(value, "r2") || spa_streq(value, "R2"))
			return APTX_ADAPTIVE_HELPER_MODE_R2;
		if (spa_streq(value, "r3") || spa_streq(value, "R3"))
			return APTX_ADAPTIVE_HELPER_MODE_R3;
	}

	/* The helper's automatic path uses the R2 CAPI wrapper.  That wrapper is
	 * the only available entry point that can carry the 2.2 capability stream,
	 * AudioReach ABR feedback, and the 44.1 kHz Lossless state machine. */
	(void)source_rate;
	return APTX_ADAPTIVE_HELPER_MODE_AUTO;
}

static uint32_t get_profile(void)
{
	const char *value = getenv(APTX_ADAPTIVE_PROFILE_ENV);
	return value == NULL ? 6u : (uint32_t)strtoul(value, NULL, 0);
}

static enum aptx_adaptive_helper_lossless_mode get_lossless_mode(void)
{
	const char *value = getenv(APTX_ADAPTIVE_LOSSLESS_ENV);

	if (value == NULL || spa_streq(value, "off") || spa_streq(value, "OFF"))
		return APTX_ADAPTIVE_HELPER_LOSSLESS_OFF;
	if (spa_streq(value, "auto") || spa_streq(value, "AUTO"))
		return APTX_ADAPTIVE_HELPER_LOSSLESS_AUTO;
	if (spa_streq(value, "force") || spa_streq(value, "FORCE"))
		return APTX_ADAPTIVE_HELPER_LOSSLESS_FORCE;
	return APTX_ADAPTIVE_HELPER_LOSSLESS_OFF;
}

static bool lossless_enabled(void)
{
	return get_lossless_mode() != APTX_ADAPTIVE_HELPER_LOSSLESS_OFF;
}

static bool get_qhs_supported(void)
{
	const char *value = getenv(APTX_ADAPTIVE_QHS_ENV);

	/* An ordinary host Bluetooth controller does not expose Qualcomm High
	 * Speed Link.  Require an explicit assertion before AUTO may enter the
	 * vendor Lossless candidate path. */
	return value != NULL && (spa_streq(value, "1") ||
			spa_streq(value, "yes") || spa_streq(value, "true") ||
			spa_streq(value, "YES") || spa_streq(value, "TRUE"));
}

static bool get_advertise_r2_2(void)
{
	const char *value = getenv(APTX_ADAPTIVE_ADVERTISE_R2_2_ENV);

	/* Report the R2.2 capability to the peer while the encoder keeps running
	 * its ordinary R2 path.  Every Qualcomm source advertises the capability
	 * (the phone sends features 0x0f000092), but this bridge must not let the
	 * proprietary R2.2 wrapper take over the encoder: that state waits for
	 * QHS/16-bit sideband feedback which a non-Qualcomm controller cannot
	 * provide, and the stream then stalls.  Keeping the two concerns apart
	 * makes it possible to test the advertisement on its own. */
	return value != NULL && (spa_streq(value, "1") ||
			spa_streq(value, "yes") || spa_streq(value, "true") ||
			spa_streq(value, "YES") || spa_streq(value, "TRUE"));
}

static bool get_abr_enabled(void)
{
	const char *value = getenv(APTX_ADAPTIVE_ABR_ENV);
	return value == NULL || (!spa_streq(value, "0") &&
			!spa_streq(value, "off") && !spa_streq(value, "OFF"));
}

static bool helper_available(void)
{
	const char *helper = getenv(APTX_ADAPTIVE_HELPER_ENV);
	const char *qemu = getenv(APTX_ADAPTIVE_QEMU_ENV);
	const char *sysroot = getenv(APTX_ADAPTIVE_SYSROOT_ENV);

	if (helper == NULL || access(helper, X_OK) < 0)
		return false;
	if (qemu != NULL || sysroot != NULL) {
		if (qemu == NULL || sysroot == NULL)
			return false;
		if (access(qemu, X_OK) < 0 || access(sysroot, R_OK | X_OK) < 0)
			return false;
	}
	return true;
}

static int read_full(int fd, void *data, size_t size)
{
	uint8_t *p = data;
	/* The helper runs inside a user-mode CPU emulator.  A stuck emulator must
	 * not block the PipeWire data thread forever, so bound every read with a
	 * deadline that is far longer than the normal sub-millisecond response. */
	int64_t deadline = now_ms() + APTX_ADAPTIVE_HELPER_READ_TIMEOUT_MS;

	while (size > 0) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN };
		int64_t remaining = deadline - now_ms();
		ssize_t n;
		int ret;

		if (remaining <= 0)
			return -ETIMEDOUT;
		ret = poll(&pfd, 1, (int)SPA_MIN(remaining, (int64_t)INT_MAX));
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			return -errno;
		}
		if (ret == 0)
			return -ETIMEDOUT;

		n = read(fd, p, size);
		if (n == 0)
			return -EPIPE;
		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
				continue;
			return -errno;
		}
		p += (size_t)n;
		size -= (size_t)n;
	}
	return 0;
}

static int write_full(int fd, const void *data, size_t size)
{
	const uint8_t *p = data;
	/* Bound every write with a deadline, exactly like read_full().  A helper
	 * that stops reading its stdin (stuck emulator, crashed QEMU, a helper
	 * that is itself blocked writing to a full stdout pipe) would otherwise
	 * block the PipeWire data thread forever once the pipe buffer is full. */
	int64_t deadline = now_ms() + APTX_ADAPTIVE_HELPER_WRITE_TIMEOUT_MS;

	while (size > 0) {
		struct pollfd pfd = { .fd = fd, .events = POLLOUT };
		int64_t remaining = deadline - now_ms();
		ssize_t n;
		int ret;

		if (remaining <= 0)
			return -ETIMEDOUT;
		ret = poll(&pfd, 1, (int)SPA_MIN(remaining, (int64_t)INT_MAX));
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			return -errno;
		}
		if (ret == 0)
			return -ETIMEDOUT;

		n = write(fd, p, size);
		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
				continue;
			return -errno;
		}
		if (n == 0)
			return -EPIPE;
		p += (size_t)n;
		size -= (size_t)n;
	}
	return 0;
}

/* Parse a boolean environment switch.  An unset variable takes the documented
 * default; an empty value means "yes"; the usual off spellings mean "no".
 * This exists because the previous code tested getenv() != NULL, so
 * APTX_ADAPTIVE_STRIP_OTA=0 silently enabled the option instead of disabling
 * it. */
static bool env_flag(const char *name, bool fallback)
{
	const char *value = getenv(name);

	if (value == NULL)
		return fallback;
	if (*value == '\0')
		return true;
	return !(spa_streq(value, "0") || spa_streq(value, "no") ||
			spa_streq(value, "false") || spa_streq(value, "off"));
}

static uint32_t read_u32le(const uint8_t data[4])
{
	return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
			(uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static void write_u32le(uint8_t data[4], uint32_t value)
{
	data[0] = value & 0xff;
	data[1] = (value >> 8) & 0xff;
	data[2] = (value >> 16) & 0xff;
	data[3] = (value >> 24) & 0xff;
}

static void close_fds(int input_fd, int output_fd)
{
	if (input_fd >= 0)
		close(input_fd);
	if (output_fd >= 0)
		close(output_fd);
}

static void reap_helper(pid_t pid)
{
	int wait_status;
	unsigned int i;

	if (pid <= 0)
		return;
	/* Closing the pipes normally makes the helper exit.  A stuck emulator must
	 * not block the caller forever, so give it a short grace period and then
	 * terminate it.  This runs from the data thread when init fails. */
	for (i = 0; i < 50; ++i) {
		pid_t r = waitpid(pid, &wait_status, WNOHANG);
		if (r == pid)
			return;
		if (r < 0 && errno != EINTR)
			return;
		usleep(20000);
	}
	kill(pid, SIGTERM);
	for (i = 0; i < 25; ++i) {
		pid_t r = waitpid(pid, &wait_status, WNOHANG);
		if (r == pid)
			return;
		if (r < 0 && errno != EINTR)
			return;
		usleep(20000);
	}
	kill(pid, SIGKILL);
	while (waitpid(pid, &wait_status, 0) < 0 && errno == EINTR)
		;
}

static void child_set_working_directory(const char *helper)
{
	char path[PATH_MAX];
	char *slash;

	if (helper == NULL || strlen(helper) >= sizeof(path))
		return;
	strcpy(path, helper);
	slash = strrchr(path, '/');
	if (slash != NULL && slash != path) {
		*slash = '\0';
		if (chdir(path) < 0)
			return;
	}
}

static pid_t spawn_helper(int *input_fd, int *output_fd)
{
	const char *helper = getenv(APTX_ADAPTIVE_HELPER_ENV);
	const char *qemu = getenv(APTX_ADAPTIVE_QEMU_ENV);
	const char *sysroot = getenv(APTX_ADAPTIVE_SYSROOT_ENV);
	int input_pipe[2] = { -1, -1 };
	int output_pipe[2] = { -1, -1 };
	pid_t pid;

	if (pipe(input_pipe) < 0 || pipe(output_pipe) < 0) {
		close_fds(input_pipe[0], input_pipe[1]);
		close_fds(output_pipe[0], output_pipe[1]);
		return -1;
	}

	pid = fork();
	if (pid < 0) {
		close_fds(input_pipe[0], input_pipe[1]);
		close_fds(output_pipe[0], output_pipe[1]);
		return -1;
	}
	if (pid == 0) {
		sigset_t empty;
		int fd;

		close(input_pipe[1]);
		close(output_pipe[0]);
		if (dup2(input_pipe[0], STDIN_FILENO) < 0 ||
				dup2(output_pipe[1], STDOUT_FILENO) < 0)
			_exit(127);
		close(input_pipe[0]);
		close(output_pipe[1]);
		/* The parent may run with signals blocked (PipeWire blocks SIGINT and
		 * SIGTERM) and with descriptors that the helper has no business
		 * holding.  Start from a clean slate so that kill() actually reaches
		 * the emulator and so that a crashed helper cannot keep a PipeWire
		 * socket alive.  dup2() above already placed our two pipes. */
		sigemptyset(&empty);
		sigprocmask(SIG_SETMASK, &empty, NULL);
		signal(SIGPIPE, SIG_DFL);
		for (fd = 3; fd < 1024; ++fd)
			close(fd);
		child_set_working_directory(helper);

		if (qemu != NULL)
			execl(qemu, qemu, "-cpu", "v68", "-L", sysroot, helper, NULL);
		else
			execl(helper, helper, NULL);
		_exit(127);
	}

	close(input_pipe[0]);
	close(output_pipe[1]);
	*input_fd = input_pipe[1];
	*output_fd = output_pipe[0];
	return pid;
}

static int helper_control(struct impl *this, uint32_t command,
		const void *payload, size_t payload_size)
{
	uint8_t header[12];
	uint8_t reply[8];
	int result;

	if (payload_size > APTX_ADAPTIVE_MAX_HELPER_REQUEST)
		return -EOVERFLOW;
	write_u32le(header, APTX_ADAPTIVE_HELPER_CONTROL);
	write_u32le(header + 4, command);
	write_u32le(header + 8, (uint32_t)payload_size);
	if ((result = write_full(this->input_fd, header, sizeof(header))) < 0 ||
			(result = write_full(this->input_fd, payload, payload_size)) < 0 ||
			(result = read_full(this->output_fd, reply, sizeof(reply))) < 0)
		return result;

	uint32_t status = read_u32le(reply);
	uint32_t reply_size = read_u32le(reply + 4);
	return reply_size == 0 && status == 0 ? 0 :
		(status == 0 ? -EPROTO : -(int)status);
}

static int hex_value(char value)
{
	if (value >= '0' && value <= '9')
		return value - '0';
	if (value >= 'a' && value <= 'f')
		return value - 'a' + 10;
	if (value >= 'A' && value <= 'F')
		return value - 'A' + 10;
	return -1;
}

static int init_r2_stream(uint8_t stream[APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE])
{
	const char *value = getenv(APTX_ADAPTIVE_STREAM_OVERRIDE_ENV);

	if (value == NULL)
		return 0;
	if (strlen(value) != APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE * 2u)
		return -EINVAL;
	for (size_t i = 0; i < APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE; ++i) {
		int high = hex_value(value[i * 2]);
		int low = hex_value(value[i * 2 + 1]);
		if (high < 0 || low < 0)
			return -EINVAL;
		stream[i] = (uint8_t)((high << 4) | low);
	}
	return 0;
}

static void build_r2_stream(const a2dp_aptx_adaptive_t *caps,
		uint8_t stream[APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE])
{
	stream[0] = caps->cap_ext_ver_num;
	memcpy(stream + 1, caps->supported_features,
			sizeof(caps->supported_features));
	memcpy(stream + 5, caps->setup_pref, sizeof(caps->setup_pref));
	memcpy(stream + 9, caps->eoc, sizeof(caps->eoc));
}

static int initialize_helper(struct impl *this, const uint8_t *codec_config,
		size_t codec_config_size)
{
	struct aptx_adaptive_helper_config config = { 0 };
	uint8_t ready[8];
	int result;

	config.protocol_version = APTX_ADAPTIVE_HELPER_PROTOCOL_VERSION;
	config.source_rate = this->source_rate;
	config.encoder_rate = this->codec_rate;
	config.mode = this->mode;
	config.profile = this->profile;
	config.mtu = this->mtu > (int)APTX_ADAPTIVE_RTP_HEADER_SIZE ?
			(uint32_t)this->mtu - APTX_ADAPTIVE_RTP_HEADER_SIZE : 0u;
	config.abr_enabled = this->abr_enabled ? 1u : 0u;
	/* The helper boundary is always S32/Q27, but the original source word size
	 * is part of the 2.2 lossless state machine.  Preserve it when PipeWire
	 * supplied S16; S24_32 and S32 remain the ordinary 32-bit CAPI path. */
	config.bits_per_sample = this->pcm_format == ADAPTIVE_PCM_S16 ? 16u : 32u;
	config.lossless_mode = this->lossless_mode;
	config.qhs_supported = this->qhs_supported ? 1u : 0u;
	config.cie_size = APTX_ADAPTIVE_HELPER_CIE_SIZE;
	memcpy(config.cie, codec_config,
			SPA_MIN(codec_config_size, sizeof(config.cie)));
	/* The peer may be told that this source supports R2.2 while the encoder
	 * must not see that capability: presenting it to the proprietary R2.2
	 * wrapper makes it wait for QHS/16-bit sideband feedback that a
	 * non-Qualcomm controller cannot supply, and the stream then stalls.
	 * The advertisement and the encoder input are therefore prepared from
	 * two different copies of the same element. */
	if (get_advertise_r2_2() && !lossless_enabled() &&
			codec_config_size >= sizeof(a2dp_aptx_adaptive_t)) {
		a2dp_aptx_adaptive_t encoder_caps;
		size_t copy = SPA_MIN(sizeof(encoder_caps), sizeof(config.cie));

		memcpy(&encoder_caps, codec_config, sizeof(encoder_caps));
		adaptive_write_features(&encoder_caps,
				adaptive_read_features(&encoder_caps) &
				~(uint32_t)APTX_ADAPTIVE_R2_2_SUPPORT_CAP);
		memcpy(config.cie, &encoder_caps, copy);
	}
	memcpy(config.r2_stream, this->r2_stream, sizeof(config.r2_stream));
	if ((result = init_r2_stream(config.r2_stream)) < 0)
		return result;

	if ((result = read_full(this->output_fd, ready, sizeof(ready))) < 0)
		return result;
	if (read_u32le(ready) != 0 || read_u32le(ready + 4) != 0)
		return -EPROTO;

	if ((result = helper_control(this, APTX_ADAPTIVE_HELPER_COMMAND_CONFIG,
				&config, sizeof(config))) < 0)
		return result;

	return 0;
}

static int32_t downsample2_sample(const struct impl *this,
		const int32_t *src, uint32_t source_frames, int channel, int index)
{
	if (index < 0)
		return this->downsample_history[channel]
			[APTX_ADAPTIVE_DOWNSAMPLE_HISTORY + index];
	if ((uint32_t)index >= source_frames)
		return 0;
	return src[(size_t)index * APTX_ADAPTIVE_CHANNELS + (size_t)channel];
}

static int32_t scale_filter_sum(int64_t sum)
{
	if (sum >= 0)
		sum += 1 << 14;
	else
		sum -= 1 << 14;
	sum >>= 15;
	if (sum > INT32_MAX)
		return INT32_MAX;
	if (sum < INT32_MIN)
		return INT32_MIN;
	return (int32_t)sum;
}

static void downsample2(struct impl *this, const void *source)
{
	const int32_t *src = source;
	const uint32_t source_frames = this->source_frames;

	for (uint32_t output_frame = 0;
			output_frame < this->codec_frames; ++output_frame) {
		int source_index = (int)(output_frame * 2u);
		for (unsigned int channel = 0; channel < APTX_ADAPTIVE_CHANNELS; ++channel) {
			int64_t sum = 0;
			for (unsigned int tap = 0; tap < APTX_ADAPTIVE_DOWNSAMPLE_TAPS; ++tap)
				sum += (int64_t)downsample2_coeffs[tap] *
					downsample2_sample(this, src, source_frames, (int)channel,
						source_index - (int)tap);
			this->codec_pcm[(size_t)output_frame * APTX_ADAPTIVE_CHANNELS + channel] =
				scale_filter_sum(sum);
		}
	}

	for (unsigned int channel = 0; channel < APTX_ADAPTIVE_CHANNELS; ++channel)
		for (unsigned int i = 0; i < APTX_ADAPTIVE_DOWNSAMPLE_HISTORY; ++i)
			this->downsample_history[channel][i] = src[
				(size_t)(source_frames - APTX_ADAPTIVE_DOWNSAMPLE_HISTORY + i) *
						APTX_ADAPTIVE_CHANNELS + channel];
}

static int codec_fill_caps(const struct media_codec *codec, uint32_t flags,
		const struct spa_dict *settings, uint8_t caps[A2DP_MAX_CAPS_SIZE])
{
	a2dp_aptx_adaptive_t adaptive_caps;

	(void)settings;
	/* The sink role only needs the capability blob to be advertised so that an
	 * Android source will negotiate aptX Adaptive towards this host; the media
	 * packets are captured at the HCI level by btmon, so decoding is not
	 * required for that use case.  Advertise the same set either way. */
	(void)flags;
	if (!helper_available())
		return -ENOTSUP;

	adaptive_init_caps(&adaptive_caps, APTX_ADAPTIVE_R2_2_SUPPORTED_FEATURES,
			APTX_ADAPTIVE_SAMPLING_FREQ_44100 |
			APTX_ADAPTIVE_SAMPLING_FREQ_48000 |
			APTX_ADAPTIVE_SAMPLING_FREQ_96000,
			APTX_ADAPTIVE_SOURCE_TYPE_2);
	adaptive_caps.info = codec->vendor;
	adaptive_log_caps("local capabilities", &adaptive_caps);
	memcpy(caps, &adaptive_caps, sizeof(adaptive_caps));
	return sizeof(adaptive_caps);
}

static int codec_select_config(const struct media_codec *codec, uint32_t flags,
		const void *caps, size_t caps_size,
		const struct media_codec_audio_info *info,
		const struct spa_dict *settings, uint8_t config[A2DP_MAX_CAPS_SIZE],
		void **config_data)
{
	a2dp_aptx_adaptive_t peer;
	a2dp_aptx_adaptive_t local;
	a2dp_aptx_adaptive_t result;
	const struct adaptive_rate *rate;
	uint32_t requested_rate = info == NULL || info->rate == 0 ?
			48000 : info->rate;
	uint8_t common_freq;
	uint8_t common_channels;
	uint32_t peer_features;
	uint32_t local_features;
	uint32_t negotiated_features;
	uint8_t negotiated_low;
	bool peer_supports_r22;
	bool advertise_r2_2;

	(void)flags;
	(void)settings;
	(void)config_data;
	if (caps == NULL || caps_size < sizeof(peer))
		return -EINVAL;
	memcpy(&peer, caps, sizeof(peer));
	adaptive_log_caps("peer capabilities", &peer);

	if (codec->vendor.vendor_id != peer.info.vendor_id ||
			codec->vendor.codec_id != peer.info.codec_id)
		return -ENOTSUP;
	if (!helper_available() || (rate = find_rate(requested_rate)) == NULL)
		return -ENOTSUP;

	/* Diagnostic override: pin the negotiated codec rate regardless of the
	 * graph rate.  Useful to replicate a working peer configuration (e.g. the
	 * MOMENTUM 5 with an Android source at 44.1 kHz) without touching the
	 * PipeWire clock. */
	{
		const char *forced = getenv("APTX_ADAPTIVE_FORCE_RATE");
		if (forced != NULL && *forced != '\0') {
			const struct adaptive_rate *r = find_rate((uint32_t)atoi(forced));
			if (r != NULL)
				rate = r;
		}
	}

	adaptive_init_caps(&local, APTX_ADAPTIVE_R2_2_SUPPORTED_FEATURES,
			APTX_ADAPTIVE_SAMPLING_FREQ_44100 |
			APTX_ADAPTIVE_SAMPLING_FREQ_48000 |
			APTX_ADAPTIVE_SAMPLING_FREQ_96000,
			APTX_ADAPTIVE_SOURCE_TYPE_2);
	peer_features = adaptive_read_features(&peer);
	peer_supports_r22 = peer.cap_ext_ver_num == APTX_ADAPTIVE_CAP_EXT_VER_NUM &&
			(peer_features & APTX_ADAPTIVE_R2_2_SUPPORT_CAP) != 0;
	advertise_r2_2 = get_advertise_r2_2();
	if (!peer_supports_r22)
		local.sampling_freq_source_type &=
			(uint8_t)~APTX_ADAPTIVE_SAMPLING_FREQ_44100;
	common_freq = adaptive_sampling_freq(&local) &
			adaptive_sampling_freq(&peer);
	/* aptX Lossless is a 44.1 kHz codec.  The MOMENTUM 5 only consumes a
	 * Lossless stream at 44.1 kHz; at 48 kHz it silently stops reading after
	 * ~1 s (observed: A2DP write returns EAGAIN and the link stalls at
	 * ~11 B/s).  Force 44.1 kHz whenever Lossless is enabled and the peer
	 * advertises R2.2 support. */
	if (lossless_enabled() && peer_supports_r22 &&
			(common_freq & APTX_ADAPTIVE_SAMPLING_FREQ_44100)) {
		const struct adaptive_rate *lossless_rate = find_rate(44100);
		if (lossless_rate != NULL)
			rate = lossless_rate;
	}
	if ((common_freq & rate->codec_frequency) != rate->codec_frequency) {
		/* The monitor's default rate is a preference, not a hard capability.
		 * As in the Qualcomm stack, fall back to the best common rate when the
		 * preferred rate is absent from the peer. */
		if (common_freq & APTX_ADAPTIVE_SAMPLING_FREQ_48000)
			rate = find_rate(48000);
		else if (common_freq & APTX_ADAPTIVE_SAMPLING_FREQ_44100)
			rate = find_rate(44100);
		else if (common_freq & APTX_ADAPTIVE_SAMPLING_FREQ_96000)
			rate = find_rate(96000);
		else
			return -ENOTSUP;
	}

	common_channels = peer.channel_mode & local.channel_mode;
	if (common_channels == 0)
		return -ENOTSUP;

	if (info != NULL && info->channels != 0 &&
			info->channels != APTX_ADAPTIVE_CHANNELS)
		return -ENOTSUP;

	result = local;
	adaptive_set_sampling_freq(&result, rate->codec_frequency);
	/* Diagnostic override: the exact meaning of the Adaptive sampling-rate
	 * bits is ambiguous across sources (Qualcomm's A2DP-offload parser uses
	 * 44100=0x08/48000=0x10/88000=0x20/192000=0x40, while the MOMENTUM 5
	 * capability record looks like a different bit set).  Allow pinning the
	 * advertised bits without touching the codec rate so every combination
	 * can be probed. */
	{
		const char *fb = getenv("APTX_ADAPTIVE_FREQ_BITS");
		if (fb != NULL && *fb != '\0') {
			uint8_t bits = (uint8_t)strtoul(fb, NULL, 0);
			result.sampling_freq_source_type =
					(bits & APTX_ADAPTIVE_SAMPLING_FREQ_MASK) |
					(adaptive_source_type(&result) &
					 APTX_ADAPTIVE_SOURCE_TYPE_MASK);
		}
	}
	/* The MOMENTUM 5 advertises both STEREO (0x02) and JOINT_STEREO (0x08) but
	 * only consumes a JOINT_STEREO stream; prefer JOINT_STEREO and keep an
	 * env override so the choice can be flipped without a rebuild. */
	{
		const char *cm = getenv("APTX_ADAPTIVE_CHANNEL_MODE");
		bool want_joint = cm == NULL || !spa_streq(cm, "stereo");
		uint32_t prefer = want_joint ? APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO :
				APTX_ADAPTIVE_CHANNEL_MODE_STEREO;
		uint32_t fallback = want_joint ? APTX_ADAPTIVE_CHANNEL_MODE_STEREO :
				APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO;

		result.channel_mode = (common_channels & prefer) ? prefer : fallback;
	}

	/* Qualcomm negotiates the extension byte instead of blindly copying the
	 * peer's record.  The high nibble is a feature advertisement and the low
	 * nibble is an intersection of source and sink features. */
	local_features = APTX_ADAPTIVE_R2_2_SUPPORTED_FEATURES;
	if (peer.cap_ext_ver_num == 0) {
		result.cap_ext_ver_num = 0;
		memset(result.supported_features, 0,
				sizeof(result.supported_features));
		memset(result.setup_pref, 0, sizeof(result.setup_pref));
		memset(result.eoc, 0, sizeof(result.eoc));
	} else {
		negotiated_low = (uint8_t)((((peer_features >> 4) |
				(local_features >> 4)) << 4) |
				((peer_features & local_features) & 0x0f));
		/* The capability bit above is only a peer advertisement that the
		 * arithmetic above copies into the negotiated record.  When Lossless
		 * is disabled the bridge must not present it to the encoder: the
		 * proprietary R2.2 wrapper then enters its Lossless candidate state
		 * at 44.1 kHz and waits for QHS/16-bit sideband feedback that this
		 * bridge never sends, which stalls the stream after a few packets.
		 * APTX_ADAPTIVE_ADVERTISE_R2_2 keeps the bit in the record the peer
		 * sees while the encoder still runs its ordinary R2 path, which is
		 * what a Qualcomm source looks like from the outside. */
		if (!lossless_enabled() && !advertise_r2_2)
			negotiated_low &=
				(uint8_t)~APTX_ADAPTIVE_R2_2_SUPPORT_CAP;
		negotiated_features = (peer_features & 0xffffff00u) |
				negotiated_low;
		/* Diagnostic override: the Android source talking to this host sends
		 * its own feature word (0x0f000017) rather than this intersection, so
		 * allow pinning it to find what the sink actually needs.  The override
		 * must not undo the safety rule above: with Lossless disabled the
		 * R2.2 capability bit makes the wrapper wait for sideband feedback that
		 * this bridge never sends, which stalls the stream.  Clear it again
		 * instead of relying on the helper's own backstop. */
		{
			const char *fe = getenv("APTX_ADAPTIVE_FEATURES");
			if (fe != NULL && *fe != '\0') {
				uint32_t override = (uint32_t)strtoul(fe, NULL, 0);

				if (!lossless_enabled() && !advertise_r2_2 &&
						(override & APTX_ADAPTIVE_R2_2_SUPPORT_CAP) != 0) {
					override &=
						~(uint32_t)APTX_ADAPTIVE_R2_2_SUPPORT_CAP;
					if (log_ != NULL)
						spa_log_warn(log_,
							"aptX Adaptive: "
							"APTX_ADAPTIVE_FEATURES requested the "
							"R2.2/Lossless bit while Lossless is "
							"disabled; clearing 0x%02x",
							APTX_ADAPTIVE_R2_2_SUPPORT_CAP);
				}
				negotiated_features = override;
			}
		}
		result.cap_ext_ver_num = APTX_ADAPTIVE_CAP_EXT_VER_NUM;
		adaptive_write_features(&result, negotiated_features);
	}

	memcpy(config, &result, sizeof(result));
	if (log_ != NULL)
		spa_log_info(log_,
				"aptX Adaptive selected: requested-rate=%u codec-rate=%u "
				"peer-r22=%d negotiated-channel=0x%02x advertise-r22=%d "
				"lossless-mode=%d",
				requested_rate, rate->codec_rate, peer_supports_r22,
				result.channel_mode, advertise_r2_2,
				lossless_enabled() ? 1 : 0);
	adaptive_log_caps("negotiated configuration", &result);
	return sizeof(result);
}

static int codec_enum_config(const struct media_codec *codec, uint32_t flags,
		const void *caps, size_t caps_size, uint32_t id, uint32_t idx,
		struct spa_pod_builder *b, struct spa_pod **param)
{
	a2dp_aptx_adaptive_t conf;
	struct spa_pod_frame f[2];
	struct spa_pod_choice *choice;
	uint32_t position[2] = { SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR };
	uint32_t count = 0;

	(void)codec;
	(void)flags;
	if (caps == NULL || caps_size < sizeof(conf))
		return -EINVAL;
	if (idx > 0)
		return 0;
	memcpy(&conf, caps, sizeof(conf));

	spa_pod_builder_push_object(b, &f[0], SPA_TYPE_OBJECT_Format, id);
	spa_pod_builder_add(b,
			SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_audio),
			SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
			/* Qualcomm's source path uses 24-bit PCM.  S16 and S32 remain
			 * accepted as explicit graph formats and are widened/narrowed to
			 * the helper's Q27 CAPI input. */
			SPA_FORMAT_AUDIO_format, SPA_POD_CHOICE_ENUM_Id(4,
					SPA_AUDIO_FORMAT_S24_32,
					SPA_AUDIO_FORMAT_S24_32,
					SPA_AUDIO_FORMAT_S16,
					SPA_AUDIO_FORMAT_S32),
			SPA_FORMAT_AUDIO_channels, SPA_POD_Int(APTX_ADAPTIVE_CHANNELS),
			SPA_FORMAT_AUDIO_position, SPA_POD_Array(sizeof(uint32_t),
					SPA_TYPE_Id, SPA_N_ELEMENTS(position), position),
			0);
	spa_pod_builder_prop(b, SPA_FORMAT_AUDIO_rate, 0);
	spa_pod_builder_push_choice(b, &f[1], SPA_CHOICE_None, 0);
	choice = (struct spa_pod_choice *)spa_pod_builder_frame(b, &f[1]);

	for (size_t i = 0; i < SPA_N_ELEMENTS(adaptive_rates); ++i) {
		const struct adaptive_rate *rate = &adaptive_rates[i];
		if ((adaptive_sampling_freq(&conf) & rate->codec_frequency) !=
				rate->codec_frequency)
			continue;
		spa_pod_builder_int(b, (int)rate->graph_rate);
		count++;
	}
	if (count == 0)
		return -ENOTSUP;
	if (count > 1)
		choice->body.type = SPA_CHOICE_Enum;
	spa_pod_builder_pop(b, &f[1]);
	*param = spa_pod_builder_pop(b, &f[0]);
	return *param == NULL ? -EIO : 1;
}

static void codec_deinit(void *data)
{
	struct impl *this = data;
	if (this == NULL)
		return;
	close_fds(this->input_fd, this->output_fd);
	reap_helper(this->pid);
	free(this);
}

static int codec_start_decode(void *data, const void *src, size_t src_size,
		uint16_t *seqnum, uint32_t *timestamp)
{
	struct impl *this = data;
	const struct rtp_header *header = src;
	const size_t header_size = sizeof(struct rtp_header);

	(void)this;
	if (src == NULL || src_size <= header_size)
		return -EINVAL;
	if (seqnum)
		*seqnum = ntohs(header->sequence_number);
	if (timestamp)
		*timestamp = ntohl(header->timestamp);
	return (int)header_size;
}

static int codec_decode(void *data, const void *src, size_t src_size,
		void *dst, size_t dst_size, size_t *dst_out)
{
	struct impl *this = data;
	static int fd = -2;

	(void)this;
	(void)dst;
	(void)dst_size;
	/* Capture-only sink: dump the decrypted aptX Adaptive payload so a
	 * reference bitstream from a working source can be compared with the
	 * one this host produces.  The data is already decrypted by the time it
	 * reaches this callback, which is exactly why this path is used instead
	 * of trying to decrypt an HCI capture. */
	if (fd == -2) {
		const char *path = getenv("APTX_ADAPTIVE_CAPTURE");
		fd = (path != NULL && *path != '\0') ?
				open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644) : -1;
	}
	if (fd >= 0 && src != NULL && src_size > 0)
		(void)write(fd, src, src_size);
	if (dst_out)
		*dst_out = 0;
	return (int)src_size;
}

static void *codec_init(const struct media_codec *codec, uint32_t flags,
		void *config, size_t config_len, const struct spa_audio_info *info,
		void *props, size_t mtu)
{
	struct impl *this;
	a2dp_aptx_adaptive_t conf;
	const struct adaptive_rate *rate;
	uint32_t source_rate;

	(void)codec;
	(void)props;
	if (config == NULL || config_len < sizeof(a2dp_aptx_adaptive_t) ||
			info == NULL ||
			info->media_type != SPA_MEDIA_TYPE_audio ||
			info->media_subtype != SPA_MEDIA_SUBTYPE_raw ||
			(info->info.raw.format != SPA_AUDIO_FORMAT_S16 &&
				info->info.raw.format != SPA_AUDIO_FORMAT_S24_32 &&
				info->info.raw.format != SPA_AUDIO_FORMAT_S32) ||
			info->info.raw.channels != APTX_ADAPTIVE_CHANNELS) {
		errno = ENOTSUP;
		return NULL;
	}
	if ((flags & MEDIA_CODEC_FLAG_SINK) != 0) {
		/* Capture-only sink: the stream must be accepted and consumed so the
		 * source keeps sending, but the payload is deliberately discarded.
		 * btmon sees the packets on the HCI interface regardless. */
		this = calloc(1, sizeof(*this));
		if (this == NULL)
			return NULL;
		this->pid = -1;
		this->input_fd = -1;
		this->output_fd = -1;
		this->sink = true;
		this->mtu = mtu;
		this->pcm_format = info->info.raw.format == SPA_AUDIO_FORMAT_S16 ?
				ADAPTIVE_PCM_S16 :
				(info->info.raw.format == SPA_AUDIO_FORMAT_S24_32 ?
				 ADAPTIVE_PCM_S24_32 : ADAPTIVE_PCM_S32);
		return this;
	}
	if (!helper_available()) {
		errno = ENOTSUP;
		return NULL;
	}
	memcpy(&conf, config, sizeof(conf));
	if (conf.info.vendor_id != APTX_ADAPTIVE_VENDOR_ID ||
			conf.info.codec_id != APTX_ADAPTIVE_CODEC_ID ||
			(adaptive_sampling_freq(&conf) != APTX_ADAPTIVE_SAMPLING_FREQ_44100 &&
			 adaptive_sampling_freq(&conf) != APTX_ADAPTIVE_SAMPLING_FREQ_48000 &&
			 adaptive_sampling_freq(&conf) != APTX_ADAPTIVE_SAMPLING_FREQ_96000) ||
			(conf.channel_mode != APTX_ADAPTIVE_CHANNEL_MODE_STEREO &&
			 conf.channel_mode != APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO)) {
		errno = ENOTSUP;
		return NULL;
	}
	if (info->info.raw.rate == 0) {
		errno = ENOTSUP;
		return NULL;
	}

	source_rate = info->info.raw.rate;
	rate = find_rate(source_rate);
	if (rate == NULL) {
		errno = ENOTSUP;
		return NULL;
	}

	this = calloc(1, sizeof(*this));
	if (this == NULL)
		return NULL;
	this->pid = -1;
	this->input_fd = -1;
	this->output_fd = -1;
	this->source_rate = source_rate;
	this->codec_rate = rate->codec_rate;
	this->mode = get_helper_mode(source_rate);
	this->codec_frames = this->mode == APTX_ADAPTIVE_HELPER_MODE_R3 ?
			APTX_ADAPTIVE_R3_CODEC_FRAMES :
			r2_frames_for_rate(this->codec_rate);
	/* Diagnostic override: the R2 wrapper's real frame length has to match the
	 * block size fed to the helper, otherwise the encoder emits a packet only
	 * every other call and the RTP timestamp jumps.  Allow measuring it
	 * without a rebuild. */
	{
		const char *cf = getenv("APTX_ADAPTIVE_CODEC_FRAMES");
		if (cf != NULL && *cf != '\0') {
			unsigned long v = strtoul(cf, NULL, 0);
			if (v >= 64 && v <= APTX_ADAPTIVE_MAX_SOURCE_FRAMES)
				this->codec_frames = (uint32_t)v;
		}
	}
	this->source_frames = this->codec_frames *
			(source_rate == rate->codec_rate ? 1u : 2u);
	this->helper_bytes = APTX_ADAPTIVE_CHANNELS *
			(APTX_ADAPTIVE_HELPER_BITS_PER_SAMPLE / 8u) * this->codec_frames;
	this->mtu = mtu > 0 ? (int)mtu : 995;
	this->profile = get_profile();
	this->downsample2 = source_rate != rate->codec_rate;
	this->pcm_format = info->info.raw.format == SPA_AUDIO_FORMAT_S16 ?
			ADAPTIVE_PCM_S16 : info->info.raw.format == SPA_AUDIO_FORMAT_S24_32 ?
			ADAPTIVE_PCM_S24_32 : ADAPTIVE_PCM_S32;
	this->source_bytes = this->pcm_format == ADAPTIVE_PCM_S16 ?
			sizeof(int16_t) : sizeof(int32_t);
	this->lossless_mode = get_lossless_mode();
	this->qhs_supported = get_qhs_supported();
	this->abr_enabled = get_abr_enabled();
	this->block_size = (int)(this->source_frames * APTX_ADAPTIVE_CHANNELS *
			this->source_bytes);
	this->abr_level = APTX_ADAPTIVE_ABR_LEVELS - 1;
	this->abr_pending_level = this->abr_level;
	if (log_ != NULL) {
		spa_log_info(log_,
				"aptX Adaptive encoder init: source-rate=%u codec-rate=%u "
				"format=%d block=%d mtu=%d mode=%d lossless=%d qhs=%d",
				this->source_rate, this->codec_rate, this->pcm_format,
				this->block_size, this->mtu, this->mode,
				this->lossless_mode, this->qhs_supported);
		if (this->abr_enabled)
			spa_log_info(log_,
					"aptX Adaptive ABR enabled, but this encoder build "
					"does not honour quality-level feedback in the "
					"standalone helper; the stream stays at a fixed "
					"rate (measured: ~212 kbps R2 at 48 kHz)");
		if (this->lossless_mode != APTX_ADAPTIVE_HELPER_LOSSLESS_OFF)
			spa_log_info(log_,
					"aptX Adaptive Lossless uses the direct R3 encoding "
					"pipeline; QHS sideband feedback is not required");
		if (this->mtu < APTX_ADAPTIVE_MIN_MTU)
			spa_log_warn(log_,
					"aptX Adaptive: transport MTU %d cannot carry an "
					"Adaptive packet (needs at least %d); every packet "
					"will be dropped",
					this->mtu, APTX_ADAPTIVE_MIN_MTU);
		if (this->abr_enabled)
			spa_log_info(log_,
					"aptX Adaptive ABR levels are driven by the local "
					"transport backlog; the controller-side RF/BER "
					"feedback of the reference stack is not available");
	}

	/* convert_to_q27() normalises every supported source format to the 32-bit
	 * Q27 stream the direct R3 pipeline consumes, so R3 works at 44.1/48/96 kHz
	 * for S16, S24_32 and S32 sources alike. */
	build_r2_stream(&conf, this->r2_stream);
	this->pid = spawn_helper(&this->input_fd, &this->output_fd);
	if (this->pid < 0)
		goto error;
	if (initialize_helper(this, config, config_len) < 0)
		goto error;
	return this;

error:
	close_fds(this->input_fd, this->output_fd);
	reap_helper(this->pid);
	free(this);
	errno = ENOTSUP;
	return NULL;
}

static int codec_get_block_size(void *data)
{
	struct impl *this = data;
	return this->block_size;
}

static int codec_start_encode(void *data, void *dst, size_t dst_size,
		uint16_t seqnum, uint32_t timestamp)
{
	struct rtp_header *rtp;

	(void)data;
	if (dst == NULL || dst_size < sizeof(*rtp))
		return -ENOSPC;

	rtp = dst;
	memset(rtp, 0, sizeof(*rtp));
	rtp->v = 2;
	rtp->pt = 96;
	rtp->sequence_number = htons(seqnum);
	rtp->timestamp = htonl(timestamp);
	return sizeof(*rtp);
}

static void convert_to_q27(struct impl *this, const void *source)
{
	const size_t sample_count = (size_t)this->source_frames *
			APTX_ADAPTIVE_CHANNELS;

	switch (this->pcm_format) {
	case ADAPTIVE_PCM_S16: {
		const int16_t *samples = source;
		for (size_t i = 0; i < sample_count; ++i)
			this->source_pcm[i] = (int32_t)((int64_t)samples[i] * 4096);
		break;
	}
	case ADAPTIVE_PCM_S24_32: {
		const int32_t *samples = source;
		for (size_t i = 0; i < sample_count; ++i)
			this->source_pcm[i] = (int32_t)((int64_t)samples[i] * 16);
		break;
	}
	case ADAPTIVE_PCM_S32: {
		const int32_t *samples = source;
		for (size_t i = 0; i < sample_count; ++i)
			this->source_pcm[i] = (int32_t)((int64_t)samples[i] >> 4);
		break;
	}
	}
}

static int codec_encode(void *data, const void *src, size_t src_size,
		void *dst, size_t dst_size, size_t *dst_out, int *need_flush)
{
	struct impl *this = data;
	uint8_t request_size[4];
	uint8_t response[8];
	struct aptx_adaptive_ota_header header;
	const uint8_t *payload;
	size_t consumed;
	const void *helper_src;
	int result;

	if (src == NULL || src_size < (size_t)this->block_size ||
			dst == NULL || dst_out == NULL || need_flush == NULL)
		return -EINVAL;
	if (dst_size == 0)
		return -ENOSPC;

	/* AudioReach receives signed S32/Q27.  Convert all graph formats before
	 * optional 2:1 rate conversion; sending SPA S32 values unchanged would
	 * incorrectly present Q31 samples to a Q27 module. */
	convert_to_q27(this, src);
	helper_src = this->source_pcm;
	if (this->downsample2) {
		downsample2(this, helper_src);
		helper_src = this->codec_pcm;
	}
	/* The helper's CAPI boundary is always S32/Q27. */
	const size_t helper_bytes = this->helper_bytes;
	write_u32le(request_size, (uint32_t)helper_bytes);
	if ((result = write_full(this->input_fd, request_size, sizeof(request_size))) < 0 ||
			(result = write_full(this->input_fd, helper_src,
				helper_bytes)) < 0 ||
			(result = read_full(this->output_fd, response, sizeof(response))) < 0)
		return result;

	uint32_t response_status = read_u32le(response);
	uint32_t response_size = read_u32le(response + 4);
	if (response_status != 0) {
		/* CAPI encoders buffer several input calls before their first
		 * complete OTA packet.  The helper reports that as EAGAIN; the
		 * input block has still been consumed and must not be replayed. */
		if (response_status == EAGAIN) {
			*dst_out = 0;
			*need_flush = NEED_FLUSH_NO;
			return this->block_size;
		}
		return -EIO;
	}
	if (response_size == 0 || response_size > sizeof(this->packet)) {
		if (response_size > sizeof(this->packet) && log_ != NULL)
			spa_log_warn(log_,
					"aptX Adaptive: helper announced a %u byte packet "
					"but the buffer holds %zu; dropping it",
					response_size, sizeof(this->packet));
		*dst_out = 0;
		*need_flush = NEED_FLUSH_NO;
		return this->block_size;
	}
	if ((result = read_full(this->output_fd, this->packet, response_size)) < 0)
		return result;
	if (aptx_adaptive_next_ota_packet(this->packet, response_size, &header,
			&payload, &consumed) < 0 || consumed != response_size)
		return -EBADMSG;
	/* The R2 CAPI output has an eight-byte host/container OTA prefix, but the
	 * Android reference source sends the 656-byte Adaptive codec frame itself
	 * as the RTP payload (the sink callback receives frames beginning 83 00,
	 * with no extra TTP/version prefix).  Keep the old behaviour available for
	 * R3/Lossless, while allowing the ordinary R2 path to strip that wrapper
	 * before it reaches A2DP. */
	const uint8_t *wire_data = this->packet;
	size_t wire_size = response_size;
	if (this->mode == APTX_ADAPTIVE_HELPER_MODE_R2 &&
			env_flag("APTX_ADAPTIVE_STRIP_OTA", false)) {
		wire_data = payload;
		wire_size = response_size - (size_t)(payload - this->packet);
	}
	/* A2DP sends one complete RTP packet per transport write. */
	if (this->mtu > 0 && wire_size + APTX_ADAPTIVE_RTP_HEADER_SIZE >
			(size_t)this->mtu) {
		if (log_ != NULL)
			spa_log_warn(log_,
					"aptX Adaptive: dropping a %zu byte packet that "
					"does not fit the %d byte transport MTU",
					wire_size, this->mtu);
		return -EMSGSIZE;
	}
	if (wire_size > dst_size)
		return -ENOSPC;

	memcpy(dst, wire_data, wire_size);
	*dst_out = wire_size;
	*need_flush = NEED_FLUSH_ALL;
	return this->block_size;
}

static unsigned int abr_level_for_unsent(const struct impl *this, size_t unsent)
{
	size_t mtu = this->mtu > 0 ? (size_t)this->mtu : 995u;
	/* media-sink passes get_transport_unsent_size(): the number of bytes still
	 * queued in the transport socket (fd_buffer_size - free space).  A larger
	 * backlog means the link is draining more slowly, which maps to a lower
	 * quality level.  Levels are 0-based here and become 1-based helper
	 * quality levels in codec_abr_process(). */
	if (unsent <= mtu / 2)
		return APTX_ADAPTIVE_ABR_LEVELS - 1;
	if (unsent <= mtu)
		return APTX_ADAPTIVE_ABR_LEVELS - 2;
	if (unsent <= mtu * 2)
		return APTX_ADAPTIVE_ABR_LEVELS - 3;
	if (unsent <= mtu * 3)
		return APTX_ADAPTIVE_ABR_LEVELS - 4;
	return 0;
}

static int codec_abr_process(void *data, size_t unsent)
{
	struct impl *this = data;
	unsigned int target;
	uint32_t quality_level;
	int result;

	/* NOTE: this control plane is wired exactly like the Qualcomm stack, but
	 * the R2.2 CAPI build used here ignores IMCL quality-level feedback when it
	 * runs without a full AudioReach container.  Measurements show identical
	 * output for level 1 and level 5, so the stream is effectively fixed-rate.
	 * The code is kept because the helper protocol is still exercised and a
	 * different encoder build may honour it. */
	if (!this->abr_enabled)
		return 0;

	target = abr_level_for_unsent(this, unsent);
	if (target == this->abr_level) {
		this->abr_pending_level = target;
		this->abr_pending_count = 0;
		return 0;
	}
	if (target != this->abr_pending_level) {
		this->abr_pending_level = target;
		this->abr_pending_count = 1;
		return 0;
	}
	if (++this->abr_pending_count < APTX_ADAPTIVE_ABR_CONFIRMATIONS)
		return 0;

	/* The socket queue is only a local fallback signal; AX210/BlueZ does not
	 * expose the Qualcomm RF/BER/QHS feedback used by the official stack.  At
	 * least send the result through the official quality-level control plane,
	 * rather than bypassing it with a direct encoder bitrate setter. */
	quality_level = target + 1u;
	result = helper_control(this,
			APTX_ADAPTIVE_HELPER_COMMAND_SET_QUALITY_LEVEL,
			&quality_level, sizeof(quality_level));
	if (result == 0) {
		this->abr_level = target;
		this->abr_pending_count = 0;
	}
	return result;
}

static void codec_get_delay(void *data, uint32_t *encoder, uint32_t *decoder)
{
	struct impl *this = data;

	/* Units are samples: media-sink converts this into nanoseconds with the
	 * graph rate.  The encoder cannot emit a sample before its block is
	 * complete, so one block is genuine latency; LDAC uses the same "one frame"
	 * convention.  The sink's own buffering arrives through DELAY_REPORT and is
	 * accounted for by the transport, not here. */
	if (encoder)
		*encoder = this->codec_frames + (this->downsample2 ? 7u : 0u);
	if (decoder)
		*decoder = 0;
}

static void codec_set_log(struct spa_log *global_log)
{
	log_ = global_log;
	spa_log_topic_init(log_, &codec_plugin_log_topic);
}

const struct media_codec a2dp_codec_aptx_adaptive = {
	.id = SPA_BLUETOOTH_AUDIO_CODEC_APTX_ADAPTIVE,
	.kind = MEDIA_CODEC_A2DP,
	.codec_id = A2DP_CODEC_VENDOR,
	.vendor = { .vendor_id = APTX_ADAPTIVE_VENDOR_ID,
		.codec_id = APTX_ADAPTIVE_CODEC_ID },
	.name = "aptx_adaptive",
	.description = "aptX Adaptive (experimental bridge)",
	.fill_caps = codec_fill_caps,
	.select_config = codec_select_config,
	.enum_config = codec_enum_config,
	.init = codec_init,
	.deinit = codec_deinit,
	.get_block_size = codec_get_block_size,
	.abr_process = codec_abr_process,
	.start_encode = codec_start_encode,
	.encode = codec_encode,
	.start_decode = codec_start_decode,
	.decode = codec_decode,
	.get_delay = codec_get_delay,
	.set_log = codec_set_log,
};

MEDIA_CODEC_EXPORT_DEF("aptx-adaptive", &a2dp_codec_aptx_adaptive);
