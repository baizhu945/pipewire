/* Spa experimental aptX Adaptive A2DP codec bridge */
/* SPDX-License-Identifier: MIT */

#include "config.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <spa/param/audio/format.h>
#include <spa/param/audio/format-utils.h>
#include <spa/utils/string.h>

#include <aptxadaptive.h>

#include "a2dp-codec-caps.h"
#include "media-codecs.h"

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
#define APTX_ADAPTIVE_STREAM_ENV "APTX_ADAPTIVE_CONFIG_STREAM_HEX"
#define APTX_ADAPTIVE_LOSSLESS_ENV "APTX_ADAPTIVE_LOSSLESS"
#define APTX_ADAPTIVE_QHS_ENV "APTX_ADAPTIVE_QHS_SUPPORT"
#define APTX_ADAPTIVE_ABR_ENV "APTX_ADAPTIVE_ABR"

#define APTX_ADAPTIVE_CHANNELS 2u
#define APTX_ADAPTIVE_HELPER_BITS_PER_SAMPLE 32u
#define APTX_ADAPTIVE_CODEC_FRAMES 672u
#define APTX_ADAPTIVE_CODEC_BYTES \
	(APTX_ADAPTIVE_CHANNELS * (APTX_ADAPTIVE_HELPER_BITS_PER_SAMPLE / 8u) * \
	 APTX_ADAPTIVE_CODEC_FRAMES)
#define APTX_ADAPTIVE_MAX_PACKET_SIZE 4096u
#define APTX_ADAPTIVE_MAX_SOURCE_FRAMES (APTX_ADAPTIVE_CODEC_FRAMES * 2u)

#define APTX_ADAPTIVE_ABR_LEVELS 5u
#define APTX_ADAPTIVE_ABR_CONFIRMATIONS 3u

/* A causal 2:1 half-band filter.  It is used only for 88.2->44.1 and
 * 192->96 graph formats, because the available Qualcomm CAPI build accepts
 * 44.1, 48 and 96 kHz as codec-native input rates. */
#define APTX_ADAPTIVE_DOWNSAMPLE_TAPS 15u
#define APTX_ADAPTIVE_DOWNSAMPLE_HISTORY (APTX_ADAPTIVE_DOWNSAMPLE_TAPS - 1u)
static const int32_t downsample2_coeffs[APTX_ADAPTIVE_DOWNSAMPLE_TAPS] = {
	0, 188, 0, -1595, 0, 9590, 0, 16384,
	0, 9590, 0, -1595, 0, 188, 0,
};

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

SPA_STATIC_ASSERT(sizeof(a2dp_aptx_adaptive_t) == 40);

struct impl {
	pid_t pid;
	int input_fd;
	int output_fd;

	uint32_t source_rate;
	uint32_t codec_rate;
	uint32_t source_frames;
	int block_size;
	int mtu;
	enum aptx_adaptive_helper_mode mode;
	uint32_t profile;
	bool downsample2;
	bool input_s16;
	uint32_t source_bytes;
	enum aptx_adaptive_helper_lossless_mode lossless_mode;
	bool qhs_supported;

	int32_t downsample_history[APTX_ADAPTIVE_CHANNELS]
		[APTX_ADAPTIVE_DOWNSAMPLE_HISTORY];
	int32_t source_pcm[APTX_ADAPTIVE_MAX_SOURCE_FRAMES * APTX_ADAPTIVE_CHANNELS];
	int32_t codec_pcm[APTX_ADAPTIVE_CODEC_FRAMES * APTX_ADAPTIVE_CHANNELS];
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

	if (value == NULL || spa_streq(value, "auto") || spa_streq(value, "AUTO"))
		return APTX_ADAPTIVE_HELPER_LOSSLESS_AUTO;
	if (spa_streq(value, "force") || spa_streq(value, "FORCE"))
		return APTX_ADAPTIVE_HELPER_LOSSLESS_FORCE;
	return APTX_ADAPTIVE_HELPER_LOSSLESS_OFF;
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
	while (size > 0) {
		ssize_t n = read(fd, p, size);
		if (n == 0)
			return -EPIPE;
		if (n < 0) {
			if (errno == EINTR)
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
	while (size > 0) {
		ssize_t n = write(fd, p, size);
		if (n < 0) {
			if (errno == EINTR)
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
	if (pid <= 0)
		return;
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
		close(input_pipe[1]);
		close(output_pipe[0]);
		if (dup2(input_pipe[0], STDIN_FILENO) < 0 ||
				dup2(output_pipe[1], STDOUT_FILENO) < 0)
			_exit(127);
		close(input_pipe[0]);
		close(output_pipe[1]);
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

	if (payload_size > UINT32_MAX)
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
	static const uint8_t default_stream[APTX_ADAPTIVE_HELPER_R2_STREAM_SIZE] = {
		1, 151, 0, 0, 15, 2, 3, 3, 3, 0, 170,
	};
	const char *value = getenv(APTX_ADAPTIVE_STREAM_ENV);

	if (value != NULL) {
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
	memcpy(stream, default_stream, sizeof(default_stream));
	return 0;
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
	config.mtu = this->mtu > 0 ? (uint32_t)this->mtu : 995u;
	config.abr_enabled = this->abr_enabled ? 1u : 0u;
	config.bits_per_sample = this->input_s16 ? 16u : 32u;
	config.lossless_mode = this->lossless_mode;
	config.qhs_supported = this->qhs_supported ? 1u : 0u;
	config.cie_size = APTX_ADAPTIVE_HELPER_CIE_SIZE;
	memcpy(config.cie, codec_config,
			SPA_MIN(codec_config_size, sizeof(config.cie)));
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
			output_frame < APTX_ADAPTIVE_CODEC_FRAMES; ++output_frame) {
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
	a2dp_aptx_adaptive_t adaptive_caps = { 0 };

	(void)flags;
	(void)settings;
	if (!helper_available())
		return -ENOTSUP;

	adaptive_caps.info = codec->vendor;
	adaptive_caps.sampling_freq =
			APTX_ADAPTIVE_SAMPLING_FREQ_44100 |
			APTX_ADAPTIVE_SAMPLING_FREQ_48000 |
			APTX_ADAPTIVE_SAMPLING_FREQ_96000;
	adaptive_caps.channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_CAPABILITIES;
	memcpy(caps, &adaptive_caps, sizeof(adaptive_caps));
	return sizeof(adaptive_caps);
}

static int codec_select_config(const struct media_codec *codec, uint32_t flags,
		const void *caps, size_t caps_size,
		const struct media_codec_audio_info *info,
		const struct spa_dict *settings, uint8_t config[A2DP_MAX_CAPS_SIZE],
		void **config_data)
{
	a2dp_aptx_adaptive_t conf;
	const struct adaptive_rate *rate;
	uint32_t requested_rate = info == NULL || info->rate == 0 ?
			48000 : info->rate;

	(void)flags;
	(void)settings;
	(void)config_data;
	if (caps == NULL || caps_size < sizeof(conf))
		return -EINVAL;
	memcpy(&conf, caps, sizeof(conf));

	if (codec->vendor.vendor_id != conf.info.vendor_id ||
			codec->vendor.codec_id != conf.info.codec_id)
		return -ENOTSUP;
	if (!helper_available() || (rate = find_rate(requested_rate)) == NULL)
		return -ENOTSUP;
	if ((conf.sampling_freq & rate->codec_frequency) != rate->codec_frequency)
		return -ENOTSUP;

	conf.sampling_freq = rate->codec_frequency;
	if (conf.channel_mode & APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO)
		conf.channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO;
	else if (conf.channel_mode & APTX_ADAPTIVE_CHANNEL_MODE_STEREO)
		conf.channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_STEREO;
	else
		return -ENOTSUP;
	if (info != NULL && info->channels != 0 &&
			info->channels != APTX_ADAPTIVE_CHANNELS)
		return -ENOTSUP;

	memcpy(config, &conf, sizeof(conf));
	return sizeof(conf);
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
			/* S16 is listed first so 44.1 kHz can preserve the exact sample
			 * word required by the Lossless candidate.  S32 remains available
			 * for ordinary Adaptive/high-resolution streams. */
			SPA_FORMAT_AUDIO_format, SPA_POD_CHOICE_ENUM_Id(3,
					SPA_AUDIO_FORMAT_S16,
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
		if ((conf.sampling_freq & rate->codec_frequency) != rate->codec_frequency)
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

static void *codec_init(const struct media_codec *codec, uint32_t flags,
		void *config, size_t config_len, const struct spa_audio_info *info,
		void *props, size_t mtu)
{
	struct impl *this;
	const struct adaptive_rate *rate;
	uint32_t source_rate;

	(void)codec;
	(void)flags;
	(void)props;
	if (config == NULL || config_len < sizeof(a2dp_aptx_adaptive_t) ||
			info == NULL || info->media_type != SPA_MEDIA_TYPE_audio ||
			info->media_subtype != SPA_MEDIA_SUBTYPE_raw ||
				(info->info.raw.format != SPA_AUDIO_FORMAT_S16 &&
					info->info.raw.format != SPA_AUDIO_FORMAT_S32) ||
			info->info.raw.channels != APTX_ADAPTIVE_CHANNELS ||
			!helper_available()) {
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
	this->source_frames = APTX_ADAPTIVE_CODEC_FRAMES *
			(source_rate == rate->codec_rate ? 1u : 2u);
	this->mtu = mtu > 0 ? (int)mtu : 995;
	this->mode = get_helper_mode(source_rate);
	this->profile = get_profile();
	this->downsample2 = source_rate != rate->codec_rate;
	this->input_s16 = info->info.raw.format == SPA_AUDIO_FORMAT_S16;
	this->source_bytes = this->input_s16 ? sizeof(int16_t) : sizeof(int32_t);
	this->lossless_mode = get_lossless_mode();
	this->qhs_supported = get_qhs_supported();
	this->abr_enabled = get_abr_enabled();
	this->block_size = (int)(this->source_frames * APTX_ADAPTIVE_CHANNELS *
			this->source_bytes);
	this->abr_level = APTX_ADAPTIVE_ABR_LEVELS - 1;
	this->abr_pending_level = this->abr_level;

	if (this->mode == APTX_ADAPTIVE_HELPER_MODE_R3 &&
			(source_rate != 48000 || this->input_s16))
		goto error;
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
	(void)data;
	(void)dst;
	(void)dst_size;
	(void)seqnum;
	(void)timestamp;
	return 0;
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
	const void *helper_src = src;
	int result;

	if (src == NULL || src_size < (size_t)this->block_size ||
			dst == NULL || dst_out == NULL || need_flush == NULL)
		return -EINVAL;
	if (dst_size == 0)
		return -ENOSPC;

	if (this->input_s16) {
		const int16_t *samples = src;
		size_t sample_count = (size_t)this->source_frames *
				APTX_ADAPTIVE_CHANNELS;

		/* The Qualcomm CAPI input is a signed S32/Q27 stream.  Multiplication
		 * by 2^12 widens a signed 16-bit source exactly, without discarding
		 * any source bit; the source word size is also sent in the helper's
		 * Lossless feedback configuration. */
		for (size_t i = 0; i < sample_count; ++i)
			this->source_pcm[i] = (int32_t)samples[i] * (1 << 12);
		helper_src = this->source_pcm;
	}
	if (this->downsample2) {
		downsample2(this, helper_src);
		helper_src = this->codec_pcm;
	}
	/* The helper's CAPI boundary is always S32/Q27, including widened S16
	 * source audio. */
	const size_t helper_bytes = APTX_ADAPTIVE_CODEC_BYTES;
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
		*dst_out = 0;
		*need_flush = NEED_FLUSH_NO;
		return this->block_size;
	}
	if ((result = read_full(this->output_fd, this->packet, response_size)) < 0)
		return result;
	if (aptx_adaptive_next_ota_packet(this->packet, response_size, &header,
			&payload, &consumed) < 0 || consumed != response_size)
		return -EBADMSG;
	/* A2DP sends one complete vendor packet per transport write.  Adaptive
	 * OTA has no generic PipeWire fragmentation marker, so never hand a
	 * packet larger than the negotiated L2CAP MTU to spa_bt_send(). */
	if (this->mtu > 0 && response_size > (size_t)this->mtu)
		return -EMSGSIZE;
	if (response_size > dst_size)
		return -ENOSPC;

	memcpy(dst, this->packet, response_size);
	*dst_out = response_size;
	*need_flush = NEED_FLUSH_ALL;
	return this->block_size;
}

static unsigned int abr_level_for_unsent(const struct impl *this, size_t unsent)
{
	size_t mtu = this->mtu > 0 ? (size_t)this->mtu : 995u;
	/* media-sink reports free socket space, so less free space means a lower
	 * link-quality level. */
	if (unsent <= mtu / 2)
		return 0;
	if (unsent <= mtu)
		return 1;
	if (unsent <= mtu * 2)
		return 2;
	if (unsent <= mtu * 3)
		return 3;
	return 4;
}

static int codec_abr_process(void *data, size_t unsent)
{
	struct impl *this = data;
	unsigned int target;
	uint32_t quality_level;
	int result;

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
	if (encoder)
		*encoder = this->downsample2 ? 7 : 0;
	if (decoder)
		*decoder = 0;
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
	.get_delay = codec_get_delay,
};

MEDIA_CODEC_EXPORT_DEF("aptx-adaptive", &a2dp_codec_aptx_adaptive);
