/* Spa experimental aptX Adaptive A2DP codec bridge */
/* SPDX-License-Identifier: MIT */

#include "config.h"

#include <errno.h>
#include <limits.h>
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
 * This bridge is deliberately opt-in at runtime.  PipeWire's real-time
 * thread talks to a long-lived helper over two pipes; the helper is expected
 * to be a user-supplied Hexagon/QEMU process containing a licensed Qualcomm
 * encoder.  No proprietary library is linked into this plugin.
 */
#define APTX_ADAPTIVE_HELPER_ENV "PIPEWIRE_APTX_ADAPTIVE_HELPER"
#define APTX_ADAPTIVE_QEMU_ENV "PIPEWIRE_APTX_ADAPTIVE_QEMU"
#define APTX_ADAPTIVE_SYSROOT_ENV "PIPEWIRE_APTX_ADAPTIVE_SYSROOT"

#define APTX_ADAPTIVE_RATE 48000
#define APTX_ADAPTIVE_CHANNELS 2
#define APTX_ADAPTIVE_BITS_PER_SAMPLE 32
#define APTX_ADAPTIVE_BLOCK_SIZE \
	(APTX_ADAPTIVE_CHANNELS * (APTX_ADAPTIVE_BITS_PER_SAMPLE / 8) * 672)
#define APTX_ADAPTIVE_MAX_PACKET_SIZE 2048

SPA_STATIC_ASSERT(sizeof(a2dp_aptx_adaptive_t) == 40);

struct impl {
	pid_t pid;
	int input_fd;
	int output_fd;
	int block_size;
};

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
		p += n;
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
		p += n;
		size -= (size_t)n;
	}
	return 0;
}

static uint32_t read_u32le(const uint8_t data[4])
{
	return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
			(uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
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

static int codec_fill_caps(const struct media_codec *codec, uint32_t flags,
		const struct spa_dict *settings, uint8_t caps[A2DP_MAX_CAPS_SIZE])
{
	static const a2dp_aptx_adaptive_t adaptive_caps = {
		.info = {
			.vendor_id = APTX_ADAPTIVE_VENDOR_ID,
			.codec_id = APTX_ADAPTIVE_CODEC_ID,
		},
		.sampling_freq = APTX_ADAPTIVE_SAMPLING_FREQ_48000,
		.channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO,
	};

	(void)codec;
	(void)flags;
	(void)settings;
	if (!helper_available())
		return -ENOTSUP;

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

	(void)flags;
	(void)settings;
	(void)config_data;
	if (caps == NULL || caps_size < sizeof(conf))
		return -EINVAL;

	memcpy(&conf, caps, sizeof(conf));
	if (codec->vendor.vendor_id != conf.info.vendor_id ||
			codec->vendor.codec_id != conf.info.codec_id)
		return -ENOTSUP;
	if (!helper_available() ||
			(info != NULL && (info->rate != APTX_ADAPTIVE_RATE ||
					info->channels != APTX_ADAPTIVE_CHANNELS)))
		return -ENOTSUP;
	if (!(conf.sampling_freq & APTX_ADAPTIVE_SAMPLING_FREQ_48000) ||
			!(conf.channel_mode & APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO))
		return -ENOTSUP;

	conf.sampling_freq = APTX_ADAPTIVE_SAMPLING_FREQ_48000;
	conf.channel_mode = APTX_ADAPTIVE_CHANNEL_MODE_JOINT_STEREO;
	memcpy(config, &conf, sizeof(conf));
	return sizeof(conf);
}

static int codec_enum_config(const struct media_codec *codec, uint32_t flags,
		const void *caps, size_t caps_size, uint32_t id, uint32_t idx,
		struct spa_pod_builder *b, struct spa_pod **param)
{
	struct spa_audio_info_raw info = { 0 };

	(void)codec;
	(void)flags;
	(void)caps;
	(void)caps_size;
	if (idx > 0)
		return 0;

	info.format = SPA_AUDIO_FORMAT_S32;
	info.channels = APTX_ADAPTIVE_CHANNELS;
	info.position[0] = SPA_AUDIO_CHANNEL_FL;
	info.position[1] = SPA_AUDIO_CHANNEL_FR;
	info.rate = APTX_ADAPTIVE_RATE;
	*param = spa_format_audio_raw_build(b, id, &info);
	return *param == NULL ? -EIO : 1;
}

static void *codec_init(const struct media_codec *codec, uint32_t flags,
		void *config, size_t config_len, const struct spa_audio_info *info,
		void *props, size_t mtu)
{
	struct impl *this;

	(void)codec;
	(void)flags;
	(void)config;
	(void)props;
	(void)mtu;
	if (config == NULL || config_len < sizeof(a2dp_aptx_adaptive_t) ||
			info == NULL || info->media_type != SPA_MEDIA_TYPE_audio ||
			info->media_subtype != SPA_MEDIA_SUBTYPE_raw ||
			info->info.raw.format != SPA_AUDIO_FORMAT_S32 ||
			info->info.raw.rate != APTX_ADAPTIVE_RATE ||
			info->info.raw.channels != APTX_ADAPTIVE_CHANNELS ||
			!helper_available()) {
		errno = ENOTSUP;
		return NULL;
	}

	this = calloc(1, sizeof(*this));
	if (this == NULL)
		return NULL;
	this->input_fd = -1;
	this->output_fd = -1;
	this->block_size = APTX_ADAPTIVE_BLOCK_SIZE;
	this->pid = spawn_helper(&this->input_fd, &this->output_fd);
	if (this->pid < 0)
		goto error;
	uint8_t ready[8];
	if (read_full(this->output_fd, ready, sizeof(ready)) < 0 ||
			read_u32le(ready) != 0 || read_u32le(ready + 4) != 0)
		goto error;
	return this;

error:
	close_fds(this->input_fd, this->output_fd);
	reap_helper(this->pid);
	free(this);
	errno = EIO;
	return NULL;
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
	uint8_t packet[APTX_ADAPTIVE_MAX_PACKET_SIZE];
	struct aptx_adaptive_ota_header header;
	const uint8_t *payload;
	size_t consumed;
	int res;

	if (src == NULL || src_size < (size_t)this->block_size ||
			dst == NULL || dst_out == NULL || need_flush == NULL)
		return -EINVAL;
	if (dst_size == 0)
		return -ENOSPC;

	request_size[0] = this->block_size & 0xff;
	request_size[1] = (this->block_size >> 8) & 0xff;
	request_size[2] = (this->block_size >> 16) & 0xff;
	request_size[3] = (this->block_size >> 24) & 0xff;
	if ((res = write_full(this->input_fd, request_size, sizeof(request_size))) < 0 ||
			(res = write_full(this->input_fd, src, this->block_size)) < 0 ||
			(res = read_full(this->output_fd, &response, sizeof(response))) < 0)
		return res;

	uint32_t response_status = read_u32le(response);
	uint32_t response_size = read_u32le(response + 4);
	if (response_status != 0 || response_size == 0 ||
			response_size > sizeof(packet))
		return -EIO;
	if (response_size > dst_size)
		return -ENOSPC;
	if ((res = read_full(this->output_fd, packet, response_size)) < 0)
		return res;
	if (aptx_adaptive_next_ota_packet(packet, response_size, &header,
			&payload, &consumed) < 0 || consumed != response_size)
		return -EBADMSG;

	memcpy(dst, packet, response_size);
	*dst_out = response_size;
	*need_flush = NEED_FLUSH_ALL;
	return this->block_size;
}

static int codec_abr_process(void *data, size_t unsent)
{
	(void)data;
	(void)unsent;
	return -ENOTSUP;
}

static void codec_get_delay(void *data, uint32_t *encoder, uint32_t *decoder)
{
	(void)data;
	if (encoder)
		*encoder = 0;
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
