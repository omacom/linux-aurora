// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * DCP Audio Bits
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * TODO:
 *  - figure some nice identification of the sound card (in case
 *    there's many DCP instances)
 */

#define DEBUG

#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/of_dma.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/jack.h>

#include "av.h"
#include "dcp.h"
#include "dcp-internal.h"
#include "audio.h"
#include "parser.h"

#define DCPAUD_ELEMENTS_MAXSIZE		16384
#define DCPAUD_PRODUCTATTRS_MAXSIZE	1024
/* The native element catalog capacity (14.7 external role). */
#define DCPAUD_ELEMENTS_V14_7		SZ_256K

/*
 * SIO takes one of its 64 descriptor slots per period, shared by all its
 * channels, and up to three DP audio outputs use one SIO. A stream with
 * more periods fails START with -ENOMEM.
 */
#define DCPAUD_PERIODS_MAX		16

struct dcp_audio {
	struct device *dev;
	struct device *dcp_dev;
	struct device *dma_dev;
	struct device_link *dma_link;
	struct dma_chan *chan;
	struct snd_card *card;
	struct snd_jack *jack;
	struct snd_pcm_substream *substream;
	unsigned int open_cookie;
	bool open_unplugged;

	struct mutex data_lock;
	bool dcp_connected; /// dcp status keep for delayed initialization
	bool connected;
	unsigned int connection_cookie;

	struct snd_pcm_chmap_elem selected_chmap;
	struct dcp_sound_cookie selected_cookie;
	/* stream runs without a sink, see dcpaud_void_start() */
	bool void_running;
	ktime_t void_start;
	u64 void_periods;
	struct delayed_work void_work;
	void *elements;
	void *productattrs;

	struct snd_pcm_chmap *chmap_info;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/*
	 * DCP firmware 14.7, external role (T6030 external processor): the
	 * native link lifecycle, one link per stream start
	 * (dcp_native_playback_ops).
	 */
	bool native_link;
	struct mutex link_lock;
	bool link_prepared;
	bool link_started;
#endif
};

/* The 14.7 external role's audio service (DRM_APPLE_DCP_T6030_AUDIO only). */
static inline bool dcpaud_native_link(struct dcp_audio *dcpaud)
{
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	return dcpaud->native_link;
#else
	return false;
#endif
}

static inline size_t dcpaud_elements_size(struct dcp_audio *dcpaud)
{
	return dcpaud_native_link(dcpaud) ? DCPAUD_ELEMENTS_V14_7 : DCPAUD_ELEMENTS_MAXSIZE;
}

static const struct snd_pcm_hardware dcp_pcm_hw = {
	.info	 = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		   SNDRV_PCM_INFO_INTERLEAVED,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_LE |
		   SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min		= 0,
	.rate_max		= UINT_MAX,
	.channels_min		= 1,
	.channels_max		= 16,
	.buffer_bytes_max	= SIZE_MAX,
	.period_bytes_min	= 4096, /* TODO */
	.period_bytes_max	= SIZE_MAX,
	.periods_min		= 2,
	.periods_max		= DCPAUD_PERIODS_MAX,
};

/*
 * Without a sink the DCP has no audio elements to derive constraints from.
 * Offer a plain stereo format so that the PCM can still be opened and
 * configured, as HDMI PCMs on other platforms can. Userspace (PipeWire,
 * PulseAudio) probes the PCM once when the card appears and drops the
 * output for good if that fails, so a monitor attached later would never
 * show up. The stream then plays into the void, see dcpaud_void_start().
 */
static const struct snd_pcm_hardware dcp_pcm_hw_unplugged = {
	.info	 = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		   SNDRV_PCM_INFO_INTERLEAVED,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				  SNDRV_PCM_RATE_48000,
	.rate_min		= 32000,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= SIZE_MAX,
	.period_bytes_min	= 4096,
	.period_bytes_max	= SIZE_MAX,
	.periods_min		= 2,
	.periods_max		= DCPAUD_PERIODS_MAX,
};

static bool dcpaud_h17p(struct dcp_audio *dcpaud)
{
	struct apple_dcp *dcp = dev_get_drvdata(dcpaud->dcp_dev);

	return dcp && dcp->fw_compat == DCP_FIRMWARE_H17P;
}

static int dcpaud_read_remote_info(struct dcp_audio *dcpaud)
{
	int ret;

	ret = dcp_audiosrv_get_elements(dcpaud->dcp_dev, dcpaud->elements,
					dcpaud_elements_size(dcpaud));
	if (ret < 0)
		return ret;

	ret = dcp_audiosrv_get_product_attrs(dcpaud->dcp_dev, dcpaud->productattrs,
					     DCPAUD_PRODUCTATTRS_MAXSIZE);
	if (ret < 0)
		return ret;

	return 0;
}

static int dcpaud_interval_bitmask(struct snd_interval *i,
				   unsigned int mask)
{
	struct snd_interval range;
	if (!mask)
		return -EINVAL;

	snd_interval_any(&range);
	range.min = __ffs(mask);
	range.max = __fls(mask);
	return snd_interval_refine(i, &range);
}

extern const struct snd_pcm_hw_constraint_list snd_pcm_known_rates;

static void dcpaud_fill_fmt_sieve(struct snd_pcm_hw_params *params,
				  struct dcp_sound_format_mask *sieve)
{
	struct snd_interval *c = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *r = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_RATE);
	struct snd_mask *f = hw_param_mask(params,
				SNDRV_PCM_HW_PARAM_FORMAT);
	int i;

	sieve->nchans = GENMASK(c->max, c->min);
	sieve->formats = f->bits[0] | ((u64) f->bits[1]) << 32; /* TODO: don't open-code */

	for (i = 0; i < snd_pcm_known_rates.count; i++) {
		unsigned int rate = snd_pcm_known_rates.list[i];

		if (snd_interval_test(r, rate))
			sieve->rates |= 1u << i;
	}
}

static void dcpaud_consult_elements(struct dcp_audio *dcpaud,
				    struct snd_pcm_hw_params *params,
				    struct dcp_sound_format_mask *hits)
{
	struct dcp_sound_format_mask sieve;
	struct dcp_parse_ctx elements = {
		.dcp = dev_get_drvdata(dcpaud->dcp_dev),
		.blob = dcpaud->elements + 4,
		.len = dcpaud_elements_size(dcpaud) - 4,
		.pos = 0,
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		.v14_7_audio = dcpaud->native_link,
#endif
	};

	dcpaud_fill_fmt_sieve(params, &sieve);
	dev_dbg(dcpaud->dev, "elements in: %llx %x %x\n", sieve.formats, sieve.nchans, sieve.rates);
	parse_sound_constraints(&elements, &sieve, hits);
	dev_dbg(dcpaud->dev, "elements out: %llx %x %x\n", hits->formats, hits->nchans, hits->rates);
}

static int dcpaud_select_cookie(struct dcp_audio *dcpaud,
				 struct snd_pcm_hw_params *params)
{
	struct dcp_sound_format_mask sieve;
	struct dcp_parse_ctx elements = {
		.dcp = dev_get_drvdata(dcpaud->dcp_dev),
		.blob = dcpaud->elements + 4,
		.len = dcpaud_elements_size(dcpaud) - 4,
		.pos = 0,
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		.v14_7_audio = dcpaud->native_link,
#endif
	};

	dcpaud_fill_fmt_sieve(params, &sieve);
	return parse_sound_mode(&elements, &sieve, &dcpaud->selected_chmap,
				&dcpaud->selected_cookie);
}

static int dcpaud_rule_channels(struct snd_pcm_hw_params *params,
                                struct snd_pcm_hw_rule *rule)
{
	struct dcp_audio *dcpaud = rule->private;
	struct snd_interval *c = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_CHANNELS);
	struct dcp_sound_format_mask hits = {0, 0, 0};

        dcpaud_consult_elements(dcpaud, params, &hits);

        return dcpaud_interval_bitmask(c, hits.nchans);
}

static int dcpaud_refine_fmt_mask(struct snd_mask *m, u64 mask)
{
	struct snd_mask mask_mask;

	if (!mask)
		return -EINVAL;
	mask_mask.bits[0] = mask;
	mask_mask.bits[1] = mask >> 32;

	return snd_mask_refine(m, &mask_mask);
}

static int dcpaud_rule_format(struct snd_pcm_hw_params *params,
                               struct snd_pcm_hw_rule *rule)
{
	struct dcp_audio *dcpaud = rule->private;
	struct snd_mask *f = hw_param_mask(params,
				SNDRV_PCM_HW_PARAM_FORMAT);
	struct dcp_sound_format_mask hits;

        dcpaud_consult_elements(dcpaud, params, &hits);

        return dcpaud_refine_fmt_mask(f, hits.formats);
}

static int dcpaud_rule_rate(struct snd_pcm_hw_params *params,
                             struct snd_pcm_hw_rule *rule)
{
	struct dcp_audio *dcpaud = rule->private;
	struct snd_interval *r = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_RATE);
	struct dcp_sound_format_mask hits;

        dcpaud_consult_elements(dcpaud, params, &hits);

        return snd_interval_rate_bits(r, hits.rates);
}

static int dcpaud_init_dma(struct dcp_audio *dcpaud)
{
	struct dma_chan *chan;
	if (dcpaud->chan)
		return 0;

	chan = of_dma_request_slave_channel(dcpaud->dev->of_node, "tx");
	/* squelch dma channel request errors, the driver will try again alter */
	if (!chan) {
		dev_warn(dcpaud->dev, "audio TX DMA channel request failed\n");
		return -ENXIO;
	} else if (chan == ERR_PTR(-EPROBE_DEFER)) {
		dev_info(dcpaud->dev, "audio TX DMA channel is not ready yet\n");
		return -ENXIO;
	} else if (IS_ERR(chan)) {
		dev_warn(dcpaud->dev, "audio TX DMA channel request failed: %ld\n", PTR_ERR(chan));
		return PTR_ERR(chan);
	}
	dcpaud->chan = chan;

	snd_pcm_set_managed_buffer(dcpaud->substream, SNDRV_DMA_TYPE_DEV_IRAM,
				   dcpaud->chan->device->dev, 1024 * 1024,
				   SIZE_MAX);

	return 0;
}

static int dcp_pcm_open(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	struct snd_dmaengine_dai_dma_data dma_data = {
		.flags = SND_DMAENGINE_PCM_DAI_FLAG_PACK,
	};
	struct snd_pcm_hardware hw;
	int ret;

	mutex_lock(&dcpaud->data_lock);
	ret = dcpaud_init_dma(dcpaud);
	if (ret < 0) {
		mutex_unlock(&dcpaud->data_lock);
		return ret;
	}

	/* H17P (T8140): the PCM opens without a sink, as on aurora-wip. */
	dcpaud->open_unplugged = !dcpaud->connected && dcpaud_h17p(dcpaud);
	if (!dcpaud->connected && !dcpaud->open_unplugged) {
		mutex_unlock(&dcpaud->data_lock);
		return -ENXIO;
	}
	dcpaud->open_cookie = dcpaud->connection_cookie;
	mutex_unlock(&dcpaud->data_lock);

	if (dcpaud->open_unplugged) {
		hw = dcp_pcm_hw_unplugged;
		goto refine;
	}

	ret = dcpaud_read_remote_info(dcpaud);
	if (ret < 0)
		return ret;

	snd_pcm_hw_rule_add(substream->runtime, 0, SNDRV_PCM_HW_PARAM_FORMAT,
			    dcpaud_rule_format, dcpaud,
			    SNDRV_PCM_HW_PARAM_CHANNELS, SNDRV_PCM_HW_PARAM_RATE, -1);
	snd_pcm_hw_rule_add(substream->runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
			    dcpaud_rule_channels, dcpaud,
			    SNDRV_PCM_HW_PARAM_FORMAT, SNDRV_PCM_HW_PARAM_RATE, -1);
	snd_pcm_hw_rule_add(substream->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			    dcpaud_rule_rate, dcpaud,
			    SNDRV_PCM_HW_PARAM_FORMAT, SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	hw = dcp_pcm_hw;
refine:
	hw.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_INTERLEAVED;
	hw.periods_min = 2;
	hw.periods_max = DCPAUD_PERIODS_MAX;
	hw.period_bytes_min = 256;
	hw.period_bytes_max = SIZE_MAX; // TODO dma_get_max_seg_size(dma_dev);
	hw.buffer_bytes_max = SIZE_MAX;
	hw.fifo_size = 16;
	ret = snd_dmaengine_pcm_refine_runtime_hwparams(substream, &dma_data,
							&hw, dcpaud->chan);
	if (ret)
		return ret;
	substream->runtime->hw = hw;

	return snd_dmaengine_pcm_open(substream, dcpaud->chan);
}

static int dcp_pcm_close(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	dcpaud->selected_chmap.channels = 0;
	dcpaud->open_unplugged = false;

	return snd_dmaengine_pcm_close(substream);
}

static int dcpaud_connection_up(struct dcp_audio *dcpaud)
{
	bool ret;
	mutex_lock(&dcpaud->data_lock);
	ret = dcpaud->connected &&
	      dcpaud->open_cookie == dcpaud->connection_cookie;
	mutex_unlock(&dcpaud->data_lock);
	return ret;
}

static int dcp_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	struct dma_slave_config slave_config;
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	int ret;

	/* no sink: the stream runs without DMA, see dcpaud_void_start() */
	if (dcpaud->open_unplugged || !dcpaud_connection_up(dcpaud))
		return 0;

	ret = dcpaud_select_cookie(dcpaud, params);
	if (ret < 0)
		return ret;
	if (!ret)
		return -EINVAL;

	memset(&slave_config, 0, sizeof(slave_config));
	ret = snd_hwparams_to_dma_slave_config(substream, params, &slave_config);
	dev_info(dcpaud->dev, "snd_hwparams_to_dma_slave_config: %d\n", ret);
	if (ret < 0)
		return ret;

	slave_config.direction = DMA_MEM_TO_DEV;
	/*
	 * The data entry from the DMA controller to the DPA peripheral
	 * is 32-bit wide no matter the actual sample size.
	 */
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	ret = dmaengine_slave_config(chan, &slave_config);
	dev_info(dcpaud->dev, "dmaengine_slave_config: %d\n", ret);
	return ret;
}

static int dcp_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;

	if (!dcpaud_connection_up(dcpaud))
		return 0;

	return dcp_audiosrv_unprepare(dcpaud->dcp_dev);
}

static int dcp_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;

	/* alsa-lib prepares right after hw_params; see dcp_pcm_hw_unplugged */
	if (dcpaud->open_unplugged || !dcpaud_connection_up(dcpaud))
		return 0;

	return dcp_audiosrv_prepare(dcpaud->dcp_dev,
				    &dcpaud->selected_cookie);
}

static u64 dcpaud_void_period_ns(struct snd_pcm_runtime *runtime)
{
	return div_u64((u64)runtime->period_size * NSEC_PER_SEC, runtime->rate);
}

static void dcpaud_void_work(struct work_struct *work)
{
	struct dcp_audio *dcpaud = container_of(to_delayed_work(work),
						struct dcp_audio, void_work);
	struct snd_pcm_substream *substream = dcpaud->substream;
	ktime_t next;
	s64 delay;

	snd_pcm_period_elapsed(substream);

	snd_pcm_stream_lock_irq(substream);
	if (dcpaud->void_running) {
		/* the next period boundary, so that periods do not drift */
		next = ktime_add_ns(dcpaud->void_start,
				    ++dcpaud->void_periods *
				    dcpaud_void_period_ns(substream->runtime));
		delay = ktime_to_ns(ktime_sub(next, ktime_get()));
		schedule_delayed_work(&dcpaud->void_work,
				      delay > 0 ? nsecs_to_jiffies(delay) : 0);
	}
	snd_pcm_stream_unlock_irq(substream);
}

/*
 * Without a sink, DCP has no audio link to start. Failing START does not
 * stop userspace from trying: PipeWire retries it on every wakeup, and a
 * stream that never starts never makes it wait, so it retries at about
 * 130000 times a second for as long as something plays (a monitor
 * without audio, or none attached). Play into the void instead, as HDMI
 * PCMs elsewhere do: the stream runs on the clock, without DMA, and its
 * samples are dropped. A sink attached later gets the next stream opened.
 */
static int dcpaud_void_start(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	u64 period_ns = dcpaud_void_period_ns(substream->runtime);

	dev_dbg(dcpaud->dev, "no sink, playing into the void\n");
	dcpaud->void_running = true;
	dcpaud->void_start = ktime_get();
	dcpaud->void_periods = 0;
	schedule_delayed_work(&dcpaud->void_work, nsecs_to_jiffies(period_ns));

	return 0;
}

static snd_pcm_uframes_t dcp_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	u64 frames;

	if (!dcpaud->void_running)
		return snd_dmaengine_pcm_pointer(substream);

	frames = mul_u64_u32_div(ktime_to_ns(ktime_sub(ktime_get(),
						       dcpaud->void_start)),
				 runtime->rate, NSEC_PER_SEC);
	return do_div(frames, runtime->buffer_size);
}

static int dcp_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (dcpaud->open_unplugged || !dcpaud_connection_up(dcpaud))
			return dcpaud_void_start(substream);

		WARN_ON(pm_runtime_get_sync(dcpaud->dev) < 0);
		ret = dcp_audiosrv_startlink(dcpaud->dcp_dev,
					     &dcpaud->selected_cookie);
		if (ret < 0)
			return ret;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (dcpaud->void_running) {
			/* sync_stop waits for the work */
			dcpaud->void_running = false;
			return 0;
		}
		break;

	default:
		return -EINVAL;
	}

	ret = snd_dmaengine_pcm_trigger(substream, cmd);
	if (ret < 0)
		return ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = dcp_audiosrv_stoplink(dcpaud->dcp_dev);
		pm_runtime_mark_last_busy(dcpaud->dev);
		__pm_runtime_put_autosuspend(dcpaud->dev);
		if (ret < 0)
			return ret;
		break;
	}

	return 0;
}

/*
 * ALSA calls this once a stream has stopped, outside the stream lock, before
 * the next hw_params, prepare or hw_free and at close.
 */
static int dcp_pcm_sync_stop(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;

	cancel_delayed_work_sync(&dcpaud->void_work);

	return 0;
}

struct snd_pcm_ops dcp_playback_ops = {
	.open = dcp_pcm_open,
	.close = dcp_pcm_close,
	.hw_params = dcp_pcm_hw_params,
	.hw_free = dcp_pcm_hw_free,
	.prepare = dcp_pcm_prepare,
	.trigger = dcp_pcm_trigger,
	.sync_stop = dcp_pcm_sync_stop,
	.pointer = dcp_pcm_pointer,
};

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/*
 * DCP firmware 14.7, external role (the T6030 external processor, J514S
 * built-in HDMI): the link lifecycle of the native audio driver:
 *
 *   prepare:   the DMA channel configured, then PrepareLink, once per link;
 *   START:     the cyclic DMA issued, then StartLink;
 *   STOP:      the DMA terminated;
 *   sync_stop: the DMA drained, then StopLink and CompleteLink;
 *   hw_free:   CompleteLink of a link prepared but never started.
 *
 * Every link method carries the selected element's link data. A link of an
 * earlier connection went with its service: its state is dropped unsent.
 * The audio service itself is open only while the PCM is.
 */
static int dcp_pcm_native_open(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	bool live;
	int ret;

	scoped_guard(mutex, &dcpaud->data_lock)
		live = dcpaud->connected;
	if (live) {
		ret = dcp_audiosrv_open(dcpaud->dcp_dev);
		if (ret)
			return ret;
	}

	ret = dcp_pcm_open(substream);
	if (ret && live)
		dcp_audiosrv_close(dcpaud->dcp_dev);

	return ret;
}

static int dcp_pcm_native_close(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	int ret = dcp_pcm_close(substream);

	dcp_audiosrv_close(dcpaud->dcp_dev);

	return ret;
}

/*
 * StopLink and CompleteLink of the link held, on the open audio service
 * (@live), or dropped unsent with a service that went away.
 */
static void dcpaud_link_teardown(struct dcp_audio *dcpaud, bool live)
{
	int ret;

	lockdep_assert_held(&dcpaud->link_lock);
	if (dcpaud->link_started && live) {
		ret = dcp_audiosrv_link(dcpaud->dcp_dev, DCP_AUDIO_LINK_STOP,
					&dcpaud->selected_cookie);
		if (ret)
			dev_err(dcpaud->dev, "StopLink failed: %d\n", ret);
	}
	dcpaud->link_started = false;
	if (dcpaud->link_prepared && live) {
		ret = dcp_audiosrv_link(dcpaud->dcp_dev, DCP_AUDIO_LINK_COMPLETE,
					&dcpaud->selected_cookie);
		if (ret)
			dev_err(dcpaud->dev, "CompleteLink failed: %d\n", ret);
	}
	dcpaud->link_prepared = false;
}

/* New parameters select a new element: the link of the old one ends first. */
static int dcp_pcm_native_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;

	scoped_guard(mutex, &dcpaud->link_lock)
		dcpaud_link_teardown(dcpaud, dcpaud_connection_up(dcpaud));

	return dcp_pcm_hw_params(substream, params);
}

static int dcp_pcm_native_hw_free(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;

	guard(mutex)(&dcpaud->link_lock);
	dcpaud_link_teardown(dcpaud, dcpaud_connection_up(dcpaud));

	return 0;
}

static int dcp_pcm_native_prepare(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	struct dma_slave_config slave_config = {
		.direction = DMA_MEM_TO_DEV,
		/* One 32-bit DPA entry per sample word, as in hw_params. */
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};
	int ret;

	if (dcpaud->open_unplugged)
		return 0;

	if (!dcpaud_connection_up(dcpaud))
		return -ENXIO;

	guard(mutex)(&dcpaud->link_lock);
	if (dcpaud->link_prepared)
		return 0;

	/* The SIO channel is configured again before every PrepareLink. */
	ret = dmaengine_slave_config(snd_dmaengine_pcm_get_chan(substream),
				     &slave_config);
	if (ret)
		return ret;

	ret = dcp_audiosrv_link(dcpaud->dcp_dev, DCP_AUDIO_LINK_PREPARE,
				&dcpaud->selected_cookie);
	if (ret)
		return ret;

	dcpaud->link_prepared = true;
	return 0;
}

static int dcp_pcm_native_start(struct snd_pcm_substream *substream, int cmd)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	int ret;

	if (!dcpaud_connection_up(dcpaud))
		return -ENXIO;

	/* The first period descriptor is issued before StartLink. */
	ret = snd_dmaengine_pcm_trigger(substream, cmd);
	if (ret)
		return ret;

	guard(mutex)(&dcpaud->link_lock);
	if (!dcpaud->link_prepared)
		return -EBADFD;
	if (dcpaud->link_started)
		return 0;
	ret = dcp_audiosrv_link(dcpaud->dcp_dev, DCP_AUDIO_LINK_START,
				&dcpaud->selected_cookie);
	dcpaud->link_started = !ret;
	return ret;
}

static int dcp_pcm_native_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/*
		 * Balanced by STOP, which the PCM core also sends after a
		 * failed start; sync_stop then drains the DMA and completes
		 * the link.
		 */
		WARN_ON(pm_runtime_get_sync(dcpaud->dev) < 0);
		return dcp_pcm_native_start(substream, cmd);

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = snd_dmaengine_pcm_trigger(substream, cmd);
		pm_runtime_mark_last_busy(dcpaud->dev);
		__pm_runtime_put_autosuspend(dcpaud->dev);
		return ret;

	default:
		return -EINVAL;
	}
}

static int dcp_pcm_native_sync_stop(struct snd_pcm_substream *substream)
{
	struct dcp_audio *dcpaud = substream->pcm->private_data;
	int ret;

	/* SIO TERMINATE acknowledged and the issued periods reported back. */
	ret = snd_dmaengine_pcm_sync_stop(substream);

	guard(mutex)(&dcpaud->link_lock);
	dcpaud_link_teardown(dcpaud, dcpaud_connection_up(dcpaud));

	return ret;
}

/*
 * The card was disconnected (its stream stopped) while the audio service
 * stays: the display was powered off or the sink released, and the service
 * is about to be closed. Its link is completed first, the DMA drained
 * before: the firmware keeps a link prepared by a client that closes without
 * CompleteLink, and refuses every later PrepareLink of that service
 * (0xe00002ea), so every later open failed.
 */
void dcpaud_release_link(struct platform_device *pdev)
{
	struct dcp_audio *dcpaud = platform_get_drvdata(pdev);

	if (!dcpaud->native_link)
		return;
	guard(mutex)(&dcpaud->link_lock);
	if (!dcpaud->link_prepared && !dcpaud->link_started)
		return;
	if (dcpaud->chan)
		dmaengine_synchronize(dcpaud->chan);
	dcpaud_link_teardown(dcpaud, true);
}

static const struct snd_pcm_ops dcp_native_playback_ops = {
	.open = dcp_pcm_native_open,
	.close = dcp_pcm_native_close,
	.hw_params = dcp_pcm_native_hw_params,
	.hw_free = dcp_pcm_native_hw_free,
	.prepare = dcp_pcm_native_prepare,
	.trigger = dcp_pcm_native_trigger,
	.sync_stop = dcp_pcm_native_sync_stop,
	.pointer = snd_dmaengine_pcm_pointer,
};
#endif

// Transitional workaround: for the chmap control TLV, advertise options
// copied from hdmi-codec.c
#include "hdmi-codec-chmap.h"

static int dcpaud_chmap_ctl_get(struct snd_kcontrol *kcontrol,
			        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct dcp_audio *dcpaud = info->private_data;
	unsigned int i;

	for (i = 0; i < info->max_channels; i++)
		ucontrol->value.integer.value[i] = \
				(i < dcpaud->selected_chmap.channels) ?
				dcpaud->selected_chmap.map[i] : SNDRV_CHMAP_UNKNOWN;

	return 0;
}


static int dcpaud_create_chmap_ctl(struct dcp_audio *dcpaud)
{
	struct snd_pcm *pcm = dcpaud->substream->pcm;
	struct snd_pcm_chmap *chmap_info;
	int ret;

	ret = snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK, NULL,
				     dcp_pcm_hw.channels_max, 0, &chmap_info);
	if (ret < 0)
		return ret;

	chmap_info->kctl->get = dcpaud_chmap_ctl_get;
	chmap_info->chmap = hdmi_codec_8ch_chmaps;
	chmap_info->private_data = dcpaud;

	return 0;
}

static int dcpaud_create_pcm(struct dcp_audio *dcpaud)
{
	struct snd_card *card = dcpaud->card;
	struct snd_pcm *pcm;
	int ret;

#define NUM_PLAYBACK 1
#define NUM_CAPTURE 0

	ret = snd_pcm_new(card, card->shortname, 0, NUM_PLAYBACK, NUM_CAPTURE, &pcm);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	if (dcpaud_native_link(dcpaud))
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &dcp_native_playback_ops);
	else
#endif
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &dcp_playback_ops);
	dcpaud->substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	pcm->nonatomic = true;
	pcm->private_data = dcpaud;
	strscpy(pcm->name, card->shortname, sizeof(pcm->name));

	return 0;
}

/* expects to be called with data_lock locked and unlocks it */
static void dcpaud_report_hotplug(struct dcp_audio *dcpaud, bool connected)
{
	struct snd_pcm_substream *substream = dcpaud->substream;

	if (!dcpaud->card || dcpaud->connected == connected) {
		mutex_unlock(&dcpaud->data_lock);
		return;
	}

	dcpaud->connected = connected;
	if (connected)
		dcpaud->connection_cookie++;
	mutex_unlock(&dcpaud->data_lock);

	snd_jack_report(dcpaud->jack, connected ? SND_JACK_AVOUT : 0);

	if (!connected) {
		snd_pcm_stream_lock(substream);
		/* a stream without a sink plays on, see dcpaud_void_start() */
		if (substream->runtime && !dcpaud->void_running)
			snd_pcm_stop(substream, SNDRV_PCM_STATE_DISCONNECTED);
		snd_pcm_stream_unlock(substream);
	}
}

static int dcpaud_create_jack(struct dcp_audio *dcpaud)
{
	struct snd_card *card = dcpaud->card;

	return snd_jack_new(card, "HDMI/DP", SND_JACK_AVOUT,
			    &dcpaud->jack, true, false);
}

static void dcpaud_set_card_names(struct dcp_audio *dcpaud)
{
	struct snd_card *card = dcpaud->card;

	strscpy(card->driver, "apple_dcp", sizeof(card->driver));
	strscpy(card->longname, "Apple DisplayPort", sizeof(card->longname));
	strscpy(card->shortname, "Apple DisplayPort", sizeof(card->shortname));
}

#ifdef CONFIG_SND_DEBUG
static void dcpaud_expose_debugfs_blob(struct dcp_audio *dcpaud, const char *name, void *base, size_t size)
{
	struct debugfs_blob_wrapper *wrapper;
	wrapper = devm_kzalloc(dcpaud->dev, sizeof(*wrapper), GFP_KERNEL);
	if (!wrapper)
		return;
	wrapper->data = base;
	wrapper->size = size;
	debugfs_create_blob(name, 0600, dcpaud->card->debugfs_root, wrapper);
}
#else
static void dcpaud_expose_debugfs_blob(struct dcp_audio *dcpaud, const char *name, void *base, size_t size) {}
#endif

extern bool hdmi_audio;

static int dcpaud_init_snd_card(struct dcp_audio *dcpaud)
{
	int ret;
	if (!hdmi_audio)
		return -ENODEV;


	ret = snd_card_new(dcpaud->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &dcpaud->card);
	if (ret)
		return ret;

	dcpaud_set_card_names(dcpaud);

	ret = dcpaud_create_pcm(dcpaud);
	if (ret)
		goto err_free_card;

	ret = dcpaud_create_chmap_ctl(dcpaud);
	if (ret)
		goto err_free_card;

	ret = dcpaud_create_jack(dcpaud);
	if (ret)
		goto err_free_card;

	ret = snd_card_register(dcpaud->card);
	if (ret)
		goto err_free_card;

	return 0;
err_free_card:
	dev_warn(dcpaud->dev, "Failed to initialize sound card: %d\n", ret);
	snd_card_free(dcpaud->card);
	dcpaud->card = NULL;
	return ret;
}

void dcpaud_connect(struct platform_device *pdev, bool connected)
{
	struct dcp_audio *dcpaud = platform_get_drvdata(pdev);

	mutex_lock(&dcpaud->data_lock);

	dcpaud_report_hotplug(dcpaud, connected);
}

void dcpaud_disconnect(struct platform_device *pdev)
{
	struct dcp_audio *dcpaud = platform_get_drvdata(pdev);

	mutex_lock(&dcpaud->data_lock);

	dcpaud_report_hotplug(dcpaud, false);
}

static int dcpaud_comp_bind(struct device *dev, struct device *main, void *data)
{
	struct dcp_audio *dcpaud = dev_get_drvdata(dev);
	struct device_node *endpoint, *dcp_node = NULL;
	struct platform_device *dcp_pdev, *dma_pdev;
	struct of_phandle_args dma_spec;
	int index;
	int ret;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable runtime PM: %d\n", ret);

	/* find linked DCP instance */
	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (endpoint) {
		dcp_node = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
	}
	if (!dcp_node || !of_device_is_available(dcp_node)) {
		of_node_put(dcp_node);
		dev_info(dev, "No audio support\n");
		goto rpm_put;
	}

	index = of_property_match_string(dev->of_node, "dma-names", "tx");
	if (index < 0) {
		dev_err(dev, "No dma-names property\n");
		goto rpm_put;
	}

	if (of_parse_phandle_with_args(dev->of_node, "dmas", "#dma-cells", index,
				       &dma_spec) || !dma_spec.np) {
		dev_err(dev, "Failed to parse dmas property\n");
		goto rpm_put;
	}

	dcp_pdev = of_find_device_by_node(dcp_node);
	of_node_put(dcp_node);
	if (!dcp_pdev) {
		dev_info(dev, "No DP/HDMI audio device, dcp not ready\n");
		goto rpm_put;
	}
	dcpaud->dcp_dev = &dcp_pdev->dev;

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/* The 14.7 external role: its native links and element catalog. */
	if (dcp_audiosrv_native_link(dcpaud->dcp_dev)) {
		void *elements = devm_kzalloc(dev, DCPAUD_ELEMENTS_V14_7, GFP_KERNEL);

		if (!elements)
			goto rpm_put;
		devm_kfree(dev, dcpaud->elements);
		dcpaud->elements = elements;
		dcpaud->native_link = true;
	}
#endif

	dma_pdev = of_find_device_by_node(dma_spec.np);
	of_node_put(dma_spec.np);
	if (!dma_pdev) {
		dev_info(dev, "No DMA device\n");
		goto rpm_put;
	}
	dcpaud->dma_dev = &dma_pdev->dev;

	dcpaud->dma_link = device_link_add(dev, dcpaud->dma_dev,
					   DL_FLAG_PM_RUNTIME |
					   DL_FLAG_RPM_ACTIVE |
					   DL_FLAG_STATELESS);

	/* ignore errors to prevent audio issues affecting the display side */
	ret = dcpaud_init_snd_card(dcpaud);

	if (!ret) {
		dcpaud_expose_debugfs_blob(dcpaud, "selected_cookie", &dcpaud->selected_cookie,
					sizeof(dcpaud->selected_cookie));
		dcpaud_expose_debugfs_blob(dcpaud, "elements", dcpaud->elements,
					dcpaud_elements_size(dcpaud));
		dcpaud_expose_debugfs_blob(dcpaud, "product_attrs", dcpaud->productattrs,
					DCPAUD_PRODUCTATTRS_MAXSIZE);
	}

rpm_put:
	pm_runtime_put(dev);

	return 0;
}

static void dcpaud_comp_unbind(struct device *dev, struct device *main,
			       void *data)
{
	struct dcp_audio *dcpaud = dev_get_drvdata(dev);

	/* snd_card_free_when_closed() checks for NULL */
	snd_card_free_when_closed(dcpaud->card);

	if (dcpaud->dma_link)
		device_link_del(dcpaud->dma_link);
}

static const struct component_ops dcpaud_comp_ops = {
	.bind	= dcpaud_comp_bind,
	.unbind	= dcpaud_comp_unbind,
};

static int dcpaud_probe(struct platform_device *pdev)
{
	struct dcp_audio *dcpaud;

	dcpaud = devm_kzalloc(&pdev->dev, sizeof(*dcpaud), GFP_KERNEL);
	if (!dcpaud)
		return -ENOMEM;

	dcpaud->elements = devm_kzalloc(&pdev->dev, DCPAUD_ELEMENTS_MAXSIZE,
					GFP_KERNEL);
	if (!dcpaud->elements)
		return -ENOMEM;

	dcpaud->productattrs = devm_kzalloc(&pdev->dev, DCPAUD_PRODUCTATTRS_MAXSIZE,
					    GFP_KERNEL);
	if (!dcpaud->productattrs)
		return -ENOMEM;

	dcpaud->dev = &pdev->dev;
	mutex_init(&dcpaud->data_lock);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	mutex_init(&dcpaud->link_lock);
#endif
	INIT_DELAYED_WORK(&dcpaud->void_work, dcpaud_void_work);
	platform_set_drvdata(pdev, dcpaud);

	return component_add(&pdev->dev, &dcpaud_comp_ops);
}

static void dcpaud_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcpaud_comp_ops);
}

static void dcpaud_shutdown(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcpaud_comp_ops);
}

static __maybe_unused int dcpaud_suspend(struct device *dev)
{
	/*
	 * Using snd_power_change_state() does not work since the sound card
	 * is what resumes runtime PM.
	 */

	return 0;
}

static __maybe_unused int dcpaud_resume(struct device *dev)
{
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(dcpaud_pm_ops, dcpaud_suspend, dcpaud_resume, NULL);

static const struct of_device_id dcpaud_of_match[] = {
	{ .compatible = "apple,dpaudio" },
	{}
};

static struct platform_driver dcpaud_driver = {
	.driver = {
		.name = "dcp-dp-audio",
		.of_match_table	= dcpaud_of_match,
		.pm		= pm_ptr(&dcpaud_pm_ops),
	},
	.probe		= dcpaud_probe,
	.remove		= dcpaud_remove,
	.shutdown	= dcpaud_shutdown,
};

int __init dcp_audio_register(void)
{
	return platform_driver_register(&dcpaud_driver);
}

void __exit dcp_audio_unregister(void)
{
        platform_driver_unregister(&dcpaud_driver);
}

