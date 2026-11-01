// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Included by audio.c: a Terminate's link release ends only the link of the
 * instance it is for, and a stopped stereo link is held between streams.
 * The production link lifecycle runs; the audio service's answer to which
 * instance the link is on, its link calls and the DMA channel are stubbed.
 */
#include <kunit/platform_device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>

struct dcpaud_link_test {
	bool link_on;
	bool link_live;
	/* The error removing the idle transfer returns */
	int idle_remove_err;
	u64 asked;
	unsigned int completes;
	unsigned int stops;
	struct dma_device dma;
	struct dma_chan channel;
	struct dma_async_tx_descriptor desc;
	char sequence[16];
	unsigned int nsequence;
};

static struct dcpaud_link_test *dcpaud_link_current;

static void dcpaud_link_record(char step)
{
	struct dcpaud_link_test *t = dcpaud_link_current;

	if (t->nsequence < sizeof(t->sequence) - 1)
		t->sequence[t->nsequence++] = step;
}

static bool dcpaud_link_test_live(struct device *dev)
{
	return dcpaud_link_current->link_live;
}

static int dcpaud_link_test_terminate(struct dma_chan *chan)
{
	dcpaud_link_record('X');
	return 0;
}

/* G the channel configured; the silence as its idle transfer: I set, i removed */
static int dcpaud_link_test_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	const struct apple_sio_idle *idle = config->peripheral_config;

	if (!idle) {
		dcpaud_link_record('G');
		return 0;
	}
	if (config->peripheral_size != sizeof(*idle))
		return -EINVAL;
	dcpaud_link_record(idle->len ? 'I' : 'i');
	return idle->len ? 0 : dcpaud_link_current->idle_remove_err;
}

static dma_cookie_t dcpaud_link_test_submit(struct dma_async_tx_descriptor *tx)
{
	return 1;
}

static struct dma_async_tx_descriptor *
dcpaud_link_test_prep(struct dma_chan *chan, dma_addr_t buf, size_t len, size_t period,
		      enum dma_transfer_direction dir, unsigned long flags)
{
	struct dcpaud_link_test *t = dcpaud_link_current;

	dcpaud_link_record('Q');
	t->desc.chan = chan;
	t->desc.tx_submit = dcpaud_link_test_submit;
	return &t->desc;
}

static void dcpaud_link_test_issue(struct dma_chan *chan)
{
}

static void dcpaud_link_test_drain(struct dma_chan *chan)
{
	dcpaud_link_record('D');
}

static enum dma_status dcpaud_link_test_status(struct dma_chan *chan,
					      dma_cookie_t cookie, struct dma_tx_state *state)
{
	return DMA_COMPLETE;
}

static bool dcpaud_link_test_on(struct device *dev, u64 gen)
{
	dcpaud_link_current->asked = gen;
	return dcpaud_link_current->link_on;
}

static int dcpaud_link_test_call(struct device *dev, enum dcp_audio_link op,
				 const struct dcp_sound_cookie *cookie)
{
	if (op == DCP_AUDIO_LINK_COMPLETE) {
		dcpaud_link_current->completes++;
		dcpaud_link_record('C');
	} else if (op == DCP_AUDIO_LINK_STOP) {
		dcpaud_link_current->stops++;
		dcpaud_link_record('S');
	} else if (op == DCP_AUDIO_LINK_PREPARE) {
		dcpaud_link_record('P');
	} else if (op == DCP_AUDIO_LINK_START) {
		dcpaud_link_record('T');
	}
	return 0;
}

static struct dcp_audio *dcpaud_link_setup(struct kunit *test, struct platform_device **pdev)
{
	struct dcpaud_link_test *t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	struct dcp_audio *dcpaud = kunit_kzalloc(test, sizeof(*dcpaud), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, t);
	KUNIT_ASSERT_NOT_NULL(test, dcpaud);
	*pdev = kunit_platform_device_alloc(test, "dcpaud-link", PLATFORM_DEVID_AUTO);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, *pdev);
	platform_set_drvdata(*pdev, dcpaud);
	dcpaud->dev = &(*pdev)->dev;
	dcpaud->dcp_dev = &(*pdev)->dev;
	dcpaud->native_link = true;
	mutex_init(&dcpaud->link_lock);
	dcpaud_link_current = t;
	t->link_live = true;
	t->channel.device = &t->dma;
	t->dma.device_terminate_all = dcpaud_link_test_terminate;
	t->dma.device_synchronize = dcpaud_link_test_drain;
	t->dma.device_tx_status = dcpaud_link_test_status;
	t->dma.device_prep_dma_cyclic = dcpaud_link_test_prep;
	t->dma.device_issue_pending = dcpaud_link_test_issue;
	t->dma.device_config = dcpaud_link_test_config;
	kunit_activate_static_stub(test, dcp_audiosrv_link_on, dcpaud_link_test_on);
	kunit_activate_static_stub(test, dcp_audiosrv_link_live, dcpaud_link_test_live);
	kunit_activate_static_stub(test, dcp_audiosrv_link, dcpaud_link_test_call);
	return dcpaud;
}

static void dcpaud_link_release_instance(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *dcpaud = dcpaud_link_setup(test, &pdev);
	u64 gen = 7;

	/* The link is on another instance: left as it is */
	dcpaud->link_prepared = true;
	dcpaud->link_started = true;
	dcpaud_release_link(pdev, &gen);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->asked, 7ULL);
	KUNIT_EXPECT_TRUE(test, dcpaud->link_prepared);
	KUNIT_EXPECT_TRUE(test, dcpaud->link_started);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->stops + dcpaud_link_current->completes, 0U);

	/* On the instance asked for: stopped and completed */
	dcpaud_link_current->link_on = true;
	dcpaud_release_link(pdev, &gen);
	KUNIT_EXPECT_FALSE(test, dcpaud->link_prepared);
	KUNIT_EXPECT_FALSE(test, dcpaud->link_started);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->stops, 1U);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->completes, 1U);

	/* Without an instance (the display's own power-off): whichever link */
	dcpaud_link_current->link_on = false;
	dcpaud->link_prepared = true;
	dcpaud_release_link(pdev, NULL);
	KUNIT_EXPECT_FALSE(test, dcpaud->link_prepared);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->completes, 2U);
}

static void dcpaud_link_test_close(void *data)
{
	snd_dmaengine_pcm_close(data);
}

static struct snd_pcm_substream *dcpaud_link_test_open(struct kunit *test,
						     struct dcp_audio *dcpaud)
{
	struct snd_pcm_substream *sub = kunit_kzalloc(test, sizeof(*sub), GFP_KERNEL);
	struct snd_pcm_runtime *runtime = kunit_kzalloc(test, sizeof(*runtime), GFP_KERNEL);
	struct snd_pcm *pcm = kunit_kzalloc(test, sizeof(*pcm), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, sub);
	KUNIT_ASSERT_NOT_NULL(test, runtime);
	KUNIT_ASSERT_NOT_NULL(test, pcm);
	pcm->private_data = dcpaud;
	pcm->nonatomic = true;
	sub->pcm = pcm;
	sub->runtime = runtime;
	KUNIT_ASSERT_EQ(test, snd_dmaengine_pcm_open(sub, &dcpaud_link_current->channel), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, dcpaud_link_test_close, sub), 0);
	return sub;
}

static void dcpaud_disconnect_sync_completes_live_owner(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	/* Disconnect cleared connected; the deferred owner release has not run. */
	audio->connected = false;
	audio->link_prepared = true;
	audio->link_started = true;
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_sync_stop(sub), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "DSC");
	KUNIT_EXPECT_FALSE(test, audio->link_prepared);
	KUNIT_EXPECT_FALSE(test, audio->link_started);
	dcpaud_release_link(pdev, NULL);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->stops, 1U);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->completes, 1U);
}

static void dcpaud_gone_service_drops_link(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	audio->link_prepared = true;
	audio->link_started = true;
	dcpaud_link_current->link_live = false;
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_sync_stop(sub), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "D");
	KUNIT_EXPECT_FALSE(test, audio->link_prepared);
	KUNIT_EXPECT_FALSE(test, audio->link_started);
}

/* A stream stopped by STOP with its DMA playing on, on the PCM's channel */
static void dcpaud_link_test_held_stop(struct kunit *test, struct dcp_audio *audio)
{
	audio->chan = &dcpaud_link_current->channel;
	audio->silence = kunit_kzalloc(test, DCPAUD_SILENCE_BYTES, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, audio->silence);
	audio->link_prepared = true;
	audio->link_started = true;
	audio->dma_cleared = true;
	/* The stream's PM reference, carried by the cleared DMA */
	pm_runtime_get_noresume(audio->dev);
}

static void dcpaud_stop_holds_link(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	dcpaud_link_test_held_stop(test, audio);
	/* The silence set first, the stream's DMA ends into it; no link call */
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_sync_stop(sub), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "IXD");
	KUNIT_EXPECT_TRUE(test, audio->silence_running);
	KUNIT_EXPECT_TRUE(test, audio->idle_set);
	KUNIT_EXPECT_TRUE(test, audio->link_started);

	KUNIT_ASSERT_EQ(test, dcp_pcm_native_hw_free(sub), 0);
	KUNIT_EXPECT_TRUE(test, audio->link_prepared);
	KUNIT_EXPECT_EQ(test, dcpaud_link_current->stops + dcpaud_link_current->completes, 0U);

	/* The display goes off: the silence ends, then the link */
	dcpaud_release_link(pdev, NULL);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "IXDiDSC");
	KUNIT_EXPECT_FALSE(test, audio->silence_running);
	KUNIT_EXPECT_FALSE(test, audio->idle_set);
	KUNIT_EXPECT_FALSE(test, audio->link_prepared);
	/* The stream's reference went with the silence */
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 0);
}

static void dcpaud_stuck_silence_keeps_pm(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	dcpaud_link_test_held_stop(test, audio);
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_sync_stop(sub), 0);
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_hw_free(sub), 0);

	/* The channel cannot be confirmed stopped: it may still read the silence */
	dcpaud_link_current->idle_remove_err = -EIO;
	dcpaud_release_link(pdev, NULL);
	KUNIT_EXPECT_FALSE(test, audio->silence_running);
	KUNIT_EXPECT_FALSE(test, audio->link_prepared);
	KUNIT_EXPECT_TRUE(test, audio->idle_set);
	KUNIT_EXPECT_TRUE(test, audio->idle_pm);
	/* The stream's reference stays with the silence */
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 1);

	/* Removed later, it gives the reference back */
	dcpaud_link_current->idle_remove_err = 0;
	scoped_guard(mutex, &audio->link_lock)
		KUNIT_EXPECT_EQ(test, dcpaud_idle(audio, false), 0);
	KUNIT_EXPECT_FALSE(test, audio->idle_set);
	KUNIT_EXPECT_FALSE(test, audio->idle_pm);
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 0);
}

static void dcpaud_held_stop_without_service(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	dcpaud_link_test_held_stop(test, audio);
	dcpaud_link_current->link_live = false;
	/* No service to hold it for: the DMA ends and the link is dropped */
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_sync_stop(sub), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "XD");
	KUNIT_EXPECT_FALSE(test, audio->silence_running);
	KUNIT_EXPECT_FALSE(test, audio->link_prepared);
	KUNIT_EXPECT_FALSE(test, audio->link_started);
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 0);
}

/* A held link and its silence, and a prepared stream to start on it */
static struct snd_pcm_substream *dcpaud_link_test_held(struct kunit *test, struct dcp_audio *audio)
{
	struct snd_pcm_substream *sub = dcpaud_link_test_open(test, audio);

	mutex_init(&audio->data_lock);
	audio->connected = true;
	audio->chan = &dcpaud_link_current->channel;
	audio->link_prepared = true;
	audio->link_started = true;
	audio->silence_running = true;
	audio->idle_set = true;
	/* The held link's PM reference, then the stream's from its START */
	pm_runtime_get_noresume(audio->dev);
	pm_runtime_get_noresume(audio->dev);
	sub->runtime->frame_bits = 64;
	sub->runtime->buffer_size = 8192;
	sub->runtime->period_size = 512;
	return sub;
}

static void dcpaud_start_on_held_link(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_held(test, audio);

	/* Its element: the stream's DMA follows the silence, no link call */
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_start(sub, SNDRV_PCM_TRIGGER_START), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "Q");
	KUNIT_EXPECT_FALSE(test, audio->silence_running);
	KUNIT_EXPECT_TRUE(test, audio->idle_set);
	KUNIT_EXPECT_TRUE(test, audio->link_started);
	pm_runtime_put_noidle(audio->dev);
}

static void dcpaud_start_replaces_held_link(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_held(test, audio);

	/* Another element: the held link ends, the stream's is configured, prepared and started */
	memset(&audio->link_cookie, 1, sizeof(audio->link_cookie));
	memset(&audio->selected_cookie, 2, sizeof(audio->selected_cookie));
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_start(sub, SNDRV_PCM_TRIGGER_START), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "iSCGPQT");
	KUNIT_EXPECT_FALSE(test, audio->idle_set);
	KUNIT_EXPECT_EQ(test, memcmp(&audio->link_cookie, &audio->selected_cookie,
				     sizeof(audio->link_cookie)), 0);
	KUNIT_EXPECT_TRUE(test, audio->link_started);
	pm_runtime_put_noidle(audio->dev);
}

static void dcpaud_start_after_held_link_went(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_held(test, audio);

	/* The held link's service went (unplug): the stream gets a link of its own */
	dcpaud_link_current->link_live = false;
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_start(sub, SNDRV_PCM_TRIGGER_START), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "iGPQT");
	KUNIT_EXPECT_FALSE(test, audio->silence_running);
	KUNIT_EXPECT_TRUE(test, audio->link_started);
	pm_runtime_put_noidle(audio->dev);
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 0);
}

static void dcpaud_failed_start_keeps_held_link(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = dcpaud_link_test_held(test, audio);

	/* Unplugged before the swap: START fails, its STOP leaves the silence */
	audio->connected = false;
	sub->runtime->channels = 2;
	sub->runtime->dma_area = kunit_kzalloc(test, 64, GFP_KERNEL);
	sub->runtime->dma_bytes = 64;
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_start(sub, SNDRV_PCM_TRIGGER_START), -ENXIO);
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_trigger(sub, SNDRV_PCM_TRIGGER_STOP), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "");
	KUNIT_EXPECT_FALSE(test, audio->dma_cleared);
	KUNIT_EXPECT_TRUE(test, audio->silence_running);
	/* Only the held silence's reference is left */
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 1);
	pm_runtime_put_noidle(audio->dev);
}

static void dcpaud_close_keeps_held_silence(struct kunit *test)
{
	struct platform_device *pdev;
	struct dcp_audio *audio = dcpaud_link_setup(test, &pdev);
	struct snd_pcm_substream *sub = kunit_kzalloc(test, sizeof(*sub), GFP_KERNEL);
	struct snd_pcm_runtime *runtime = kunit_kzalloc(test, sizeof(*runtime), GFP_KERNEL);
	struct snd_pcm *pcm = kunit_kzalloc(test, sizeof(*pcm), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, sub);
	KUNIT_ASSERT_NOT_NULL(test, runtime);
	KUNIT_ASSERT_NOT_NULL(test, pcm);
	pcm->private_data = audio;
	pcm->nonatomic = true;
	sub->pcm = pcm;
	sub->runtime = runtime;
	KUNIT_ASSERT_EQ(test, snd_dmaengine_pcm_open(sub, &dcpaud_link_current->channel), 0);
	audio->chan = &dcpaud_link_current->channel;
	audio->link_prepared = true;
	audio->link_started = true;
	audio->silence_running = true;
	audio->idle_set = true;
	pm_runtime_get_noresume(audio->dev);

	/* The PCM's terminate leaves the idle silence: nothing queued, no link call */
	KUNIT_ASSERT_EQ(test, dcp_pcm_native_close(sub), 0);
	KUNIT_EXPECT_STREQ(test, dcpaud_link_current->sequence, "XD");
	KUNIT_EXPECT_TRUE(test, audio->silence_running);
	KUNIT_EXPECT_TRUE(test, audio->idle_set);
	KUNIT_EXPECT_TRUE(test, audio->link_started);
	KUNIT_EXPECT_EQ(test, atomic_read(&audio->dev->power.usage_count), 1);
	pm_runtime_put_noidle(audio->dev);
}

static struct kunit_case dcpaud_link_cases[] = {
	KUNIT_CASE(dcpaud_link_release_instance),
	KUNIT_CASE(dcpaud_disconnect_sync_completes_live_owner),
	KUNIT_CASE(dcpaud_gone_service_drops_link),
	KUNIT_CASE(dcpaud_stop_holds_link),
	KUNIT_CASE(dcpaud_stuck_silence_keeps_pm),
	KUNIT_CASE(dcpaud_held_stop_without_service),
	KUNIT_CASE(dcpaud_start_on_held_link),
	KUNIT_CASE(dcpaud_start_replaces_held_link),
	KUNIT_CASE(dcpaud_start_after_held_link_went),
	KUNIT_CASE(dcpaud_failed_start_keeps_held_link),
	KUNIT_CASE(dcpaud_close_keeps_held_silence),
	{}
};

static struct kunit_suite dcpaud_link_suite = {
	.name = "apple-dcpaud-release-link",
	.test_cases = dcpaud_link_cases,
};

kunit_test_suite(dcpaud_link_suite);
