// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC machine driver for Apple Silicon Macs
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on sound/soc/qcom/{sc7180.c|common.c}
 * Copyright (c) 2018, Linaro Limited.
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 *
 * The platform driver has independent frontend and backend DAIs with the
 * option of routing backends to any of the frontends. The platform
 * driver configures the routing based on DPCM couplings in ASoC runtime
 * structures, which in turn are determined from DAPM paths by ASoC. But the
 * platform driver doesn't supply relevant DAPM paths and leaves that up for
 * the machine driver to fill in. The filled-in virtual topology can be
 * anything as long as any backend isn't connected to more than one frontend
 * at any given time. (The limitation is due to the unsupported case of
 * reparenting of live BEs.)
 */

/* #define DEBUG */

#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/capability.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/simple_card_utils.h>
#include <sound/soc.h>
#include <sound/soc-jack.h>
#include <uapi/linux/input-event-codes.h>
#include <uapi/linux/sched/types.h>

#define DRIVER_NAME "snd-soc-macaudio"

/*
 * CPU side is bit and frame clock provider
 * I2S has both clocks inverted
 */
#define MACAUDIO_DAI_FMT	(SND_SOC_DAIFMT_I2S | \
				 SND_SOC_DAIFMT_CBC_CFC | \
				 SND_SOC_DAIFMT_GATED | \
				 SND_SOC_DAIFMT_IB_IF)
#define MACAUDIO_JACK_MASK	(SND_JACK_HEADSET | SND_JACK_HEADPHONE)
#define MACAUDIO_SLOTWIDTH	32
/*
 * Maximum BCLK frequency
 *
 * Codec maximums:
 *  CS42L42  26.0 MHz
 *  TAS2770  27.1 MHz
 *  TAS2764  24.576 MHz
 */
#define MACAUDIO_MAX_BCLK_FREQ	24576000

#define SPEAKER_MAGIC_VALUE (s32)0xdec1be15
/* Primary, Secondary and Speaker Sense */
#define MACAUDIO_NUM_FE_LINKS	3
/* The "Speaker Volume Unlock" lease, in milliseconds */
#define SPEAKER_LOCK_TIMEOUT 250
/*
 * While "Speaker Protection" is the interface, this long without an owner of
 * it (the engine died, or never came) puts "Speaker Volume Unlock" back in
 * charge, at its ceiling.
 */
#define SPEAKER_LEASE_UNOWNED_MS	30000
/* The speaker protection lease an owner may ask for, in milliseconds */
#define SPEAKER_LEASE_MIN_MS	10
#define SPEAKER_LEASE_MAX_MS	100
/*
 * The withdrawal's workers run SCHED_FIFO above the audio server's data
 * threads (PipeWire's default is 88) and the protection supervisor: a
 * spinning real-time thread on one CPU cannot hold them off, as they wake
 * on another.
 */
#define SPEAKER_WORKER_PRIO	90
/*
 * A withdrawal while the owner's lease still runs (release, owner gone, a
 * new owner) ramps the amps down before the stream stops, so that the stop
 * is not a click: at most this long, never past the lease's end, one step
 * at least this far from the last.
 */
#define SPEAKER_RAMP_MAX_US	10000
#define SPEAKER_RAMP_STEP_US	500
#define MACAUDIO_MAX_SPEAKER_ID_GPIOS 8

enum macaudio_amp_type {
	AMP_NONE,
	AMP_TAS5770,
	AMP_SN012776,
	AMP_SSM3515,
	/*
	 * A fixed-gain amplifier with no controls of its own; the volume is
	 * a software gain the CPU DAI applies before the samples reach it.
	 */
	AMP_MAX98360A,
};

enum macaudio_spkr_config {
	SPKR_NONE,	/* No speakers */
	SPKR_1W,	/* 1 woofer / ch */
	SPKR_2W,	/* 2 woofers / ch */
	SPKR_1W1T,	/* 1 woofer + 1 tweeter / ch */
	SPKR_2W1T,	/* 2 woofers + 1 tweeter / ch */
};

struct macaudio_platform_cfg {
	bool enable_speakers;
	enum macaudio_amp_type amp;
	enum macaudio_spkr_config speakers;
	bool stereo;
	/* The amps' highest gain ("Amp Gain Volume"); not for the SN012776 */
	int amp_gain;
	int safe_vol;
	/*
	 * The speaker codec's output widget, "OUT" (the TAS and SSM
	 * amplifiers) unless set.
	 */
	const char *spk_out_widget;
	/*
	 * Software-gain amplifier: the CPU DAI whose front-end converts the
	 * speaker samples through the volume control at copy time.
	 */
	const char *sw_gain_dai;
	/*
	 * A CPU component other than the MCA: its front-end DAI names in
	 * place of "mca-pcm-N", the DAI format its serial links run (the
	 * default MACAUDIO_DAI_FMT when 0) and the primary and secondary
	 * front-ends' bclk ratios when the link's frame length is not ours
	 * to choose.
	 */
	const char *fe_dai_names[MACAUDIO_NUM_FE_LINKS];
	unsigned int dai_fmt;
	unsigned int primary_bclk_ratio;
	unsigned int secondary_bclk_ratio;
	/*
	 * The front-ends' CPU component is their PCM provider as well: no
	 * platform lookup by OF node, which takes every component on that
	 * node (the J616s AOP speaker and jack components share one, each
	 * for its own card).
	 */
	bool fe_cpu_is_platform;
	/*
	 * Front-ends (bits by link index) whose CPU DAI only completes the
	 * fixed DAPM routes and can never carry a stream on this machine.
	 * Their links are not bound, so the card has no PCM device for them
	 * that userspace could probe and fail to open. The CPU component
	 * keeps the other front-ends on their link numbers.
	 */
	unsigned int route_only_fes;
	/*
	 * The speaker front-end's channel count where the CPU component's
	 * stream has more slots than amps (0: one channel per amp). The amps
	 * keep the first slots in codec order; the others carry silence.
	 */
	unsigned int speaker_fe_channels;
	/*
	 * Restore the amp's DT playback corner on grant, where protection
	 * supplies the crossover ahead of the low-frequency thermal pilot.
	 * The 800 Hz policy floor would remove that pilot from the tweeters.
	 * Only an owner that declares the crossover gets it (see
	 * "Speaker Protection Crossover"). Every withdrawal restores the floor
	 * along with the safe level.
	 */
	bool tweeter_hpf_granted;
	/* The speaker links' DAI format, if not dai_fmt */
	unsigned int spk_dai_fmt;
	/*
	 * The speakers can only be fed from the secondary front-end: each
	 * front-end owns the DMA channel of one serializer (AOP-fed models).
	 */
	bool spk_secondary_only;
	/*
	 * No voltage or current sense reaches the AP, so no protection daemon
	 * can govern the speakers: their SN012776 volumes stay under safe_vol,
	 * locked, unlocked or granted alike, and there is no sense PCM. A
	 * granted lease runs them at safe_vol, its operating gain.
	 */
	bool spk_fixed_ceiling;
};

static const char *volume_control_names[] = {
	[AMP_TAS5770] = "* Speaker Playback Volume",
	[AMP_SN012776] = "* Speaker Volume",
	[AMP_SSM3515] = "* DAC Playback Volume",
	[AMP_MAX98360A] = "* Speaker Playback Volume",
};

#define SN012776_0DB 201
#define SN012776_DB(x) (SN012776_0DB + 2 * (x))
/* Same as SN012776 */
#define TAS5770_0DB SN012776_0DB
#define TAS5770_DB(x) SN012776_DB(x)

#define SSM3515_0DB (255 - 64) /* +24dB max, steps of 3/8 dB */
#define SSM3515_DB(x) (SSM3515_0DB + (8 * (x) / 3))

/* Software gain of the CPU DAI: 0.5 dB steps, 127 = the amplifier's full scale */
#define MAX98360A_0DB 127
#define MAX98360A_DB(x) (MAX98360A_0DB + 2 * (x))

/* CS42L84 DAC: 0.5 dB steps from -128 dB, 256 = 0 dB, up to +12 dB */
#define CS42L84_DAC_0DB 256

struct ma_codec_idle {
	int idle_mode;
	u32 idle_mask;
};

/* A speaker volume control and its value, as the system-sleep lock found it */
struct macaudio_pm_volume {
	struct snd_kcontrol *kctl;
	long value[2];
};

/*
 * The interface that owns the speakers (see "Speaker Protection Interface"):
 * the volume lock of speakersafetyd and speakerguardd 0.1, or the lease of a
 * protection engine in the signal path.
 */
enum macaudio_spk_iface {
	MACAUDIO_SPK_IFACE_UNLOCK,
	MACAUDIO_SPK_IFACE_LEASE,
};

/*
 * The software-gain CPU component's register that selects the volume
 * table's top: 0 the legacy ceiling (half the amplifier's full scale), 1 the
 * full scale (sound/soc/apple/aop_t8140.rs, REG_SPKR_FULL_SCALE).
 */
#define MACAUDIO_SW_GAIN_REG_FULL_SCALE	1

struct macaudio_snd_data {
	struct snd_soc_card card;
	struct snd_soc_jack jack;
	/* The jack links these into its pin list, so each card needs its own */
	struct snd_soc_jack_pin jack_pins[2];
	int jack_plugin_state;

	const struct macaudio_platform_cfg *cfg;
	bool has_speakers;
	bool has_sense;
	bool has_safety;
	unsigned int max_channels;
	struct gpio_descs *speaker_id_gpios;

	struct macaudio_link_props {
		/* frontend props */
		unsigned int bclk_ratio;
		bool is_sense;
		/* the front-end that applies the software speaker gain */
		bool sw_gain;

		/* backend props */
		bool is_speakers;
		bool is_headphones;
		unsigned int tdm_mask;
		struct ma_codec_idle *codecs;
	} *link_props;

	/* Speaker protection ownership: see macaudio_spk_put() */
	struct mutex spk_mutex;
	struct snd_kcontrol *spk_kctl;
	struct snd_ctl_file *spk_owner;
	/*
	 * Speaker playback may start (macaudio_spk_admitted()). Set under
	 * spk_mutex; cleared under it or by the lease timer, in hard IRQ
	 * context, at the lease's end. Read without spk_mutex.
	 */
	bool spk_granted;
	/* The amps are at the operating gain (spk_mutex) */
	bool spk_active;
	/* A safe level that did not take: speaker playback refused until reload */
	bool spk_floor_failed;
	/* The lease ran out: the owner's next write is refused with -ETIMEDOUT */
	bool spk_lapsed;
	/*
	 * Between PM_SUSPEND_PREPARE and PM_POST_SUSPEND: no grant. Set
	 * without spk_mutex (macaudio_pm_notify()), cleared under it.
	 */
	bool pm_quiesced;
	/* The card is being removed: no grant, no amp access */
	bool spk_dead;
	/*
	 * Speaker starts refused because protection was not granted: every
	 * one (debugfs speaker_starts_refused), and since the last grant.
	 */
	atomic_t spk_refused;
	atomic_t spk_refused_since_grant;
	/* Written under spk_mutex, read without it */
	ktime_t spk_lease_end;
	/*
	 * When and why the start gate was last closed ahead of the floor (the
	 * lease timer, or system sleep), for the withdrawal's log line.
	 */
	ktime_t spk_gate_closed_at;
	const char *spk_gate_reason;
	struct hrtimer spk_lease_timer;
	/* The floor after a lease ran out; takes spk_mutex */
	struct kthread_worker *spk_worker;
	struct kthread_work spk_expire_work;
	/* Stops the speaker streams; never takes spk_mutex (see macaudio_spk_grant()) */
	struct kthread_worker *spk_stop_worker;
	struct kthread_work spk_stop_work;
	struct notifier_block pm_nb;
	/* The speaker volume controls, the driver's alone */
	struct snd_kcontrol **spk_vols;
	unsigned int spk_nr_vols;
	struct snd_ctl_elem_value *spk_uval;
	/*
	 * The tweeter amps' playback high-pass corners, the driver's too, and
	 * their safe item and each amp's original DT item while granted
	 */
	struct snd_kcontrol **spk_hpfs;
	unsigned int spk_nr_hpfs;
	unsigned int spk_hpf_safe;
	unsigned int *spk_hpf_granted;
	/*
	 * The owner's engine runs the crossover ahead of the amps, so a grant
	 * may open the tweeters' high-pass. Written under spk_mutex.
	 */
	struct snd_kcontrol *spk_xover_kctl;
	bool spk_xover;

	/*
	 * The interface in charge (enum macaudio_spk_iface): written under
	 * spk_mutex and volume_lock_mutex, read with either or without both.
	 */
	int spk_iface;
	struct snd_kcontrol *spk_iface_kctl;
	/* "Speaker Volume Unlock" back in charge after a lease without owner */
	struct delayed_work spk_unowned_work;
	/* The amps' own gains, fixed at their limit while leased */
	struct snd_kcontrol **spk_amp_gains;
	unsigned int spk_nr_amp_gains;
	/* The software-gain CPU component, whose volume table top follows the interface */
	struct snd_soc_component *sw_gain_component;

	/*
	 * "Speaker Volume Unlock": the speakersafetyd / speakerguardd 0.1
	 * interface (see macaudio_vlimit_update()).
	 */
	int speaker_sample_rate;
	struct snd_kcontrol *speaker_sample_rate_kctl;

	struct mutex volume_lock_mutex;
	bool speaker_volume_unlocked;
	bool speaker_volume_was_locked;
	struct snd_kcontrol *speaker_lock_kctl;
	struct snd_ctl_file *speaker_lock_owner;
	u64 bes_active;
	bool speaker_lock_timeout_enabled;
	ktime_t speaker_lock_timeout;
	ktime_t speaker_lock_remain;
	struct delayed_work lock_timeout_work;
	struct work_struct lock_update_work;
	/*
	 * The speaker volumes as the system-sleep lock found them. Locking clips
	 * them to its ceiling and unlocking only lifts the ceiling; the daemon
	 * keeps its lease across the sleep and does not set them again.
	 */
	struct macaudio_pm_volume *pm_volumes;
	unsigned int pm_nr_volumes;
	/* Allocated with them, so that setting them again cannot fail */
	struct snd_ctl_elem_value *pm_uval;

	/*
	 * Software-gain amplifier: a lower gain cannot change samples that
	 * were converted before it was set, so every lock event bumps the
	 * epoch and stops the front-end stream; it is refused until it has
	 * been prepared (and its buffer cleared) under the new epoch.
	 */
	atomic_t sw_gain_lock_epoch;
	atomic_t sw_gain_prepared_epoch;
	atomic_t sw_gain_limit_failed;
	struct work_struct sw_gain_stop_work;

	/*
	 * A lock that did not take on every speaker volume (a control write
	 * failed) is a failed lock: the playback streams are stopped and
	 * refused until the driver is reloaded, since a volume above the safe
	 * value may be left behind.
	 */
	bool speaker_lock_failed;
	struct work_struct lock_fail_work;
};

static int please_blow_up_my_speakers;
module_param(please_blow_up_my_speakers, int, 0644);
MODULE_PARM_DESC(please_blow_up_my_speakers, "Deprecated: cannot bypass speaker admission or protection");

/* "Speaker Protection" owns the speakers, not "Speaker Volume Unlock" */
static bool macaudio_spk_leased(const struct macaudio_snd_data *ma)
{
	return READ_ONCE(ma->spk_iface) == MACAUDIO_SPK_IFACE_LEASE;
}


SND_SOC_DAILINK_DEFS(primary,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-0")), // CPU
	DAILINK_COMP_ARRAY(COMP_DUMMY()), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY())); // platform (filled at runtime)

SND_SOC_DAILINK_DEFS(secondary,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-1")), // CPU
	DAILINK_COMP_ARRAY(COMP_DUMMY()), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(sense,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-2")), // CPU
	DAILINK_COMP_ARRAY(COMP_DUMMY()), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * The front-ends are triggered before their back-ends (the default order,
 * made explicit here): macaudio_fe_trigger() runs first of all on START,
 * so a start it refuses leaves the DMA and the back-ends untouched.
 */
static struct snd_soc_dai_link macaudio_fe_links[] = {
	{
		.name = "Primary",
		.stream_name = "Primary",
		.dynamic = 1,
		.dpcm_merged_rate = 1,
		.dpcm_merged_chan = 1,
		.dpcm_merged_format = 1,
		.dai_fmt = MACAUDIO_DAI_FMT,
		.trigger = { SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE },
		SND_SOC_DAILINK_REG(primary),
	},
	{
		.name = "Secondary",
		.stream_name = "Secondary",
		.dynamic = 1,
		.dpcm_merged_rate = 1,
		.dpcm_merged_chan = 1,
		.dpcm_merged_format = 1,
		.dai_fmt = MACAUDIO_DAI_FMT,
		.playback_only = 1,
		.trigger = { SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE },
		SND_SOC_DAILINK_REG(secondary),
	},
	{
		.name = "Speaker Sense",
		.stream_name = "Speaker Sense",
		.capture_only = 1,
		.dynamic = 1,
		.dai_fmt = (SND_SOC_DAIFMT_I2S | \
					SND_SOC_DAIFMT_CBP_CFP | \
					SND_SOC_DAIFMT_GATED | \
					SND_SOC_DAIFMT_IB_IF),
		SND_SOC_DAILINK_REG(sense),
	},
};

static_assert(ARRAY_SIZE(macaudio_fe_links) == MACAUDIO_NUM_FE_LINKS);

static struct macaudio_link_props macaudio_fe_link_props[] = {
	{
		/*
		 * Primary FE
		 *
		 * The bclk ratio at 64 for the primary frontend is important
		 * to ensure that the headphones codec's idea of left and right
		 * in a stereo stream over I2S fits in nicely with everyone else's.
		 * (This is until the headphones codec's driver supports
		 * set_tdm_slot.)
		 *
		 * The low bclk ratio precludes transmitting more than two
		 * channels over I2S, but that's okay since there is the secondary
		 * FE for speaker arrays anyway.
		 */
		.bclk_ratio = 64,
	},
	{
		/*
		 * Secondary FE
		 *
		 * Here we want frames plenty long to be able to drive all
		 * those fancy speaker arrays.
		 */
		.bclk_ratio = 256,
	},
	{
		.is_sense = 1,
	}
};

/*
 * "Speaker Volume Unlock": speaker protection by a userspace daemon
 * (speakersafetyd, speakerguardd 0.1), the default interface.
 *
 * The daemon holds SNDRV_CTL_IOCTL_ELEM_LOCK on "Speaker Volume Unlock" and
 * on every speaker volume control, and pings the unlock control with
 * SPEAKER_MAGIC_VALUE at least every SPEAKER_LOCK_TIMEOUT ms while the
 * speakers play. While it does, the volumes may go up to the amps' 0 dB;
 * otherwise they are held at the lock's ceiling. The daemon governs them
 * from its thermal model.
 */

static void macaudio_vlimit_unlock(struct macaudio_snd_data *ma, bool unlock)
{
	int ret, max, hits = 0;
	const char *name = volume_control_names[ma->cfg->amp];
	struct snd_kcontrol *kctl;
	bool failed = false;

	if (!name) {
		WARN_ON_ONCE(1);
		return;
	}

	switch (ma->cfg->amp) {
	case AMP_NONE:
		WARN_ON_ONCE(1);
		return;
	case AMP_TAS5770:
		if (unlock)
			max = TAS5770_0DB;
		else
			max = 1; //TAS5770_DB(ma->cfg->safe_vol);
		break;
	case AMP_SN012776:
		if (ma->cfg->spk_fixed_ceiling)
			max = SN012776_DB(ma->cfg->safe_vol);
		else if (unlock)
			max = SN012776_0DB;
		else
			max = 1; //SN012776_DB(ma->cfg->safe_vol);
		break;
	case AMP_SSM3515:
		if (unlock)
			max = SSM3515_0DB;
		else
			max = SSM3515_DB(ma->cfg->safe_vol);
		break;
	case AMP_MAX98360A:
		if (unlock)
			max = MAX98360A_0DB;
		else
			max = MAX98360A_DB(ma->cfg->safe_vol);
		break;
	}

	/*
	 * One control at a time: snd_soc_limit_volume() stops at the first
	 * control it cannot clip, which would leave the rest at their old
	 * limit and value.
	 */
	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if (!snd_soc_control_matches(kctl, name))
			continue;

		hits++;
		ret = snd_soc_limit_volume(&ma->card, kctl->id.name, max);
		if (ret < 0) {
			dev_err(ma->card.dev, "Failed to %slock volume %s: %d\n",
				unlock ? "un" : "", kctl->id.name, ret);
			failed = true;
		}
	}
	if (!hits)
		dev_err(ma->card.dev, "Failed to %slock volume %s: %d\n",
			unlock ? "un" : "", name, -EINVAL);

	/*
	 * With a software gain the limit is the whole protection: refuse to
	 * play rather than run the amplifier at full scale unlimited.
	 */
	if (ma->cfg->amp == AMP_MAX98360A)
		atomic_set(&ma->sw_gain_limit_failed, failed || !hits);

	if (failed && !unlock) {
		WRITE_ONCE(ma->speaker_lock_failed, true);
		schedule_work(&ma->lock_fail_work);
	}
}

/*
 * Stop every running playback stream of the card, or with sw_gain_only those
 * on software-gain links whose prepare predates the current lock epoch.
 */
static void macaudio_stop_playback(struct macaudio_snd_data *ma, bool sw_gain_only)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_pcm_substream *substream;
	unsigned long flags;

	/*
	 * No volume or control lock may be held while taking the stream lock.
	 * Pick the links from the runtime being walked: not every playback
	 * substream on the card carries a runtime as its private data.
	 */
	for_each_card_rtds(&ma->card, rtd) {
		/* A back-end's substream shares its front-end's runtime: stop front-ends only. */
		if (!rtd->pcm || rtd->dai_link->no_pcm)
			continue;
		if (sw_gain_only && !ma->link_props[rtd->dai_link->id].sw_gain)
			continue;
		substream = rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		if (!substream)
			continue;
		snd_pcm_stream_lock_irqsave(substream, flags);
		if (substream->runtime &&
		    (!sw_gain_only ||
		     atomic_read(&ma->sw_gain_prepared_epoch) !=
				atomic_read(&ma->sw_gain_lock_epoch))) {
			switch (substream->runtime->state) {
			case SNDRV_PCM_STATE_PREPARED:
			case SNDRV_PCM_STATE_RUNNING:
			case SNDRV_PCM_STATE_DRAINING:
			case SNDRV_PCM_STATE_PAUSED:
				snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
				break;
			default:
				break;
			}
		}
		snd_pcm_stream_unlock_irqrestore(substream, flags);
	}
}

/* Stop every running playback stream of the card after a failed lock. */
static void macaudio_lock_fail_work(struct work_struct *work)
{
	struct macaudio_snd_data *ma = container_of(work, struct macaudio_snd_data,
						  lock_fail_work);

	dev_crit(ma->card.dev, "speaker volume lock failed: stopping and refusing playback\n");
	macaudio_stop_playback(ma, false);
}

static void macaudio_pm_volumes_drop(struct macaudio_snd_data *ma)
{
	kfree(ma->pm_volumes);
	kfree(ma->pm_uval);
	ma->pm_volumes = NULL;
	ma->pm_nr_volumes = 0;
	ma->pm_uval = NULL;
}

/*
 * Keep the speaker volumes before the system-sleep lock clips them.
 *
 * Context: volume_lock_mutex and the card's controls_rwsem held, so the daemon
 * cannot set a volume between this and the clip.
 */
static void macaudio_pm_volumes_save(struct macaudio_snd_data *ma)
{
	const char *name = volume_control_names[ma->cfg->amp];
	struct snd_ctl_elem_value *uval = NULL;
	struct macaudio_pm_volume *vols = NULL;
	struct snd_kcontrol *kctl;
	unsigned int nr = 0, i = 0;
	int ret;

	lockdep_assert_held(&ma->volume_lock_mutex);
	lockdep_assert_held(&ma->card.snd_card->controls_rwsem);

	if (!ma->has_safety || !name || ma->card.snd_card->shutdown)
		return;
	/*
	 * Locked already, as after a sleep the daemon has not pinged since:
	 * the volumes are at the ceiling, and those from before it are kept.
	 */
	if (!ma->speaker_volume_unlocked)
		return;
	macaudio_pm_volumes_drop(ma);

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list)
		if (snd_soc_control_matches(kctl, name))
			nr++;
	if (!nr)
		return;

	vols = kzalloc_objs(*vols, nr);
	uval = kzalloc_obj(*uval);
	if (!vols || !uval)
		goto out;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if (!snd_soc_control_matches(kctl, name))
			continue;
		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		ret = kctl->get(kctl, uval);
		if (ret < 0) {
			dev_warn(ma->card.dev, "Failed to keep volume %s for the system sleep: %d\n",
				 kctl->id.name, ret);
			goto out;
		}
		vols[i].kctl = kctl;
		vols[i].value[0] = uval->value.integer.value[0];
		vols[i].value[1] = uval->value.integer.value[1];
		i++;
	}

	ma->pm_volumes = vols;
	ma->pm_nr_volumes = i;
	ma->pm_uval = uval;
	return;
out:
	kfree(uval);
	kfree(vols);
}

/*
 * At the unlock after a system sleep: the volumes the daemon had set.
 *
 * Context: volume_lock_mutex and the card's controls_rwsem held, so no volume
 * write races these.
 */
static void macaudio_pm_volumes_restore(struct macaudio_snd_data *ma)
{
	struct snd_ctl_elem_value *uval = ma->pm_uval;
	unsigned int i;
	int ret;

	if (!ma->pm_volumes)
		return;

	for (i = 0; i < ma->pm_nr_volumes; i++) {
		struct snd_kcontrol *kctl = ma->pm_volumes[i].kctl;

		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		uval->value.integer.value[0] = ma->pm_volumes[i].value[0];
		uval->value.integer.value[1] = ma->pm_volumes[i].value[1];
		ret = kctl->put(kctl, uval);
		if (ret < 0)
			dev_err(ma->card.dev, "Failed to restore volume %s: %d\n",
				kctl->id.name, ret);
		else if (ret > 0)
			snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &kctl->id);
	}
	macaudio_pm_volumes_drop(ma);
}

static void macaudio_vlimit_update(struct macaudio_snd_data *ma)
{
	int i;
	bool unlock = true;
	struct snd_kcontrol *kctl;
	const char *reason;

	/* Do nothing if there is no safety configured */
	if (!ma->has_safety)
		return;

	/*
	 * "Speaker Protection" owns the speakers: the lock is held at its
	 * ceiling until the interface switches back (macaudio_spk_leave_lease()).
	 */
	if (macaudio_spk_leased(ma))
		return;

	/* Check that someone is holding the main lock */
	if (!ma->speaker_lock_owner) {
		reason = "Main control not locked";
		unlock = false;
	}

	/* Check that the control has been pinged within the timeout */
	if (ma->speaker_lock_remain <= 0) {
		reason = "Lock timeout";
		unlock = false;
	}

	/*
	 * System sleep: userspace is frozen long before the card suspends (over
	 * half a second on J616s), and a playback stream left running then plays
	 * out its stale buffer and stops on an xrun into the still unmuted amps.
	 * Lock the volumes before the freeze instead of when the frozen daemon's
	 * lease runs out.
	 */
	if (ma->pm_quiesced) {
		reason = "System suspend";
		unlock = false;
	}

	/* Check that *every* limited control is locked by the same owner */
	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if(!snd_soc_control_matches(kctl, volume_control_names[ma->cfg->amp]))
			continue;

		for (i = 0; i < kctl->count; i++) {
			if (kctl->vd[i].owner != ma->speaker_lock_owner) {
				reason = "Not all child controls locked by the same process";
				unlock = false;
			}
		}
	}


	if (unlock != ma->speaker_volume_unlocked) {
		if (unlock) {
			dev_info(ma->card.dev, "Speaker volumes unlocked\n");
		} else  {
			dev_info(ma->card.dev, "Speaker volumes locked: %s\n", reason);
			/* The system-sleep lock is the kernel's own, not a lease the daemon lost */
			if (!ma->pm_quiesced)
				ma->speaker_volume_was_locked = true;
		}

		macaudio_vlimit_unlock(ma, unlock);
		if (unlock)
			macaudio_pm_volumes_restore(ma);
		if (!unlock && ma->cfg->amp == AMP_MAX98360A) {
			/* Samples already converted at the old gain must not play */
			atomic_inc(&ma->sw_gain_lock_epoch);
			schedule_work(&ma->sw_gain_stop_work);
		}
		ma->speaker_volume_unlocked = unlock;
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->speaker_lock_kctl->id);
	}
}

/*
 * The volume lock's part of a system sleep (see macaudio_pm_notify()), after
 * pm_quiesced has been set or cleared.
 *
 * Volume writes hold the controls for writing: none can land between
 * keeping the volumes and clipping them. Control writes take
 * volume_lock_mutex inside it as well.
 */
static void macaudio_vlimit_pm(struct macaudio_snd_data *ma, bool quiesce)
{
	down_read(&ma->card.snd_card->controls_rwsem);
	mutex_lock(&ma->volume_lock_mutex);
	/*
	 * The lease ends here, so after the resume the volumes stay locked until
	 * the daemon's next ping. Its clock (CLOCK_MONOTONIC) stops across s2idle:
	 * a lease left running would unlock them at resume and expire before the
	 * thawed daemon pings, which reports a lost lock and restarts it cold.
	 */
	if (quiesce) {
		cancel_delayed_work(&ma->lock_timeout_work);
		ma->speaker_lock_remain = 0;
		macaudio_pm_volumes_save(ma);
	}
	macaudio_vlimit_update(ma);
	mutex_unlock(&ma->volume_lock_mutex);
	up_read(&ma->card.snd_card->controls_rwsem);
}

static void macaudio_vlimit_enable_timeout(struct macaudio_snd_data *ma)
{
	mutex_lock(&ma->volume_lock_mutex);

	if (ma->speaker_lock_timeout_enabled) {
		mutex_unlock(&ma->volume_lock_mutex);
		return;
	}

	if (ma->speaker_lock_remain > 0) {
		ma->speaker_lock_timeout = ktime_add(ktime_get(), ma->speaker_lock_remain);
		schedule_delayed_work(&ma->lock_timeout_work, usecs_to_jiffies(ktime_to_us(ma->speaker_lock_remain)));
		dev_dbg(ma->card.dev, "Enabling volume limit timeout: %ld us left\n",
			(long)ktime_to_us(ma->speaker_lock_remain));
	}

	macaudio_vlimit_update(ma);

	ma->speaker_lock_timeout_enabled = true;
	mutex_unlock(&ma->volume_lock_mutex);
}

static void macaudio_vlimit_disable_timeout(struct macaudio_snd_data *ma)
{
	ktime_t now;

	mutex_lock(&ma->volume_lock_mutex);

	if (!ma->speaker_lock_timeout_enabled) {
		mutex_unlock(&ma->volume_lock_mutex);
		return;
	}

	now = ktime_get();

	cancel_delayed_work(&ma->lock_timeout_work);

	if (ktime_after(now, ma->speaker_lock_timeout))
		ma->speaker_lock_remain = 0;
	else if (ma->speaker_lock_remain > 0)
		ma->speaker_lock_remain = ktime_sub(ma->speaker_lock_timeout, now);

	dev_dbg(ma->card.dev, "Disabling volume limit timeout: %ld us left\n",
		(long)ktime_to_us(ma->speaker_lock_remain));

	macaudio_vlimit_update(ma);

	ma->speaker_lock_timeout_enabled = false;

	mutex_unlock(&ma->volume_lock_mutex);
}

static void macaudio_vlimit_timeout_work(struct work_struct *wrk)
{
        struct macaudio_snd_data *ma = container_of(to_delayed_work(wrk),
						    struct macaudio_snd_data, lock_timeout_work);
	ktime_t now, left;

	mutex_lock(&ma->volume_lock_mutex);

	/*
	 * A ping or a disabled timeout can have cancelled this work after it
	 * had already started running; neither must expire the lease.
	 */
	if (!ma->speaker_lock_timeout_enabled || !ma->speaker_lock_owner ||
	    ma->speaker_lock_remain <= 0)
		goto out;

	/* A ping renewed the lease while this work was already running */
	now = ktime_get();
	if (ktime_before(now, ma->speaker_lock_timeout)) {
		left = ktime_sub(ma->speaker_lock_timeout, now);
		schedule_delayed_work(&ma->lock_timeout_work,
				      usecs_to_jiffies(ktime_to_us(left)));
		goto out;
	}

	ma->speaker_lock_remain = 0;
	macaudio_vlimit_update(ma);

out:
	mutex_unlock(&ma->volume_lock_mutex);
}

static void macaudio_vlimit_update_work(struct work_struct *wrk)
{
        struct macaudio_snd_data *ma = container_of(wrk,
						    struct macaudio_snd_data, lock_update_work);

	/* An unlock here sets the kept volumes again: no volume write may race it */
	down_read(&ma->card.snd_card->controls_rwsem);
	if (ma->bes_active)
		macaudio_vlimit_enable_timeout(ma);
	else
		macaudio_vlimit_disable_timeout(ma);
	up_read(&ma->card.snd_card->controls_rwsem);
}

static bool macaudio_sw_gain_pcm(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];

	return props->sw_gain && substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static void macaudio_sw_gain_stop_work(struct work_struct *work)
{
	struct macaudio_snd_data *ma = container_of(work, struct macaudio_snd_data,
						  sw_gain_stop_work);

	macaudio_stop_playback(ma, true);
}

/*
 * Speaker protection ownership: "Speaker Protection", a protection engine
 * in the signal path. It owns the speakers only while it is the interface
 * ("Speaker Protection Interface", see macaudio_spk_iface_put()); under
 * "Speaker Volume Unlock" an owner may hold the control, but nothing is
 * granted and its locks and releases touch no amp.
 *
 * While leased, the speakers play only while a protection engine owns them. Without an
 * owner, or once its lease has run out, every speaker amp sits at the safe
 * level and no speaker playback can start: that is the hardware floor, and it
 * holds from the card's registration on.
 *
 * The owner is the control file that holds SNDRV_CTL_IOCTL_ELEM_LOCK on
 * "Speaker Protection", taken by a CAP_SYS_ADMIN process: the speakers are
 * not a seat user's to own. Writing a lease length in milliseconds
 * (SPEAKER_LEASE_MIN_MS to SPEAKER_LEASE_MAX_MS) to it grants protection
 * for that long: the amps go to their fixed operating gain and speaker
 * playback may start. The owner renews it by writing again before it runs
 * out; every write needs CAP_SYS_ADMIN too. Writing 0, unlocking the control
 * or closing the file ends it at once: no start from then on, and on amps
 * with a digital gain the stream still playing is ramped down within the
 * lease (at most SPEAKER_RAMP_MAX_US) before it stops, so that the stop
 * does not click.
 *
 * The amps are only written on a powered card: a withdrawal while the card
 * suspends or resumes (the owner dying at thaw) closes the gate and stops
 * the streams at once and waits for the resume to finish before it puts
 * the amps at the safe level. The ownership controls are INACTIVE, so that
 * alsactl's restore never writes them.
 *
 * A lease that runs out closes the start gate at its end, from a hard
 * hrtimer that no thread can delay, and every start checks the lease's end
 * itself as well. Two SCHED_FIFO workers then stop speaker playback and put
 * the amps at the safe level. The owner's next write fails with -ETIMEDOUT
 * and grants nothing, so an owner that stalled learns of it before it plays
 * again; so does an owner that was frozen across a system sleep. Reads
 * return 1 while protection is granted, 0 otherwise, and every change is
 * notified.
 *
 * On machines that open the tweeters' playback high-pass while granted, a
 * grant opens it only for an owner that has declared, on the boolean
 * "Speaker Protection Crossover", that its engine passes every speaker
 * channel through the machine's own crossover and high-pass ahead of the
 * amps. Without that declaration the tweeters stay at their safe corner
 * while granted. The owner writes it with CAP_SYS_ADMIN, through its own
 * ELEM_LOCK on that control (taken while it owns "Speaker Protection"),
 * and only while protection is not granted: a grant latches it, and a
 * write meanwhile fails with -EBUSY. Any other write fails with -EPERM.
 * The declaration goes back to 0 when the owner unlocks "Speaker
 * Protection" or closes its file, and when a new owner takes it.
 *
 * The speaker volume controls belong to the driver: userspace can read them
 * but never write them. While protection is granted they hold the amps at a
 * fixed, known gain; all attenuation is the engine's, in its signal path.
 */

static long macaudio_spk_gain(const struct macaudio_snd_data *ma, bool granted)
{
	switch (ma->cfg->amp) {
	case AMP_TAS5770:
		return granted ? TAS5770_0DB : 0;
	case AMP_SN012776:
		if (!granted)
			return 0;
		/* The volumes' limit is the ceiling, so is the operating gain */
		return ma->cfg->spk_fixed_ceiling ? SN012776_DB(ma->cfg->safe_vol) :
						    SN012776_0DB;
	case AMP_MAX98360A:
		return granted ? MAX98360A_0DB : MAX98360A_DB(ma->cfg->safe_vol);
	default:
		return -EINVAL;
	}
}

#define MACAUDIO_TWEETER_HPF_SAFE	"800 Hz"
#define MACAUDIO_TWEETER_HPF_NAME	"* Tweeter HPF Corner Frequency"

/*
 * Set every tweeter's playback high-pass corner to its granted or its safe
 * item. Returns 0, or a negative error if any write failed.
 *
 * Context: spk_mutex held.
 */
static int macaudio_spk_set_hpf(struct macaudio_snd_data *ma, bool granted)
{
	struct snd_ctl_elem_value *uval = ma->spk_uval;
	unsigned int item;
	struct snd_kcontrol *kctl;
	int ret, err = 0;
	unsigned int i;

	lockdep_assert_held(&ma->spk_mutex);

	if (!ma->spk_nr_hpfs)
		return 0;
	if (!uval)
		return -EINVAL;

	for (i = 0; i < ma->spk_nr_hpfs; i++) {
		kctl = ma->spk_hpfs[i];
		item = granted ? ma->spk_hpf_granted[i] : ma->spk_hpf_safe;
		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		uval->value.enumerated.item[0] = item;
		ret = kctl->put(kctl, uval);
		if (ret < 0) {
			dev_err(ma->card.dev, "Failed to set %s to item %u: %d\n",
				kctl->id.name, item, ret);
			err = ret;
		} else if (ret > 0) {
			snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &kctl->id);
		}
	}
	return err;
}

/*
 * Set every speaker volume control to @gain, notifying userspace of the
 * change if @notify. Returns the number of controls set, or a negative error
 * if any write failed or none exists.
 *
 * Context: spk_mutex held.
 */
static int macaudio_spk_set_level(struct macaudio_snd_data *ma, long gain, bool notify)
{
	struct snd_ctl_elem_value *uval = ma->spk_uval;
	struct snd_kcontrol *kctl;
	int ret, err = 0;
	unsigned int i;

	lockdep_assert_held(&ma->spk_mutex);

	if (gain < 0 || !uval)
		return -EINVAL;

	for (i = 0; i < ma->spk_nr_vols; i++) {
		kctl = ma->spk_vols[i];
		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		/* Both channels of a stereo volume; a mono one reads the first */
		uval->value.integer.value[0] = gain;
		uval->value.integer.value[1] = gain;
		ret = kctl->put(kctl, uval);
		if (ret < 0) {
			dev_err(ma->card.dev, "Failed to set %s to %ld: %d\n",
				kctl->id.name, gain, ret);
			err = ret;
		} else if (ret > 0 && notify) {
			snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &kctl->id);
		}
	}

	if (!ma->spk_nr_vols)
		return -ENOENT;
	return err ? err : ma->spk_nr_vols;
}

/* Every speaker volume control at the operating gain or the safe level */
static int macaudio_spk_set_gain(struct macaudio_snd_data *ma, bool granted)
{
	return macaudio_spk_set_level(ma, macaudio_spk_gain(ma, granted), true);
}

/*
 * The ramp's steps below the operating gain, in dB, to the floor after the
 * last: about even in loudness, so each step is a small one.
 */
static const s8 macaudio_spk_ramp_db[] = {
	-2, -4, -6, -9, -12, -16, -20, -26, -32, -40, -50,
};

/* Amps whose volume is a digital gain in 0.5 dB steps with mute at 0 */
static bool macaudio_spk_ramps(const struct macaudio_snd_data *ma)
{
	return ma->cfg->amp == AMP_SN012776 || ma->cfg->amp == AMP_TAS5770;
}

/*
 * Ramp the amps from the operating gain towards the floor while the
 * stream still plays, so that its stop afterwards does not click. The
 * start gate is already closed. Stops at @deadline, whatever step it has
 * reached: the floor follows at once.
 *
 * Context: spk_mutex held, the card powered (D0).
 */
static void macaudio_spk_ramp_down(struct macaudio_snd_data *ma, ktime_t deadline)
{
	ktime_t next = ktime_get();
	unsigned int i;
	s64 wait;

	lockdep_assert_held(&ma->spk_mutex);

	for (i = 0; i < ARRAY_SIZE(macaudio_spk_ramp_db); i++) {
		/* A fixed ceiling is the operating gain: start below it */
		if (ma->cfg->spk_fixed_ceiling &&
		    macaudio_spk_ramp_db[i] >= ma->cfg->safe_vol)
			continue;
		if (!ktime_before(ktime_get(), deadline))
			return;
		/* A failure here is the floor's to catch; it notifies the end */
		macaudio_spk_set_level(ma, SN012776_DB(macaudio_spk_ramp_db[i]), false);
		next = ktime_add_us(next, SPEAKER_RAMP_STEP_US);
		wait = ktime_us_delta(min(next, deadline), ktime_get());
		if (wait > 0)
			usleep_range(wait, wait + 50);
	}
}

/* A back-end of the speakers is connected to this front-end substream. */
static bool macaudio_fe_drives_speakers(struct macaudio_snd_data *ma,
					struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm;

	for_each_dpcm_be(fe, stream, dpcm)
		if (ma->link_props[dpcm->be->dai_link->id].is_speakers)
			return true;
	return false;
}

/* Stop every running front-end that plays to the speakers. */
static void macaudio_stop_speaker_playback(struct macaudio_snd_data *ma)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_pcm_substream *substream;
	unsigned long flags;

	/*
	 * No mutex may be held that a trigger takes; the stream lock orders
	 * this against a concurrent start. A back-end's substream shares its
	 * front-end's runtime: stop front-ends only.
	 */
	for_each_card_rtds(&ma->card, rtd) {
		if (!rtd->pcm || rtd->dai_link->no_pcm)
			continue;
		substream = rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		if (!substream)
			continue;
		snd_pcm_stream_lock_irqsave(substream, flags);
		if (substream->runtime &&
		    macaudio_fe_drives_speakers(ma, rtd, SNDRV_PCM_STREAM_PLAYBACK)) {
			switch (substream->runtime->state) {
			case SNDRV_PCM_STATE_RUNNING:
			case SNDRV_PCM_STATE_DRAINING:
			case SNDRV_PCM_STATE_PAUSED:
				snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
				break;
			default:
				break;
			}
		}
		snd_pcm_stream_unlock_irqrestore(substream, flags);
	}
}

static void macaudio_spk_stop_work(struct kthread_work *work)
{
	struct macaudio_snd_data *ma = container_of(work, struct macaudio_snd_data,
						  spk_stop_work);

	macaudio_stop_speaker_playback(ma);
}

/*
 * Speaker playback may start: protection is granted and its lease has not
 * run out. Every start checks this (the front-end's and the back-end's
 * trigger), so a lease that ends refuses the next start at once, whether or
 * not the lease timer has run.
 *
 * Context: any, the stream lock held.
 */
static bool macaudio_spk_admitted(struct macaudio_snd_data *ma)
{
	/* Pairs with the release in macaudio_spk_grant(): the lease end is set */
	if (!smp_load_acquire(&ma->spk_granted) || READ_ONCE(ma->spk_floor_failed))
		return false;
	return ktime_before(ktime_get(), READ_ONCE(ma->spk_lease_end));
}

/*
 * Withdraw protection: no new speaker playback, the running one stopped and
 * every amp at the safe level. A safe level that did not take on every amp
 * is a failed floor: speaker playback stays refused until the driver is
 * reloaded.
 *
 * Context: spk_mutex held.
 */
static void __macaudio_spk_floor(struct macaudio_snd_data *ma, const char *reason, bool ramp)
{
	struct snd_card *card = ma->card.snd_card;
	bool was = ma->spk_active;
	/* The lease timer or a system sleep closed the gate before this */
	bool closed_ahead = was && !READ_ONCE(ma->spk_granted);
	ktime_t deadline;
	int powered;

	lockdep_assert_held(&ma->spk_mutex);

	WRITE_ONCE(ma->spk_granted, false);
	ma->spk_active = false;
	if (ma->spk_dead)
		return;
	/* The callback never takes spk_mutex */
	hrtimer_cancel(&ma->spk_lease_timer);

	/*
	 * A trigger refuses the start from here on. While the lease still
	 * runs, the amps ramp down under the playing stream first; never
	 * past the lease, and only on a powered card, whose stream can play.
	 */
	if (ramp && was && !closed_ahead && macaudio_spk_ramps(ma)) {
		deadline = min(READ_ONCE(ma->spk_lease_end),
			       ktime_add_us(ktime_get(), SPEAKER_RAMP_MAX_US));
		snd_power_ref(card);
		if (snd_power_get_state(card) == SNDRV_CTL_POWER_D0)
			macaudio_spk_ramp_down(ma, deadline);
		snd_power_unref(card);
	}
	/* Stop what already runs. */
	kthread_queue_work(ma->spk_stop_worker, &ma->spk_stop_work);

	/*
	 * The amps are written only on a powered card. From the owner's file
	 * closing, or the lease's end, this may run while the card suspends
	 * or resumes: its codecs then sync their registers and ASoC brings
	 * the streams back, and an amp write in the middle races both. The
	 * floor waits for the resume to finish; a stream cannot start before
	 * it does, and the gate is closed. A card going away (-ENODEV) still
	 * gets the floor written.
	 */
	powered = snd_power_ref_and_wait(card);

	if (macaudio_spk_set_gain(ma, false) < 0) {
		WRITE_ONCE(ma->spk_floor_failed, true);
		dev_crit(ma->card.dev,
			 "speaker safe level not set on every amp: speaker playback refused\n");
	}
	/* The amps are down first: the tweeters' safe filter goes back after */
	if (macaudio_spk_set_hpf(ma, false) < 0) {
		WRITE_ONCE(ma->spk_floor_failed, true);
		dev_crit(ma->card.dev,
			 "tweeter safe high-pass not set on every amp: speaker playback refused\n");
	}
	if (!powered)
		snd_power_unref(card);

	if (was && ma->spk_kctl) {
		if (closed_ahead)
			dev_info(ma->card.dev,
				 "Speaker protection withdrawn: %s (start gate closed by %s %lld us before)\n",
				 reason, READ_ONCE(ma->spk_gate_reason) ?: "?",
				 ktime_us_delta(ktime_get(), READ_ONCE(ma->spk_gate_closed_at)));
		else
			dev_info(ma->card.dev, "Speaker protection withdrawn: %s\n", reason);
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->spk_kctl->id);
	}
}

/* Withdraw protection at once: the lease ran out, a sleep, a failure */
static void macaudio_spk_floor(struct macaudio_snd_data *ma, const char *reason)
{
	__macaudio_spk_floor(ma, reason, false);
}

/*
 * Withdraw protection from a lease that still runs: released, the owner
 * gone or replaced. The amps ramp down before the stream stops.
 */
static void macaudio_spk_floor_ramped(struct macaudio_snd_data *ma, const char *reason)
{
	__macaudio_spk_floor(ma, reason, true);
}

/*
 * Close the start gate and stop the speaker streams, ahead of the floor
 * that follows under spk_mutex: from the lease timer and from system sleep,
 * which must not wait for anything first.
 *
 * Context: any, including hard IRQ.
 */
static void macaudio_spk_close_gate(struct macaudio_snd_data *ma, const char *why)
{
	if (READ_ONCE(ma->spk_granted)) {
		WRITE_ONCE(ma->spk_gate_closed_at, ktime_get());
		WRITE_ONCE(ma->spk_gate_reason, why);
	}
	WRITE_ONCE(ma->spk_granted, false);
	kthread_queue_work(ma->spk_stop_worker, &ma->spk_stop_work);
}

/*
 * Grant protection for @ms from now.
 *
 * Context: spk_mutex held, an owner present and protection not granted.
 */
static int macaudio_spk_grant(struct macaudio_snd_data *ma, long ms)
{
	int ret, refused;

	lockdep_assert_held(&ma->spk_mutex);

	if (READ_ONCE(ma->spk_floor_failed))
		return -EIO;

	/*
	 * A withdrawal queued the stop of every speaker stream: it must have
	 * run before the amps go back to the operating gain, or a stream it
	 * was meant to stop would play on. The stop worker runs nothing that
	 * takes spk_mutex or the card's controls_rwsem, so it cannot wait for
	 * this caller.
	 */
	kthread_flush_work(&ma->spk_stop_work);

	/*
	 * Only a powered card is granted: the amps may be syncing their
	 * registers in a resume. (A write from userspace has waited for it
	 * already.)
	 */
	if (snd_power_get_state(ma->card.snd_card) != SNDRV_CTL_POWER_D0)
		return -EBUSY;

	/*
	 * The tweeters' filter opens before the amps go up, never after, and
	 * only for an engine that runs the crossover ahead of them.
	 */
	ret = macaudio_spk_set_hpf(ma, ma->spk_xover);
	if (ret < 0) {
		macaudio_spk_floor(ma, "tweeter high-pass not set");
		return ret;
	}
	ret = macaudio_spk_set_gain(ma, true);
	if (ret < 0) {
		macaudio_spk_floor(ma, "operating gain not set");
		return ret;
	}
	/* A system sleep began while the amps went up: it closed the gate already */
	if (READ_ONCE(ma->pm_quiesced)) {
		macaudio_spk_floor(ma, "system sleep");
		return -EBUSY;
	}

	ma->spk_active = true;
	WRITE_ONCE(ma->spk_lease_end, ktime_add_ms(ktime_get(), ms));
	/* Pairs with the acquire in macaudio_spk_admitted() */
	smp_store_release(&ma->spk_granted, true);
	hrtimer_start(&ma->spk_lease_timer, ms_to_ktime(ms), HRTIMER_MODE_REL_HARD);
	refused = atomic_xchg(&ma->spk_refused_since_grant, 0);
	if (refused)
		dev_info(ma->card.dev, "Speaker protection granted (%d speaker starts refused before)\n",
			 refused);
	else
		dev_info(ma->card.dev, "Speaker protection granted\n");
	if (ma->spk_nr_hpfs && !ma->spk_xover)
		dev_info(ma->card.dev,
			 "No crossover declared: tweeter high-pass stays at the safe corner\n");
	if (ma->spk_kctl)
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &ma->spk_kctl->id);
	return 0;
}

/*
 * The lease's end, in hard IRQ context: no thread, however it is scheduled,
 * can hold off the closing of the start gate. The stop and the safe level
 * follow from the workers.
 */
static enum hrtimer_restart macaudio_spk_lease_timer(struct hrtimer *timer)
{
	struct macaudio_snd_data *ma = container_of(timer, struct macaudio_snd_data,
						    spk_lease_timer);
	ktime_t end = READ_ONCE(ma->spk_lease_end);

	/* Renewed while the timer was already firing */
	if (ktime_before(ktime_get(), end)) {
		hrtimer_set_expires(timer, end);
		return HRTIMER_RESTART;
	}
	if (!READ_ONCE(ma->spk_granted))
		return HRTIMER_NORESTART;

	macaudio_spk_close_gate(ma, "the lease's end");
	kthread_queue_work(ma->spk_worker, &ma->spk_expire_work);
	return HRTIMER_NORESTART;
}

static void macaudio_spk_expire_work(struct kthread_work *work)
{
	struct macaudio_snd_data *ma = container_of(work, struct macaudio_snd_data,
						    spk_expire_work);

	mutex_lock(&ma->spk_mutex);
	/*
	 * Only the timer clears the gate with the amps still at the operating
	 * gain; a release, a new owner or the owner's own late write may
	 * have dealt with it since.
	 */
	if (ma->spk_active && !READ_ONCE(ma->spk_granted)) {
		ma->spk_lapsed = true;
		macaudio_spk_floor(ma, "lease expired");
	}
	mutex_unlock(&ma->spk_mutex);
}

static struct kthread_worker *macaudio_spk_worker(const char *what)
{
	struct sched_attr attr = {
		.size = sizeof(attr),
		.sched_policy = SCHED_FIFO,
		.sched_priority = SPEAKER_WORKER_PRIO,
	};
	struct kthread_worker *worker;
	int ret;

	worker = kthread_run_worker(0, "macaudio-%s", what);
	if (IS_ERR(worker))
		return worker;
	ret = sched_setattr_nocheck(worker->task, &attr);
	if (ret) {
		kthread_destroy_worker(worker);
		return ERR_PTR(ret);
	}
	return worker;
}

static void macaudio_spk_fini(void *data)
{
	struct macaudio_snd_data *ma = data;

	cancel_delayed_work_sync(&ma->spk_unowned_work);
	hrtimer_cancel(&ma->spk_lease_timer);
	/* The expiry may queue a stop: its worker goes first */
	if (!IS_ERR_OR_NULL(ma->spk_worker))
		kthread_destroy_worker(ma->spk_worker);
	if (!IS_ERR_OR_NULL(ma->spk_stop_worker))
		kthread_destroy_worker(ma->spk_stop_worker);
	ma->spk_worker = NULL;
	ma->spk_stop_worker = NULL;
}

static int macaudio_pm_notify(struct notifier_block *nb, unsigned long action,
			      void *data);

static void macaudio_spk_unowned_work(struct work_struct *work);

/* The lease timer, the workers and the sleep notifier, until macaudio_spk_fini() */
static int macaudio_spk_init(struct macaudio_snd_data *ma)
{
	hrtimer_setup(&ma->spk_lease_timer, macaudio_spk_lease_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_HARD);
	kthread_init_work(&ma->spk_expire_work, macaudio_spk_expire_work);
	INIT_DELAYED_WORK(&ma->spk_unowned_work, macaudio_spk_unowned_work);
	kthread_init_work(&ma->spk_stop_work, macaudio_spk_stop_work);
	/* First in the chain: no other driver's sleep preparation goes before the floor */
	ma->pm_nb.notifier_call = macaudio_pm_notify;
	ma->pm_nb.priority = INT_MAX;

	ma->spk_worker = macaudio_spk_worker("spk-floor");
	if (IS_ERR(ma->spk_worker))
		return PTR_ERR(ma->spk_worker);
	ma->spk_stop_worker = macaudio_spk_worker("spk-stop");
	if (IS_ERR(ma->spk_stop_worker)) {
		int ret = PTR_ERR(ma->spk_stop_worker);

		ma->spk_stop_worker = NULL;
		macaudio_spk_fini(ma);
		return ret;
	}
	return 0;
}

/*
 * System sleep: protection is withdrawn at PM_SUSPEND_PREPARE (and
 * PM_HIBERNATION_PREPARE), first in the notifier chain and before any task
 * freezes, not in a device suspend callback: a sleep that stops short of
 * the devices (pm_test=freezer) withdraws it too. The gate closes and the
 * streams stop before anything can wait; the safe level follows under
 * spk_mutex. The lease timer is never stopped without that floor, so a
 * lease still ends at its deadline while the floor waits.
 *
 * The owner was frozen with the rest of userspace, so the state of its
 * engine is unknown: its first write after the resume fails with
 * -ETIMEDOUT, as after any stall, and the one after it grants.
 */
static int macaudio_pm_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct macaudio_snd_data *ma = container_of(nb, struct macaudio_snd_data, pm_nb);
	ktime_t t0;
	s64 waited;

	/* A card without speakers (the J616s jack) has nothing to floor */
	if (!ma->has_speakers)
		return NOTIFY_DONE;

	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		/*
		 * Before anything that can wait, spk_mutex included: a writer
		 * that holds it may be stuck behind the sleep itself. No grant
		 * and no start from here on, and the running streams stop.
		 */
		WRITE_ONCE(ma->pm_quiesced, true);
		macaudio_spk_close_gate(ma, "system sleep");
		t0 = ktime_get();
		mutex_lock(&ma->spk_mutex);
		waited = ktime_ms_delta(ktime_get(), t0);
		if (waited > 50)
			dev_warn(ma->card.dev,
				 "speaker floor for system sleep waited %lld ms for the protection lock\n",
				 waited);
		if (macaudio_spk_leased(ma))
			macaudio_spk_floor(ma, "system sleep");
		mutex_unlock(&ma->spk_mutex);
		macaudio_vlimit_pm(ma, true);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		mutex_lock(&ma->spk_mutex);
		WRITE_ONCE(ma->pm_quiesced, false);
		if (ma->spk_owner)
			ma->spk_lapsed = true;
		mutex_unlock(&ma->spk_mutex);
		macaudio_vlimit_pm(ma, false);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

/*
 * The "Speaker Protection Interface" switch.
 *
 * At every card bind "Speaker Volume Unlock" is in charge, exactly as on a
 * kernel without the lease. A CAP_SYS_ADMIN write of "Speaker Protection"
 * hands the speakers to the lease, through the floor of both: the volume
 * lock's ceiling first, then no speaker start, the running streams stopped,
 * the lease's safe level, the amps' own gains fixed at their limit, and only
 * then the ceiling lifted, with every volume still at the safe level. A
 * write of "Speaker Volume Unlock", or SPEAKER_LEASE_UNOWNED_MS without an
 * owner of "Speaker Protection", hands them back through the lease's floor:
 * the volumes stay at the safe level under the lock's ceiling, and go up
 * only once a daemon holds the lock and pings it, as after a cold bind. Its
 * first ping after the switch fails with -ETIMEDOUT: the lock was lost.
 *
 * While leased the lock's ping fails with -EBUSY, and the speaker volumes
 * and amp gains are the driver's: userspace writes fail with -EPERM.
 *
 * The switch is state for the boot, not a mixer setting: INACTIVE keeps
 * alsactl off it. Whatever makes it persist (the protection engine's
 * service) writes it again at every boot.
 */

static const char * const macaudio_spk_iface_texts[] = {
	[MACAUDIO_SPK_IFACE_UNLOCK] = "Speaker Volume Unlock",
	[MACAUDIO_SPK_IFACE_LEASE] = "Speaker Protection",
};

static void macaudio_spk_iface_notify(struct macaudio_snd_data *ma)
{
	if (ma->spk_dead)
		return;
	if (ma->spk_iface_kctl)
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->spk_iface_kctl->id);
	if (ma->speaker_lock_kctl)
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->speaker_lock_kctl->id);
	if (ma->spk_kctl)
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->spk_kctl->id);
}

/* The software gain's table top: the full scale only while leased */
static int macaudio_sw_gain_full_scale(struct macaudio_snd_data *ma, bool full)
{
	if (!ma->sw_gain_component)
		return 0;
	return snd_soc_component_write(ma->sw_gain_component,
				       MACAUDIO_SW_GAIN_REG_FULL_SCALE, full);
}

/*
 * Every amp's own gain at its limit, a fixed operating point, as the
 * engine's tuning assumes. Returns 0 or a negative error.
 *
 * Context: spk_mutex held.
 */
static int macaudio_spk_fix_amp_gains(struct macaudio_snd_data *ma)
{
	struct snd_ctl_elem_value *uval = ma->spk_uval;
	struct snd_ctl_elem_info *uinfo;
	struct snd_kcontrol *kctl;
	unsigned int i;
	int ret = 0;

	lockdep_assert_held(&ma->spk_mutex);

	if (!ma->spk_nr_amp_gains)
		return 0;
	if (!uval)
		return -EINVAL;
	uinfo = kzalloc_obj(*uinfo);
	if (!uinfo)
		return -ENOMEM;

	for (i = 0; i < ma->spk_nr_amp_gains; i++) {
		kctl = ma->spk_amp_gains[i];
		memset(uinfo, 0, sizeof(*uinfo));
		uinfo->id = kctl->id;
		ret = kctl->info(kctl, uinfo);
		if (ret < 0)
			break;
		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		uval->value.integer.value[0] = uinfo->value.integer.max;
		uval->value.integer.value[1] = uinfo->value.integer.max;
		ret = kctl->put(kctl, uval);
		if (ret < 0)
			break;
		if (ret > 0)
			snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &kctl->id);
		ret = 0;
	}
	if (ret < 0)
		dev_err(ma->card.dev, "Failed to fix %s at its limit: %d\n",
			kctl->id.name, ret);
	kfree(uinfo);
	return ret;
}

/*
 * Hand the speakers to "Speaker Protection".
 *
 * Context: the card's controls_rwsem and spk_mutex held, the card powered.
 */
static int macaudio_spk_enter_lease(struct macaudio_snd_data *ma)
{
	int ret;

	lockdep_assert_held(&ma->spk_mutex);

	if (ma->spk_dead)
		return -ENODEV;
	if (!ma->has_speakers || !ma->has_safety)
		return -ENODEV;
	if (READ_ONCE(ma->pm_quiesced))
		return -EBUSY;
	if (READ_ONCE(ma->spk_floor_failed) || READ_ONCE(ma->speaker_lock_failed))
		return -EIO;

	/* The volume lock's ceiling first: whatever a daemon had unlocked goes down */
	mutex_lock(&ma->volume_lock_mutex);
	cancel_delayed_work(&ma->lock_timeout_work);
	ma->speaker_lock_remain = 0;
	macaudio_pm_volumes_drop(ma);
	macaudio_vlimit_update(ma);
	if (READ_ONCE(ma->speaker_lock_failed)) {
		mutex_unlock(&ma->volume_lock_mutex);
		return -EIO;
	}
	/* No speaker start from here on: nothing is granted yet */
	WRITE_ONCE(ma->spk_iface, MACAUDIO_SPK_IFACE_LEASE);
	mutex_unlock(&ma->volume_lock_mutex);

	/* What still plays stops before the amps are touched */
	kthread_queue_work(ma->spk_stop_worker, &ma->spk_stop_work);
	kthread_flush_work(&ma->spk_stop_work);

	/* The lease's floor; a failed one refuses speaker playback for good */
	macaudio_spk_floor(ma, "interface switched");
	if (READ_ONCE(ma->spk_floor_failed))
		ret = -EIO;
	else
		ret = macaudio_spk_fix_amp_gains(ma);
	if (ret) {
		WRITE_ONCE(ma->spk_floor_failed, true);
		dev_crit(ma->card.dev,
			 "speaker floor for the lease not set: speaker playback refused\n");
		macaudio_spk_iface_notify(ma);
		return ret;
	}

	/* The volumes stay at the safe level: the ceiling no longer applies */
	mutex_lock(&ma->volume_lock_mutex);
	macaudio_vlimit_unlock(ma, true);
	mutex_unlock(&ma->volume_lock_mutex);
	ret = macaudio_sw_gain_full_scale(ma, true);
	if (ret < 0)
		dev_err(ma->card.dev, "software gain full scale not set: %d\n", ret);

	if (!ma->spk_owner)
		mod_delayed_work(system_percpu_wq, &ma->spk_unowned_work,
				 msecs_to_jiffies(SPEAKER_LEASE_UNOWNED_MS));
	dev_info(ma->card.dev,
		 "Speaker protection interface: Speaker Protection (lease); Speaker Volume Unlock refused\n");
	macaudio_spk_iface_notify(ma);
	return 0;
}

/*
 * Hand the speakers back to "Speaker Volume Unlock", at its floor.
 *
 * Context: the card's controls_rwsem and spk_mutex held.
 */
static void macaudio_spk_leave_lease(struct macaudio_snd_data *ma, const char *why)
{
	lockdep_assert_held(&ma->spk_mutex);

	if (!macaudio_spk_leased(ma))
		return;
	cancel_delayed_work(&ma->spk_unowned_work);

	/* No start, the stream stopped, every amp at the safe level */
	macaudio_spk_floor_ramped(ma, why);
	if (!ma->spk_dead) {
		kthread_flush_work(&ma->spk_stop_work);
		if (macaudio_sw_gain_full_scale(ma, false) < 0)
			WRITE_ONCE(ma->spk_floor_failed, true);
	}

	mutex_lock(&ma->volume_lock_mutex);
	/* The lock's ceiling over the safe level; it lifts on a fresh ping only */
	if (!ma->spk_dead)
		macaudio_vlimit_unlock(ma, false);
	ma->speaker_volume_unlocked = false;
	cancel_delayed_work(&ma->lock_timeout_work);
	ma->speaker_lock_remain = 0;
	ma->speaker_lock_timeout = 0;
	macaudio_pm_volumes_drop(ma);
	/* A daemon still holding the lock learns that it lost it */
	ma->speaker_volume_was_locked = true;
	if (READ_ONCE(ma->spk_floor_failed) && !READ_ONCE(ma->speaker_lock_failed)) {
		WRITE_ONCE(ma->speaker_lock_failed, true);
		schedule_work(&ma->lock_fail_work);
	}
	if (ma->cfg->amp == AMP_MAX98360A) {
		/* Samples converted at the lease's gain must not play */
		atomic_inc(&ma->sw_gain_lock_epoch);
		schedule_work(&ma->sw_gain_stop_work);
	}
	WRITE_ONCE(ma->spk_iface, MACAUDIO_SPK_IFACE_UNLOCK);
	macaudio_vlimit_update(ma);
	mutex_unlock(&ma->volume_lock_mutex);

	dev_info(ma->card.dev, "Speaker protection interface: Speaker Volume Unlock (%s)\n", why);
	macaudio_spk_iface_notify(ma);
}

static void macaudio_spk_unowned_work(struct work_struct *work)
{
	struct macaudio_snd_data *ma = container_of(to_delayed_work(work),
						    struct macaudio_snd_data,
						    spk_unowned_work);

	/* The same locks as a control write, in the same order */
	down_write(&ma->card.snd_card->controls_rwsem);
	mutex_lock(&ma->spk_mutex);
	if (!ma->spk_owner && !ma->spk_dead)
		macaudio_spk_leave_lease(ma, "no owner of Speaker Protection");
	mutex_unlock(&ma->spk_mutex);
	up_write(&ma->card.snd_card->controls_rwsem);
}

static int macaudio_spk_iface_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	return snd_ctl_enum_info(uinfo, 1, ARRAY_SIZE(macaudio_spk_iface_texts),
				 macaudio_spk_iface_texts);
}

static int macaudio_spk_iface_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	uvalue->value.enumerated.item[0] = READ_ONCE(ma->spk_iface);
	return 0;
}

static int macaudio_spk_iface_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	unsigned int item = uvalue->value.enumerated.item[0];
	int ret;

	if (item >= ARRAY_SIZE(macaudio_spk_iface_texts))
		return -EINVAL;
	/* The speakers are not a seat user's to hand over */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&ma->spk_mutex);
	if (item == READ_ONCE(ma->spk_iface)) {
		ret = 0;
	} else if (item == MACAUDIO_SPK_IFACE_LEASE) {
		ret = macaudio_spk_enter_lease(ma);
		if (!ret)
			ret = 1;
	} else {
		macaudio_spk_leave_lease(ma, "switched by the administrator");
		ret = 1;
	}
	mutex_unlock(&ma->spk_mutex);
	return ret;
}

static int macaudio_spk_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SPEAKER_LEASE_MAX_MS;
	return 0;
}

static int macaudio_spk_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	uvalue->value.integer.value[0] = macaudio_spk_admitted(ma) ? 1 : 0;
	return 0;
}

static int macaudio_spk_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	long ms = uvalue->value.integer.value[0];
	int ret = 0;

	if (ms != 0 && (ms < SPEAKER_LEASE_MIN_MS || ms > SPEAKER_LEASE_MAX_MS))
		return -EINVAL;
	/* The owner's file may have been handed on: each write is checked */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&ma->spk_mutex);

	/* Only the file that holds the control's lock owns the speakers */
	if (!ma->spk_owner || ma->spk_dead) {
		ret = -EPERM;
		goto out;
	}

	/* "Speaker Volume Unlock" owns the speakers: nothing to grant or release */
	if (!macaudio_spk_leased(ma)) {
		ret = -EBUSY;
		goto out;
	}

	if (!ms) {
		ma->spk_lapsed = false;
		macaudio_spk_floor_ramped(ma, "released by the owner");
		goto out;
	}

	/* Before the stall report, which a resume sets: it must reach the owner */
	if (READ_ONCE(ma->pm_quiesced)) {
		ret = -EBUSY;
		goto out;
	}

	/* The owner stalled past its lease: tell it once, grant nothing */
	if (ma->spk_lapsed) {
		ma->spk_lapsed = false;
		ret = -ETIMEDOUT;
		goto out;
	}

	if (!ma->spk_active) {
		ret = macaudio_spk_grant(ma, ms);
		goto out;
	}

	/*
	 * A renewal. A lease that has already run out is not renewed, whether
	 * the timer has closed the gate yet or not: the owner stalled.
	 */
	if (!macaudio_spk_admitted(ma)) {
		macaudio_spk_floor(ma, "lease expired");
		ret = -ETIMEDOUT;
		goto out;
	}
	WRITE_ONCE(ma->spk_lease_end, ktime_add_ms(ktime_get(), ms));
	hrtimer_start(&ma->spk_lease_timer, ms_to_ktime(ms), HRTIMER_MODE_REL_HARD);
	/* The old lease ran out between the check and the new end */
	if (!READ_ONCE(ma->spk_granted)) {
		macaudio_spk_floor(ma, "lease expired");
		ret = -ETIMEDOUT;
	}
out:
	mutex_unlock(&ma->spk_mutex);
	return ret;
}

/*
 * The crossover declaration lapses with the ownership that made it.
 *
 * Context: spk_mutex held.
 */
static void macaudio_spk_xover_clear(struct macaudio_snd_data *ma)
{
	lockdep_assert_held(&ma->spk_mutex);

	if (!ma->spk_xover)
		return;
	WRITE_ONCE(ma->spk_xover, false);
	if (ma->spk_xover_kctl && !ma->spk_dead)
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->spk_xover_kctl->id);
}

static int macaudio_spk_lock(struct snd_kcontrol *kcontrol, struct snd_ctl_file *owner)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	/* The protection supervisor, not whoever has the seat's control device */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&ma->spk_mutex);
	/* A new owner starts without protection and grants it itself */
	ma->spk_owner = owner;
	ma->spk_lapsed = false;
	macaudio_spk_xover_clear(ma);
	if (macaudio_spk_leased(ma)) {
		cancel_delayed_work(&ma->spk_unowned_work);
		macaudio_spk_floor_ramped(ma, "new owner");
	}
	mutex_unlock(&ma->spk_mutex);
	return 0;
}

static void macaudio_spk_unlock(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	mutex_lock(&ma->spk_mutex);
	ma->spk_owner = NULL;
	ma->spk_lapsed = false;
	macaudio_spk_xover_clear(ma);
	if (macaudio_spk_leased(ma)) {
		macaudio_spk_floor_ramped(ma, "owner gone");
		/* A restarting engine takes it back in time; a dead one does not */
		if (!ma->spk_dead)
			mod_delayed_work(system_percpu_wq, &ma->spk_unowned_work,
					 msecs_to_jiffies(SPEAKER_LEASE_UNOWNED_MS));
	}
	mutex_unlock(&ma->spk_mutex);
}

static int macaudio_spk_xover_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	uvalue->value.integer.value[0] = READ_ONCE(ma->spk_xover);
	return 0;
}

/*
 * The owner declares, or withdraws, the crossover in its engine. ALSA
 * serializes this with every lock and unlock, file close included, and has
 * refused a write from any file but the one holding this control's lock.
 */
static int macaudio_spk_xover_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	long val = uvalue->value.integer.value[0];
	int ret;

	if (val != 0 && val != 1)
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&ma->spk_mutex);
	if (!ma->spk_owner || ma->spk_dead || kcontrol->vd[0].owner != ma->spk_owner) {
		ret = -EPERM;
	} else if (ma->spk_active || READ_ONCE(ma->spk_granted)) {
		/* The grant latched it: the next one after the floor takes a change */
		ret = -EBUSY;
	} else {
		ret = ma->spk_xover != !!val;
		WRITE_ONCE(ma->spk_xover, !!val);
	}
	mutex_unlock(&ma->spk_mutex);
	return ret;
}

/* Only the owner of "Speaker Protection" may hold the declaration's lock. */
static int macaudio_spk_xover_lock(struct snd_kcontrol *kcontrol, struct snd_ctl_file *owner)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	int ret = 0;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	mutex_lock(&ma->spk_mutex);
	if (!owner || owner != ma->spk_owner || ma->spk_dead)
		ret = -EPERM;
	mutex_unlock(&ma->spk_mutex);
	return ret;
}

#define MACAUDIO_SPK_XOVER_NAME	"Speaker Protection Crossover"

/*
 * The ownership controls are state between the kernel and the protection
 * supervisor, not mixer settings. INACTIVE keeps mixers and alsactl's
 * restore off them (the write would fail anyway: only the owner may),
 * VOLATILE says the kernel changes them on its own.
 */
#define MACAUDIO_SPK_OWNERSHIP_ACCESS (SNDRV_CTL_ELEM_ACCESS_READWRITE | \
				       SNDRV_CTL_ELEM_ACCESS_VOLATILE | \
				       SNDRV_CTL_ELEM_ACCESS_INACTIVE)

static const struct snd_kcontrol_new macaudio_spk_xover_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = MACAUDIO_SPK_XOVER_NAME,
	.access = MACAUDIO_SPK_OWNERSHIP_ACCESS,
	.info = snd_ctl_boolean_mono_info,
	.get = macaudio_spk_xover_get,
	.put = macaudio_spk_xover_put,
	.lock = macaudio_spk_xover_lock,
};

/* ALSA serializes this with protection lock/unlock, including file close. */
static int macaudio_sense_owner_check(struct macaudio_snd_data *ma, struct snd_ctl_file *file)
{
	if (!macaudio_spk_leased(ma))
		return 0;
	return ma->spk_owner && ma->spk_owner != file ? -EPERM : 0;
}

static int macaudio_sense_write_check(struct snd_kcontrol *kctl, struct snd_ctl_file *file)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(snd_soc_dapm_kcontrol_to_dapm(kctl));
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	return macaudio_sense_owner_check(ma, file);
}

/*
 * The speaker route may not change under a grant: the owner admitted the
 * front-end its engine drives, and another one routed to the speakers would
 * reach the amps without protection until the owner noticed. Re-route only
 * while protection is withdrawn.
 */
static int macaudio_route_check(struct macaudio_snd_data *ma)
{
	/* Pairs with the release in macaudio_spk_grant() */
	return smp_load_acquire(&ma->spk_granted) ? -EBUSY : 0;
}

static int macaudio_route_write_check(struct snd_kcontrol *kctl, struct snd_ctl_file *file)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(snd_soc_dapm_kcontrol_to_dapm(kctl));

	return macaudio_route_check(snd_soc_card_get_drvdata(card));
}

/*
 * Match both mono and prefixed amp DAPM sense switches, and the speaker
 * route; leave jack controls alone.
 */
static void macaudio_protect_sense(struct snd_soc_card *card)
{
	struct snd_kcontrol *kctl;

	list_for_each_entry(kctl, &card->snd_card->controls, list) {
		if (snd_soc_control_matches(kctl, "* ISENSE Switch") ||
		    snd_soc_control_matches(kctl, "* VSENSE Switch") ||
		    !strcmp(kctl->id.name, "ISENSE Switch") ||
		    !strcmp(kctl->id.name, "VSENSE Switch"))
			kctl->write_check = macaudio_sense_write_check;
		else if (!strcmp(kctl->id.name, "Speaker Playback Mux"))
			kctl->write_check = macaudio_route_write_check;
	}
}

/*
 * Speaker volumes and amp gains: the volume lock's daemon sets them under
 * "Speaker Volume Unlock"; while leased they are the driver's alone.
 */
static int macaudio_spk_owned_write_check(struct snd_kcontrol *kctl,
					  struct snd_ctl_file *file)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kctl);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(component->card);

	return macaudio_spk_leased(ma) ? -EPERM : 0;
}

/*
 * Keep the speaker volumes, which the lease sets, and refuse userspace
 * writes to them while leased (macaudio_spk_owned_write_check()).
 *
 * Context: before the card registers, or with the card's controls held.
 */
static int macaudio_spk_take_volumes(struct macaudio_snd_data *ma)
{
	const char *name = volume_control_names[ma->cfg->amp];
	struct device *dev = ma->card.dev;
	struct snd_kcontrol *kctl;
	unsigned int nr = 0;

	if (!name)
		return -EINVAL;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list)
		if (snd_soc_control_matches(kctl, name))
			nr++;
	if (!nr)
		return -ENOENT;

	ma->spk_vols = devm_kcalloc(dev, nr, sizeof(*ma->spk_vols), GFP_KERNEL);
	ma->spk_uval = devm_kzalloc(dev, sizeof(*ma->spk_uval), GFP_KERNEL);
	if (!ma->spk_vols || !ma->spk_uval)
		return -ENOMEM;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if (!snd_soc_control_matches(kctl, name))
			continue;
		kctl->write_check = macaudio_spk_owned_write_check;
		ma->spk_vols[ma->spk_nr_vols++] = kctl;
	}
	return 0;
}

static bool macaudio_is_amp_gain(const struct macaudio_snd_data *ma,
				 struct snd_kcontrol *kctl)
{
	if (ma->cfg->amp != AMP_TAS5770 && ma->cfg->amp != AMP_SN012776)
		return false;
	return snd_soc_control_matches(kctl, "* Amp Gain Volume") ||
	       !strcmp(kctl->id.name, "Amp Gain Volume");
}

/*
 * The amps' own gains ("Amp Gain Volume"): limited at fixup, theirs to set
 * under "Speaker Volume Unlock", fixed at the limit while leased.
 *
 * Context: before the card registers.
 */
static int macaudio_spk_take_amp_gains(struct macaudio_snd_data *ma)
{
	struct device *dev = ma->card.dev;
	struct snd_kcontrol *kctl;
	unsigned int nr = 0;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list)
		if (macaudio_is_amp_gain(ma, kctl))
			nr++;
	if (!nr)
		return 0;

	ma->spk_amp_gains = devm_kcalloc(dev, nr, sizeof(*ma->spk_amp_gains), GFP_KERNEL);
	if (!ma->spk_amp_gains)
		return -ENOMEM;
	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if (!macaudio_is_amp_gain(ma, kctl))
			continue;
		kctl->write_check = macaudio_spk_owned_write_check;
		ma->spk_amp_gains[ma->spk_nr_amp_gains++] = kctl;
	}
	return 0;
}

/* The item of enum control @kctl named @text, or a negative error */
static int macaudio_enum_item(struct snd_kcontrol *kctl, const char *text)
{
	struct snd_ctl_elem_info *uinfo;
	unsigned int i, items;
	int ret;

	uinfo = kzalloc_obj(*uinfo);
	if (!uinfo)
		return -ENOMEM;
	uinfo->id = kctl->id;
	ret = kctl->info(kctl, uinfo);
	if (!ret && uinfo->type != SNDRV_CTL_ELEM_TYPE_ENUMERATED)
		ret = -EINVAL;
	items = ret ? 0 : uinfo->value.enumerated.items;
	ret = -ENOENT;
	for (i = 0; i < items; i++) {
		uinfo->value.enumerated.item = i;
		if (kctl->info(kctl, uinfo))
			break;
		if (!strcmp(uinfo->value.enumerated.name, text)) {
			ret = i;
			break;
		}
	}
	kfree(uinfo);
	return ret;
}

/*
 * The tweeters' playback high-pass corners are the driver's where the
 * machine opens them while granted: keep them, with both their items.
 *
 * Context: before the card registers, or with the card's controls held.
 */
static int macaudio_spk_take_hpfs(struct macaudio_snd_data *ma)
{
	struct device *dev = ma->card.dev;
	struct snd_kcontrol *kctl;
	struct snd_ctl_elem_value *uval;
	unsigned int i, nr = 0;
	int safe, ret;

	if (!ma->cfg->tweeter_hpf_granted)
		return 0;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list)
		if (snd_soc_control_matches(kctl, MACAUDIO_TWEETER_HPF_NAME))
			nr++;
	if (!nr)
		return -ENOENT;

	ma->spk_hpfs = devm_kcalloc(dev, nr, sizeof(*ma->spk_hpfs), GFP_KERNEL);
	ma->spk_hpf_granted = devm_kcalloc(dev, nr, sizeof(*ma->spk_hpf_granted), GFP_KERNEL);
	if (!ma->spk_hpfs || !ma->spk_hpf_granted)
		return -ENOMEM;

	uval = kzalloc_obj(*uval);
	if (!uval)
		return -ENOMEM;

	list_for_each_entry(kctl, &ma->card.snd_card->controls, list) {
		if (!snd_soc_control_matches(kctl, MACAUDIO_TWEETER_HPF_NAME))
			continue;
		safe = macaudio_enum_item(kctl, MACAUDIO_TWEETER_HPF_SAFE);
		if (safe < 0) {
			ret = safe;
			goto out;
		}
		/* Capture before fixup installs the floor: the codec loaded the DT. */
		memset(uval, 0, sizeof(*uval));
		uval->id = kctl->id;
		ret = kctl->get(kctl, uval);
		if (ret < 0)
			goto out;
		if (ma->spk_nr_hpfs && safe != ma->spk_hpf_safe) {
			ret = -EINVAL;
			goto out;
		}
		ma->spk_hpf_safe = safe;
		ma->spk_hpf_granted[ma->spk_nr_hpfs] = uval->value.enumerated.item[0];
		for (i = 0; i < kctl->count; i++)
			kctl->vd[i].access &= ~SNDRV_CTL_ELEM_ACCESS_WRITE;
		ma->spk_hpfs[ma->spk_nr_hpfs++] = kctl;
	}
	ret = 0;
out:
	kfree(uval);
	return ret;
}

static int macaudio_copy_link(struct device *dev, struct snd_soc_dai_link *target,
			       struct snd_soc_dai_link *source)
{
	memcpy(target, source, sizeof(struct snd_soc_dai_link));

	target->cpus = devm_kmemdup(dev, target->cpus,
				sizeof(*target->cpus) * target->num_cpus,
				GFP_KERNEL);
	target->codecs = devm_kmemdup(dev, target->codecs,
				sizeof(*target->codecs) * target->num_codecs,
				GFP_KERNEL);
	target->platforms = devm_kmemdup(dev, target->platforms,
				sizeof(*target->platforms) * target->num_platforms,
				GFP_KERNEL);

	if (!target->cpus || !target->codecs || !target->platforms)
		return -ENOMEM;

	return 0;
}

/* A Speaker DT link alone does not establish a protected backend. */
static bool macaudio_speakers_allowed(const struct macaudio_platform_cfg *cfg)
{
	/* Every admitted backend must have a configured, enforced safe level. */
	return cfg->enable_speakers && cfg->safe_vol < 0 &&
		cfg->speakers != SPKR_NONE &&
		(cfg->amp == AMP_TAS5770 || cfg->amp == AMP_SN012776 ||
		 cfg->amp == AMP_MAX98360A);
}

/*
 * An SN012776's operating level: the second byte of its apple,speaker-config,
 * the amp's row of its group's speaker-config in the Mac's ADT, which the
 * boot loader copies to the amp's node. Only its low five bits are the level.
 */
#define SN012776_ROW_LEN	4
#define SN012776_ROW_LEVEL	1
#define SN012776_LEVEL_MASK	0x1f
/* The highest level the amp's "Amp Gain Volume" takes */
#define SN012776_LEVEL_MAX	0x14

static int macaudio_sn012776_level(struct device_node *amp)
{
	u8 row[SN012776_ROW_LEN];
	int level;

	if (of_property_count_u8_elems(amp, "apple,speaker-config") != SN012776_ROW_LEN ||
	    of_property_read_u8_array(amp, "apple,speaker-config", row, SN012776_ROW_LEN))
		return -ENOENT;

	level = row[SN012776_ROW_LEVEL] & SN012776_LEVEL_MASK;
	return level <= SN012776_LEVEL_MAX ? level : -ERANGE;
}

/*
 * Speakers on SN012776 amps run only when every amp has its level from the
 * Mac; a boot loader that does not pass it leaves them off.
 */
static bool macaudio_sn012776_levels_present(struct device *dev,
					     struct device_node *codec)
{
	struct of_phandle_args args;
	int i, n, level;

	n = of_count_phandle_with_args(codec, "sound-dai", "#sound-dai-cells");
	for (i = 0; i < n; i++) {
		if (of_parse_phandle_with_args(codec, "sound-dai",
					       "#sound-dai-cells", i, &args))
			return false;
		level = macaudio_sn012776_level(args.np);
		if (level < 0)
			dev_err(dev, "%pOF: %s amp level from the boot loader\n", args.np,
				level == -ENOENT ? "no" : "out-of-range");
		of_node_put(args.np);
		if (level < 0)
			return false;
	}
	return n > 0;
}

/*
 * Every SN012776's "Amp Gain Volume" ends at its own level; the lease fixes
 * it there (macaudio_spk_fix_amp_gains()).
 */
static int macaudio_limit_sn012776_levels(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	int level, ret;

	for_each_card_components(card, component) {
		struct device_node *np = component->dev->of_node;

		if (!np || !of_device_is_compatible(np, "ti,sn012776"))
			continue;
		level = macaudio_sn012776_level(np);
		if (level < 0)
			return dev_err_probe(card->dev, level, "%pOF: no amp level\n", np);
		snprintf(name, sizeof(name), "%s%sAmp Gain Volume",
			 component->name_prefix ?: "",
			 component->name_prefix ? " " : "");
		ret = snd_soc_limit_volume(card, name, level);
		if (ret != 1)
			return dev_err_probe(card->dev, ret < 0 ? ret : -EINVAL,
					     "limit on '%s': %d\n", name, ret);
	}
	return 0;
}

static int macaudio_parse_of_component(struct device_node *node, int index,
				struct snd_soc_dai_link_component *comp)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(node, "sound-dai", "#sound-dai-cells",
						index, &args);
	if (ret)
		return ret;
	comp->of_node = args.np;
	return snd_soc_get_dai_name(&args, &comp->dai_name);
}

/*
 * Parse one DPCM backend from the devicetree. This means taking one
 * of the CPU DAIs and combining it with one or more CODEC DAIs.
 */
static int macaudio_parse_of_be_dai_link(struct macaudio_snd_data *ma,
				struct snd_soc_dai_link *link,
				int be_index, int ncodecs_per_be,
				struct device_node *cpu,
				struct device_node *codec)
{
	struct snd_soc_dai_link_component *comp;
	struct device *dev = ma->card.dev;
	int codec_base = be_index * ncodecs_per_be;
	int ret, i;

	link->no_pcm = 1;

	link->dai_fmt = ma->cfg->dai_fmt ?: MACAUDIO_DAI_FMT;

	link->num_codecs = ncodecs_per_be;
	link->codecs = devm_kcalloc(dev, ncodecs_per_be,
				    sizeof(*comp), GFP_KERNEL);
	link->num_cpus = 1;
	link->cpus = devm_kzalloc(dev, sizeof(*comp), GFP_KERNEL);

	if (!link->codecs || !link->cpus)
		return -ENOMEM;

	link->num_platforms = 0;

	for_each_link_codecs(link, i, comp) {
		ret = macaudio_parse_of_component(codec, codec_base + i, comp);
		if (ret)
			return dev_err_probe(ma->card.dev, ret, "parsing CODEC DAI of link '%s' at %pOF\n",
					     link->name, codec);
	}

	ret = macaudio_parse_of_component(cpu, be_index, link->cpus);
	if (ret)
		return dev_err_probe(ma->card.dev, ret, "parsing CPU DAI of link '%s' at %pOF\n",
				     link->name, codec);

	link->name = link->cpus[0].dai_name;

	return 0;
}

static int macaudio_get_codec_idle_props(struct device_node *np,
					 struct ma_codec_idle *idle, int i,
					 int be_index, int ncodecs_per_cpu)
{
	char propname[32];
	int ret, sys_codec;

	/*
	 * Depending on the BE index on the DAI, we need to offset our i
	 */
	sys_codec = i + (be_index * ncodecs_per_cpu);

	snprintf(propname, 32, "dai-tdm-idle-mode-%d", sys_codec);
	ret = of_property_match_string(np, propname, "zero");
	if (!ret) {
		idle->idle_mode = SND_SOC_DAI_TDM_IDLE_ZERO;
	} else {
		ret = of_property_match_string(np, propname, "pulldown");
		if (!ret)
			idle->idle_mode = SND_SOC_DAI_TDM_IDLE_PULLDOWN;
		else
			return ret;
	}

	snprintf(propname, 32, "dai-tdm-slot-tx-idle-mask-%d", sys_codec);
	of_property_read_u32(np, propname, &idle->idle_mask);

	return 0;
}

static int macaudio_parse_of(struct macaudio_snd_data *ma)
{
	struct device_node *codec = NULL;
	struct device_node *cpu = NULL;
	struct device_node *np = NULL;
	struct device_node *platform = NULL;
	struct snd_soc_dai_link *link = NULL;
	struct snd_soc_card *card = &ma->card;
	struct device *dev = card->dev;
	struct macaudio_link_props *link_props;
	int ret, num_links, i;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err_probe(dev, ret, "parsing card name\n");
		return ret;
	}
	/*
	 * Set long_name to prevent snd_soc_set_dmi_name() from setting one from
	 * make believe data u-boot provides in its SMBIOS emulation.
	 */
	card->long_name = card->name;

	/* Populate links, start with the fixed number of FE links */
	num_links = ARRAY_SIZE(macaudio_fe_links);

	/* Now add together the (dynamic) number of BE links */
	for_each_available_child_of_node(dev->of_node, np) {
		int num_cpus;

		cpu = of_get_child_by_name(np, "cpu");
		if (!cpu) {
			ret = dev_err_probe(dev, -EINVAL,
				"missing CPU DAI node at %pOF\n", np);
			goto err_free;
		}

		num_cpus = of_count_phandle_with_args(cpu, "sound-dai",
						"#sound-dai-cells");

		if (num_cpus <= 0) {
			ret = dev_err_probe(card->dev, -EINVAL,
				"missing sound-dai property at %pOF\n", cpu);
			goto err_free;
		}
		of_node_put(cpu);
		cpu = NULL;

		/* Each CPU specified counts as one BE link */
		num_links += num_cpus;
	}

	/* Allocate the DAI link array */
	card->dai_link = devm_kcalloc(dev, num_links, sizeof(*link), GFP_KERNEL);
	ma->link_props = devm_kcalloc(dev, num_links, sizeof(*ma->link_props), GFP_KERNEL);
	if (!card->dai_link || !ma->link_props)
		return -ENOMEM;

	link = card->dai_link;
	link_props = ma->link_props;

	for (i = 0; i < ARRAY_SIZE(macaudio_fe_links); i++) {
		ret = macaudio_copy_link(dev, link, &macaudio_fe_links[i]);
		if (ret)
			goto err_free;

		memcpy(link_props, &macaudio_fe_link_props[i], sizeof(struct macaudio_link_props));
		if (ma->cfg->fe_dai_names[i])
			link->cpus[0].dai_name = ma->cfg->fe_dai_names[i];
		if (ma->cfg->route_only_fes & BIT(i))
			link->ignore = 1;
		if (ma->cfg->dai_fmt && !link_props->is_sense)
			link->dai_fmt = ma->cfg->dai_fmt;
		if (i == 0 && ma->cfg->primary_bclk_ratio)
			link_props->bclk_ratio = ma->cfg->primary_bclk_ratio;
		if (i == 1 && ma->cfg->secondary_bclk_ratio)
			link_props->bclk_ratio = ma->cfg->secondary_bclk_ratio;
		if (ma->cfg->sw_gain_dai)
			link_props->sw_gain = !strcmp(link->cpus[0].dai_name,
						      ma->cfg->sw_gain_dai);
		link++; link_props++;
	}

	for (i = 0; i < num_links; i++)
		card->dai_link[i].id = i;

	/* We might disable the speakers, so count again */
	num_links = ARRAY_SIZE(macaudio_fe_links);

	/* Fill in the BEs */
	for_each_available_child_of_node(dev->of_node, np) {
		const char *link_name;
		bool speakers;
		int be_index, num_codecs, num_bes, ncodecs_per_cpu, nchannels;
		unsigned int left_mask, right_mask;

		ret = of_property_read_string(np, "link-name", &link_name);
		if (ret) {
			dev_err_probe(card->dev, ret, "missing link name\n");
			goto err_free;
		}

		dev_dbg(ma->card.dev, "parsing link '%s'\n", link_name);

		speakers = !strcmp(link_name, "Speaker")
			   || !strcmp(link_name, "Speakers");
		if (speakers) {
			if (!macaudio_speakers_allowed(ma->cfg)) {
				dev_err(card->dev, "driver can't assure safety on this model, disabling speakers\n");
				continue;
			}
			if (ma->cfg->amp == AMP_SN012776) {
				struct device_node *spk_codec = of_get_child_by_name(np, "codec");
				bool levels = spk_codec &&
					macaudio_sn012776_levels_present(card->dev, spk_codec);

				of_node_put(spk_codec);
				if (!levels) {
					dev_err(card->dev, "the boot loader passed no amp levels, disabling speakers\n");
					continue;
				}
			}
			ma->has_speakers = 1;
			if (ma->cfg->amp != AMP_SSM3515 && ma->cfg->safe_vol != 0 &&
			    !ma->cfg->spk_fixed_ceiling)
				ma->has_sense = 1;
		}

		cpu = of_get_child_by_name(np, "cpu");
		codec = of_get_child_by_name(np, "codec");

		if (!codec || !cpu) {
			ret = dev_err_probe(dev, -EINVAL,
				"missing DAI specifications for '%s'\n", link_name);
			goto err_free;
		}

		num_bes = of_count_phandle_with_args(cpu, "sound-dai",
						     "#sound-dai-cells");
		if (num_bes <= 0) {
			ret = dev_err_probe(card->dev, -EINVAL,
				"missing sound-dai property at %pOF\n", cpu);
			goto err_free;
		}

		num_codecs = of_count_phandle_with_args(codec, "sound-dai",
							"#sound-dai-cells");
		if (num_codecs <= 0) {
			ret = dev_err_probe(card->dev, -EINVAL,
				"missing sound-dai property at %pOF\n", codec);
			goto err_free;
		}

		dev_dbg(ma->card.dev, "link '%s': %d CPUs %d CODECs\n",
			link_name, num_bes, num_codecs);

		if (num_codecs % num_bes != 0) {
			ret = dev_err_probe(card->dev, -EINVAL,
				"bad combination of CODEC (%d) and CPU (%d) number at %pOF\n",
				num_codecs, num_bes, np);
			goto err_free;
		}

		/*
		 * Now parse the cpu/codec lists into a number of DPCM backend links.
		 * In each link there will be one DAI from the cpu list paired with
		 * an evenly distributed number of DAIs from the codec list. (As is
		 * the binding semantics.)
		 */
		ncodecs_per_cpu = num_codecs / num_bes;
		nchannels = num_codecs * (speakers ? 1 : 2);

		/* Save the max number of channels on the platform */
		if (nchannels > ma->max_channels)
			ma->max_channels = nchannels;
		if (speakers && ma->cfg->speaker_fe_channels > ma->max_channels)
			ma->max_channels = ma->cfg->speaker_fe_channels;

		/*
		 * If there is a single speaker, assign two channels to it, because
		 * it can do downmix.
		 */
		if (nchannels < 2)
			nchannels = 2;

		left_mask = 0;
		for (i = 0; i < nchannels; i += 2)
			left_mask = left_mask << 2 | 1;
		right_mask = left_mask << 1;

		for (be_index = 0; be_index < num_bes; be_index++) {
			/*
			 * Set initial link name to be overwritten by a BE-specific
			 * name later so that we can use at least use the provisional
			 * name in error messages.
			 */
			link->name = link_name;

			ret = macaudio_parse_of_be_dai_link(ma, link, be_index,
							    ncodecs_per_cpu, cpu, codec);
			if (ret)
				goto err_free;

			link_props->is_speakers = speakers;
			link_props->is_headphones = !speakers;
			if (speakers && ma->cfg->spk_dai_fmt)
				link->dai_fmt = ma->cfg->spk_dai_fmt;

			link_props->codecs = devm_kcalloc(dev, ncodecs_per_cpu,
							  sizeof(struct ma_codec_idle), GFP_KERNEL);
			if (!link_props->codecs)
				return -ENOMEM;
			for (i = 0; i < ncodecs_per_cpu; i++) {
				ret = macaudio_get_codec_idle_props(np, &link_props->codecs[i],
								    i, be_index, ncodecs_per_cpu);

				/* Not every codec's bus keeper will be configured */
				if (ret)
					dev_dbg(card->dev, "Codec %d has no idle props\n", i);
			}

			if (num_bes == 2)
				/* This sound peripheral is split between left and right BE */
				link_props->tdm_mask = be_index ? right_mask : left_mask;
			else
				/* One BE covers all of the peripheral */
				link_props->tdm_mask = left_mask | right_mask;

			/* Steal platform OF reference for use in FE links later */
			platform = link->cpus->of_node;

			link++; link_props++;
		}

		of_node_put(codec);
		of_node_put(cpu);
		cpu = codec = NULL;

		num_links += num_bes;
	}

	for (i = 0; i < ARRAY_SIZE(macaudio_fe_links); i++) {
		if (ma->cfg->fe_cpu_is_platform)
			card->dai_link[i].num_platforms = 0;
		else
			card->dai_link[i].platforms->of_node = platform;

		/*
		 * A stream with more slots than amps: the front-end carries
		 * every slot, while a back-end with a single amp would cap it
		 * at that codec's own channel count (ASoC merges a lone
		 * codec's channels, not several codecs'). The amp takes its
		 * slots from its TDM mask.
		 */
		if (ma->cfg->speaker_fe_channels &&
		    !strcmp(card->dai_link[i].name, "Secondary"))
			card->dai_link[i].dpcm_merged_chan = 0;
	}

	/* Skip the speaker sense PCM link if this amp has no sense (or no speakers) */
	if (!ma->has_sense) {
		for (i = 0; i < ARRAY_SIZE(macaudio_fe_links); i++) {
			if (ma->link_props[i].is_sense) {
				memmove(&card->dai_link[i], &card->dai_link[i + 1],
					(num_links - i - 1) * sizeof (struct snd_soc_dai_link));
				num_links--;
				break;
			}
		}
	}

	card->num_links = num_links;

	return 0;

err_free:
	of_node_put(codec);
	of_node_put(cpu);
	of_node_put(np);

	if (!card->dai_link)
		return ret;

	for (i = 0; i < num_links; i++) {
		/*
		 * TODO: If we don't go through this path are the references
		 * freed inside ASoC?
		 */
		snd_soc_of_put_dai_link_codecs(&card->dai_link[i]);
		snd_soc_of_put_dai_link_cpus(&card->dai_link[i]);
	}

	return ret;
}

static int macaudio_get_runtime_bclk_ratio(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dpcm *dpcm;

	/*
	 * If this is a FE, look it up in link_props directly.
	 * If this is a BE, look it up in the respective FE.
	 */
	if (!rtd->dai_link->no_pcm)
		return ma->link_props[rtd->dai_link->id].bclk_ratio;

	for_each_dpcm_fe(rtd, substream->stream, dpcm) {
		int fe_id = dpcm->fe->dai_link->id;

		return ma->link_props[fe_id].bclk_ratio;
	}

	return 0;
}

static int macaudio_dpcm_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	int bclk_ratio = macaudio_get_runtime_bclk_ratio(substream);
	int i;

	if (props->is_sense) {
		rate->min = rate->max = cpu_dai->symmetric_rate;
		return 0;
	}

	/* Speakers BE */
	if (props->is_speakers) {
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			/* Sense PCM: keep the existing BE rate (0 if not already running) */
			rate->min = rate->max = cpu_dai->symmetric_rate;

			return 0;
		} else {
			/*
			 * Set the sense PCM rate control to inform userspace of the
			 * new sample rate.
			 */
			ma->speaker_sample_rate = params_rate(params);
			snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &ma->speaker_sample_rate_kctl->id);
		}
	}

	if (bclk_ratio) {
		struct snd_soc_dai *dai;
		int mclk = params_rate(params) * bclk_ratio;

		for_each_rtd_codec_dais(rtd, i, dai) {
			snd_soc_dai_set_sysclk(dai, 0, mclk, SND_SOC_CLOCK_IN);
			snd_soc_dai_set_bclk_ratio(dai, bclk_ratio);
		}

		snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
		snd_soc_dai_set_bclk_ratio(cpu_dai, bclk_ratio);
	}

	return 0;
}

static int macaudio_fe_startup(struct snd_pcm_substream *substream)
{

	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	int max_rate, ret;

	if (props->is_sense) {
		/*
		 * Sense stream will not return data while playback is inactive,
		 * so do not time out.
		 */
		substream->wait_time = MAX_SCHEDULE_TIMEOUT;
		return 0;
	}

	ret = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_CHANNELS,
					   0, ma->max_channels);
	if (ret < 0)
		return ret;

	max_rate = MACAUDIO_MAX_BCLK_FREQ / props->bclk_ratio;
	ret = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_RATE,
					   0, max_rate);
	if (ret < 0)
		return ret;

	return 0;
}

static int macaudio_fe_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_dpcm *dpcm;

	be = NULL;
	for_each_dpcm_be(rtd, substream->stream, dpcm) {
		be = dpcm->be;
		break;
	}

	if (!be) {
		dev_err(rtd->dev, "opening PCM device '%s' with no audio route configured by the user\n",
				rtd->dai_link->name);
		return -EINVAL;
	}

	return macaudio_dpcm_hw_params(substream, params);
}


static void macaudio_dpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *dai;
	int bclk_ratio = macaudio_get_runtime_bclk_ratio(substream);
	int i;

	if (bclk_ratio) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_dai_set_sysclk(dai, 0, 0, SND_SOC_CLOCK_IN);

		snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
	}
}

static int macaudio_be_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	int i;

	if (props->is_speakers && substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * Clear the DAI rates, so the next open can change the sample rate.
		 * This won't happen automatically if the sense PCM is open.
		 */
		for_each_rtd_dais(rtd, i, dai) {
			dai->symmetric_rate = 0;
		}

		/* Notify userspace that the speakers are closed */
		ma->speaker_sample_rate = 0;
		snd_ctl_notify(ma->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &ma->speaker_sample_rate_kctl->id);
	}

	return 0;
}

static bool macaudio_trigger_starts(int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return true;
	default:
		return false;
	}
}

/*
 * The speakers start only while protection is granted (macaudio_spk_put()).
 * DPCM triggers a back-end only for the first front-end that starts on it,
 * so every front-end that drives the speakers is checked as well, before
 * its DMA and its back-ends (see macaudio_fe_links): one that would join a
 * speaker back-end left running across a withdrawal is refused too.
 */
static bool macaudio_fe_refuses(struct macaudio_snd_data *ma, struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_substream *substream, int cmd)
{
	if (!ma->has_speakers || substream->stream != SNDRV_PCM_STREAM_PLAYBACK ||
	    !macaudio_trigger_starts(cmd) || !macaudio_spk_leased(ma))
		return false;
	return macaudio_fe_drives_speakers(ma, rtd, substream->stream) &&
	       !macaudio_spk_admitted(ma);
}

/* A refused speaker start: counted (debugfs speaker_starts_refused), never logged */
static int macaudio_spk_refuse(struct macaudio_snd_data *ma)
{
	atomic_inc(&ma->spk_refused);
	atomic_inc(&ma->spk_refused_since_grant);
	return -EPERM;
}

/*
 * A refused start is the floor doing its job, not a failure, and an audio
 * server retries it from its data loop: dozens to hundreds of times per
 * withdrawal. The links are marked quiet_trigger_eperm (see
 * macaudio_quiet_refusals()), so ASoC fails these triggers without logging
 * them; each refusal is counted instead, and the next grant reports the run.
 */
static int macaudio_fe_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    READ_ONCE(ma->speaker_lock_failed))
		return -EIO;

	if (!macaudio_sw_gain_pcm(substream))
		return 0;

	if (atomic_read(&ma->sw_gain_limit_failed))
		return -EIO;
	/* The CPU DAI's prepare clears the old samples out of the buffer next */
	atomic_set(&ma->sw_gain_prepared_epoch,
		   atomic_read(&ma->sw_gain_lock_epoch));
	return 0;
}

/*
 * Runs before the CPU DAI and the back-ends are triggered (see the
 * front-end links). While leased, a speaker start without a grant is
 * refused. Under the volume lock, a lock event since prepare means the
 * buffer holds samples converted at the old gain, so the start is refused
 * and the stream has to be prepared again.
 */
static int macaudio_fe_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);

	if (macaudio_fe_refuses(ma, rtd, substream, cmd))
		return macaudio_spk_refuse(ma);

	/*
	 * A prepare can pass just before a failed lock is recorded and the
	 * stop work then find the stream not yet prepared: refuse its start.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    READ_ONCE(ma->speaker_lock_failed) && macaudio_trigger_starts(cmd))
		return -EIO;

	if (!macaudio_sw_gain_pcm(substream) || !macaudio_trigger_starts(cmd))
		return 0;

	if (atomic_read(&ma->sw_gain_limit_failed) ||
	    atomic_read(&ma->sw_gain_prepared_epoch) !=
			atomic_read(&ma->sw_gain_lock_epoch))
		return -EPIPE;
	return 0;
}

/* The gate's -EPERM on every link of a card with speakers is a quiet refusal */
static void macaudio_quiet_refusals(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link)
		link->quiet_trigger_eperm = 1;
}

/*
 * The back-end's link trigger runs before its DAIs: a refused start starts
 * nothing. A grant withdrawn after this check stops the stream through
 * macaudio_stop_speaker_playback(), under the same stream lock.
 */
static int macaudio_be_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];

	if (!props->is_speakers || substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (macaudio_spk_leased(ma) && !macaudio_spk_admitted(ma))
			return macaudio_spk_refuse(ma);
		ma->bes_active |= BIT(rtd->dai_link->id);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		ma->bes_active &= ~BIT(rtd->dai_link->id);
		break;
	default:
		return -EINVAL;
	}

	/* The volume lock's lease runs only while the speakers play */
	schedule_work(&ma->lock_update_work);
	return 0;
}

static const struct snd_soc_ops macaudio_fe_ops = {
	.startup	= macaudio_fe_startup,
	.shutdown	= macaudio_dpcm_shutdown,
	.hw_params	= macaudio_fe_hw_params,
	.prepare	= macaudio_fe_prepare,
	.trigger	= macaudio_fe_trigger,
};

static const struct snd_soc_ops macaudio_be_ops = {
	.hw_free	= macaudio_be_hw_free,
	.shutdown	= macaudio_dpcm_shutdown,
	.hw_params	= macaudio_dpcm_hw_params,
	.trigger	= macaudio_be_trigger,
};

static int macaudio_be_assign_tdm(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	unsigned int mask;
	int nslots, ret, i;

	if (!props->tdm_mask)
		return 0;

	mask = props->tdm_mask;
	nslots = __fls(mask) + 1;

	if (rtd->dai_link->num_codecs == 1) {
		ret = snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_codec(rtd, 0), mask,
					       0, nslots, MACAUDIO_SLOTWIDTH);

		/*
		 * Headphones get a pass on -ENOTSUPP (see the comment
		 * around bclk_ratio value for primary FE), as does an
		 * amplifier without a TDM interface behind a software gain.
		 */
		if (ret == -ENOTSUPP &&
		    (props->is_headphones || ma->cfg->amp == AMP_MAX98360A))
			return 0;

		return ret;
	}

	for_each_rtd_codec_dais(rtd, i, dai) {
		int slot = __ffs(mask);

		mask &= ~(1 << slot);
		ret = snd_soc_dai_set_tdm_slot(dai, 1 << slot, 0, nslots,
					       MACAUDIO_SLOTWIDTH);
		if (ret)
			return ret;

		if (props->codecs[i].idle_mode) {
			ret = snd_soc_dai_set_tdm_idle(dai, props->codecs[i].idle_mask,
						       0, props->codecs[i].idle_mode, 0);
		}
	}

	return 0;
}

static int macaudio_be_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	int i, ret;

	ret = macaudio_be_assign_tdm(rtd);
	if (ret < 0)
		return ret;

	if (props->is_headphones) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_component_set_jack(dai->component, &ma->jack, NULL);
	}

	return 0;
}

static void macaudio_be_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	int i;

	if (props->is_headphones) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_component_set_jack(dai->component, NULL, NULL);
	}
}

static int macaudio_fe_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	int nslots = props->bclk_ratio / MACAUDIO_SLOTWIDTH;

	if (props->is_sense)
		return snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_cpu(rtd, 0), 0, 0xffff, 16, 16);

	return snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_cpu(rtd, 0), (1 << nslots) - 1,
					(1 << nslots) - 1, nslots, MACAUDIO_SLOTWIDTH);
}

static const struct snd_soc_jack_pin macaudio_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int macaudio_speaker_id_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = BIT(ma->speaker_id_gpios->ndescs) - 1;
	uinfo->value.integer.step = 1;

	return 0;
}

static int macaudio_speaker_id_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	unsigned int value = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < ma->speaker_id_gpios->ndescs; i++) {
		ret = gpiod_get_value_cansleep(ma->speaker_id_gpios->desc[i]);
		if (ret < 0)
			return ret;
		if (ret)
			value |= BIT(i);
	}
	uvalue->value.integer.value[0] = value;

	return 0;
}

static const struct snd_kcontrol_new macaudio_speaker_id_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Speaker Variant ID",
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.info = macaudio_speaker_id_info,
	.get = macaudio_speaker_id_get,
};

static int macaudio_probe(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	int ret;

	dev_dbg(card->dev, "%s!\n", __func__);

	BUILD_BUG_ON(sizeof(ma->jack_pins) != sizeof(macaudio_jack_pins));
	memcpy(ma->jack_pins, macaudio_jack_pins, sizeof(ma->jack_pins));

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack",
			SND_JACK_HEADSET | SND_JACK_HEADPHONE |
			SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			SND_JACK_BTN_2 | SND_JACK_BTN_3,
			&ma->jack, ma->jack_pins,
			ARRAY_SIZE(ma->jack_pins));
	if (ret < 0) {
		dev_err(card->dev, "jack creation failed: %d\n", ret);
		return ret;
	}

	/*
	 * Apple remotes use BTN_0..2; CS42L83's analogue fallback can also
	 * report BTN_3 for the fourth button of other headsets.
	 */
	snd_jack_set_key(ma->jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(ma->jack.jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	snd_jack_set_key(ma->jack.jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
	snd_jack_set_key(ma->jack.jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);

	return ret;
}

static int macaudio_add_backend_dai_route(struct snd_soc_card *card, struct snd_soc_dai *dai,
					  bool is_speakers)
{
	struct snd_soc_dapm_route routes[2];
	struct snd_soc_dapm_route *r;
	int nroutes = 0;
	int ret;

	memset(routes, 0, sizeof(routes));

	dev_dbg(card->dev, "adding routes for '%s'\n", dai->name);

	r = &routes[nroutes++];
	if (is_speakers)
		r->source = "Speaker Playback";
	else
		r->source = "Headphone Playback";
	r->sink = dai->stream[SNDRV_PCM_STREAM_PLAYBACK].widget->name;

	/* If headphone jack, add capture path */
	if (!is_speakers) {
		r = &routes[nroutes++];
		r->source = dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget->name;
		r->sink = "Headset Capture";
	}

	/* If speakers, add sense capture path (a playback-only CPU DAI has none) */
	if (is_speakers && dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget) {
		r = &routes[nroutes++];
		r->source = dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget->name;
		r->sink = "Speaker Sense Capture";
	}

	ret = snd_soc_dapm_add_routes(card->dapm, routes, nroutes);
	if (ret)
		dev_err(card->dev, "failed adding dynamic DAPM routes for %s\n",
			dai->name);
	return ret;
}

static int macaudio_add_pin_routes(struct snd_soc_card *card, struct snd_soc_component *component,
				   bool is_speakers)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct snd_soc_dapm_route routes[2];
	struct snd_soc_dapm_route *r;
	int nroutes = 0;
	char buf[32];
	int ret;

	memset(routes, 0, sizeof(routes));

	/* Connect the far ends of CODECs to pins */
	if (is_speakers) {
		const char *out = ma->cfg->spk_out_widget ?: "OUT";

		r = &routes[nroutes++];
		r->source = out;
		if (component->name_prefix) {
			snprintf(buf, sizeof(buf) - 1, "%s %s", component->name_prefix, out);
			r->source = buf;
		}
		r->sink = "Speaker";
	} else {
		r = &routes[nroutes++];
		r->source = "Jack HP";
		r->sink = "Headphone";
		r = &routes[nroutes++];
		r->source = "Headset Mic";
		r->sink = "Jack HS";
	}

	ret = snd_soc_dapm_add_routes(card->dapm, routes, nroutes);
	if (ret)
		dev_err(card->dev, "failed adding dynamic DAPM routes for %s\n",
			component->name);
	return ret;
}

static int macaudio_late_probe(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *dai;
	int ret, i;

	/* Add the dynamic DAPM routes */
	for_each_card_rtds(card, rtd) {
		struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];

		if (!rtd->dai_link->no_pcm)
			continue;

		for_each_rtd_cpu_dais(rtd, i, dai) {
			ret = macaudio_add_backend_dai_route(card, dai, props->is_speakers);

			if (ret)
				return ret;
		}

		for_each_rtd_codec_dais(rtd, i, dai) {
			ret = macaudio_add_pin_routes(card, dai->component,
						      props->is_speakers);

			if (ret)
				return ret;
		}
	}

	/*
	 * The volume lock's floor holds from here on, before userspace sees the
	 * card: "Speaker Volume Unlock" is in charge at every bind.
	 */
	if (ma->has_speakers) {
		ma->speaker_sample_rate_kctl = snd_soc_card_get_kcontrol(card,
									 "Speaker Sample Rate");
		ma->speaker_lock_kctl = snd_soc_card_get_kcontrol(card,
								  "Speaker Volume Unlock");
		ma->spk_kctl = snd_soc_card_get_kcontrol(card, "Speaker Protection");
		ma->spk_iface_kctl = snd_soc_card_get_kcontrol(card,
							       "Speaker Protection Interface");
		if (!ma->speaker_sample_rate_kctl || !ma->speaker_lock_kctl ||
		    !ma->spk_kctl || !ma->spk_iface_kctl)
			return -EINVAL;
		ret = macaudio_spk_take_volumes(ma);
		if (ret)
			return dev_err_probe(card->dev, ret, "speaker volumes missing\n");
		/* Save codec-loaded DT corners before the floor and PM notifier. */
		ret = macaudio_spk_take_hpfs(ma);
		if (ret)
			return dev_err_probe(card->dev, ret,
					     "tweeter high-pass controls missing\n");
		ret = macaudio_spk_take_amp_gains(ma);
		if (ret)
			return ret;
		for_each_card_rtds(card, rtd) {
			if (!rtd->dai_link->no_pcm &&
			    ma->link_props[rtd->dai_link->id].sw_gain)
				ma->sw_gain_component = snd_soc_rtd_to_cpu(rtd, 0)->component;
		}
		/* The volume table's top under the lock: the legacy ceiling */
		ret = macaudio_sw_gain_full_scale(ma, false);
		if (ret < 0)
			return dev_err_probe(card->dev, ret, "software gain ceiling not set\n");

		mutex_lock(&ma->volume_lock_mutex);
		macaudio_vlimit_unlock(ma, false);
		ret = READ_ONCE(ma->speaker_lock_failed) ? -EIO : 0;
		mutex_unlock(&ma->volume_lock_mutex);
		if (ret)
			return dev_err_probe(card->dev, ret, "speaker volume lock not set\n");
#ifdef CONFIG_DEBUG_FS
		debugfs_create_atomic_t("speaker_starts_refused", 0444, card->debugfs_card_root,
					&ma->spk_refused);
#endif
	}

	/* Append identity metadata without renumbering the existing controls. */
	if (ma->speaker_id_gpios) {
		ret = snd_soc_add_card_controls(card, &macaudio_speaker_id_control, 1);
		if (ret)
			return ret;
	}

	/* After it, for the same reason */
	if (ma->has_speakers) {
		ret = snd_soc_add_card_controls(card, &macaudio_spk_xover_control, 1);
		if (ret)
			return ret;
		ma->spk_xover_kctl = snd_soc_card_get_kcontrol(card, MACAUDIO_SPK_XOVER_NAME);
		if (!ma->spk_xover_kctl)
			return -EINVAL;
	}

	/* The notifier walks the card's controls: only while the card is bound */
	return register_pm_notifier(&ma->pm_nb);
}

static int macaudio_card_remove(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	unregister_pm_notifier(&ma->pm_nb);
	/*
	 * A control file closed from here on still unlocks "Speaker Protection",
	 * while the codecs are going: nothing may touch them any more.
	 */
	mutex_lock(&ma->spk_mutex);
	ma->spk_dead = true;
	WRITE_ONCE(ma->spk_granted, false);
	mutex_unlock(&ma->spk_mutex);
	/*
	 * Nothing queues a withdrawal's work any more but a lease timer that
	 * is firing: stop it first. The works walk the card's runtimes and
	 * controls: finish them while those still exist.
	 */
	hrtimer_cancel(&ma->spk_lease_timer);
	kthread_cancel_work_sync(&ma->spk_expire_work);
	kthread_cancel_work_sync(&ma->spk_stop_work);
	cancel_delayed_work_sync(&ma->spk_unowned_work);
	/*
	 * No back-end can trigger any more, but a lock update it queued takes
	 * the card's controls_rwsem: finish it while the snd_card still exists.
	 */
	cancel_work_sync(&ma->lock_update_work);
	cancel_delayed_work_sync(&ma->lock_timeout_work);
	mutex_lock(&ma->volume_lock_mutex);
	macaudio_pm_volumes_drop(ma);
	mutex_unlock(&ma->volume_lock_mutex);
	return 0;
}

/* Userspace may read the controls matching @name but never write them. */
static void macaudio_deny_write(struct snd_soc_card *card, const char *name)
{
	struct snd_kcontrol *kctl;
	unsigned int i;

	list_for_each_entry(kctl, &card->snd_card->controls, list) {
		if (!snd_soc_control_matches(kctl, name))
			continue;
		for (i = 0; i < kctl->count; i++)
			kctl->vd[i].access &= ~SNDRV_CTL_ELEM_ACCESS_WRITE;
	}
}

/* Called during control fixup, before the card is exposed to userspace. */
static int macaudio_set_fixed_enum(struct snd_soc_card *card, const char *name,
				   const char *value)
{
	int ret;

	/* NULL: keep the value the control has */
	if (value) {
		ret = snd_soc_set_enum_kctl(card, name, value);
		if (ret < 0)
			return ret;
	}

	ret = snd_soc_deactivate_kctl(card, name, 0);
	if (ret < 0)
		return ret;

	/* INACTIVE only hides controls; direct ALSA writes require WRITE. */
	macaudio_deny_write(card, name);
	return ret;
}

#define CHECK(call, pattern, value, min)                                       \
	{                                                                      \
		int ret = call(card, pattern, value);                          \
		int err = (ret >= 0 && ret < min) ? -ERANGE : ret;             \
		if (err < 0) {                                                 \
			dev_err(card->dev, "%s on '%s': %d\n", #call, pattern, \
				ret);                                          \
			return err;                                    \
		} else {                                                       \
			dev_dbg(card->dev, "%s on '%s': %d hits\n", #call,     \
				pattern, ret);                                 \
		}                                                              \
	}

#define CHECK_CONCAT(call, suffix, value) \
	{ \
		snprintf(buf, sizeof(buf), "%s%s", prefix, suffix); \
		CHECK(call, buf, value, 1); \
	}

static int macaudio_set_speaker(struct snd_soc_card *card, const char *prefix, bool tweeter)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	char buf[256];

	if (!ma->has_speakers)
		return 0;

	switch (ma->cfg->amp) {
	case AMP_TAS5770:
		if (ma->cfg->stereo)
			CHECK_CONCAT(macaudio_set_fixed_enum, "ASI1 Sel", "Left");

		CHECK_CONCAT(snd_soc_limit_volume, "Amp Gain Volume", ma->cfg->amp_gain);
		break;
	case AMP_SN012776:
		if (ma->cfg->stereo)
			CHECK_CONCAT(macaudio_set_fixed_enum, "ASI1 Sel", "Left");

		/*
		 * Each amp's gain ends at its level from the Mac's ADT
		 * (macaudio_limit_sn012776_levels()). The tweeters' playback
		 * high-pass starts at the protection floor; the woofers keep
		 * the corner the amps were given from the ADT.
		 */
		CHECK_CONCAT(macaudio_set_fixed_enum, "HPF Corner Frequency",
			     tweeter ? MACAUDIO_TWEETER_HPF_SAFE : NULL);

		CHECK_CONCAT(macaudio_set_fixed_enum, "OCE Handling", "Retry");
		break;
	case AMP_SSM3515:
		/* TODO: check */
		CHECK_CONCAT(macaudio_set_fixed_enum, "DAC Analog Gain Select", "8.4 V Span");

		/* TODO: HPF, needs new call to set */
		break;
	case AMP_MAX98360A:
		/* No controls of its own; the gain sits on the CPU DAI */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int macaudio_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	const char *p;

	/* Set the card ID early to avoid races with udev */
	p = strrchr(card->name, ' ');
	if (p) {
		snprintf(card->snd_card->id, sizeof(card->snd_card->id),
			 "Apple%s", p + 1);
	}

	/*
	 * Before the card registers, so userspace never sees the codec's full
	 * range: a CS42L84 jack stops at the DAC's 0 dB. Above it the DAC only
	 * adds digital gain, which clips a full-scale signal, and macOS's own
	 * headphone volume ends at 0 dB on every machine (the "Built-in
	 * Headphone" OutputVolumeMax of the machine's audio configuration).
	 * The same for every board; a card without a CS42L84 jack (the
	 * speaker-only cards) has no such control.
	 */
	if (snd_soc_card_get_kcontrol(card, "Jack DAC Playback Volume"))
		CHECK(snd_soc_limit_volume, "Jack DAC Playback Volume",
		      CS42L84_DAC_0DB, 1);

	if (!ma->has_speakers)
		return 0;

	macaudio_protect_sense(card);

	if (ma->cfg->amp == AMP_SN012776) {
		int ret = macaudio_limit_sn012776_levels(card);

		if (ret)
			return ret;
	}

	if (ma->cfg->spk_secondary_only)
		CHECK(snd_soc_set_enum_kctl, "Speaker Playback Mux", "Secondary", 1);

	/*
	 * This needs some care to avoid matches against cs42l84's
	 * "Jack HPF Corner Frequency".
	 */
	switch(ma->cfg->speakers) {
	case SPKR_NONE:
		return -EINVAL;
	case SPKR_1W:
		/* only 1W stereo system (J313) is uses cs42l83 */
		if (ma->cfg->stereo) {
			CHECK(macaudio_set_speaker, "* ", false, 0);
		} else {
			CHECK(macaudio_set_speaker, "", false, 0);
		}
		break;
	case SPKR_2W:
		CHECK(macaudio_set_speaker, "* Front ", false, 0);
		CHECK(macaudio_set_speaker, "* Rear ", false, 0);
		break;
	case SPKR_1W1T:
		CHECK(macaudio_set_speaker, "* Tweeter ", true, 0);
		CHECK(macaudio_set_speaker, "* Woofer ", false, 0);
		break;
	case SPKR_2W1T:
		CHECK(macaudio_set_speaker, "* Tweeter ", true, 0);
		CHECK(macaudio_set_speaker, "* Woofer 1 ", false, 0);
		CHECK(macaudio_set_speaker, "* Woofer 2 ", false, 0);
		break;
	}

	return 0;
}

static const char * const macaudio_spk_mux_texts[] = {
	"Primary",
	"Secondary"
};

SOC_ENUM_SINGLE_VIRT_DECL(macaudio_spk_mux_enum, macaudio_spk_mux_texts);

static const struct snd_kcontrol_new macaudio_spk_mux =
	SOC_DAPM_ENUM("Speaker Playback Mux", macaudio_spk_mux_enum);

static const char * const macaudio_hp_mux_texts[] = {
	"Primary",
	"Secondary"
};

SOC_ENUM_SINGLE_VIRT_DECL(macaudio_hp_mux_enum, macaudio_hp_mux_texts);

static const struct snd_kcontrol_new macaudio_hp_mux =
	SOC_DAPM_ENUM("Headphones Playback Mux", macaudio_hp_mux_enum);

static const struct snd_soc_dapm_widget macaudio_snd_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SPK("Speaker (Static)", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

	SND_SOC_DAPM_MUX("Speaker Playback Mux", SND_SOC_NOPM, 0, 0, &macaudio_spk_mux),
	SND_SOC_DAPM_MUX("Headphone Playback Mux", SND_SOC_NOPM, 0, 0, &macaudio_hp_mux),

	SND_SOC_DAPM_AIF_OUT("Speaker Playback", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Headphone Playback", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("Headset Capture", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("Speaker Sense Capture", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static int macaudio_sss_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;

	return 0;
}

static int macaudio_sss_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	/*
	 * TODO: Check if any locking is in order here. I would
	 * assume there is some ALSA-level lock, but DAPM implementations
	 * of kcontrol ops do explicit locking, so look into it.
	 */
	uvalue->value.integer.value[0] = ma->speaker_sample_rate;

	return 0;
}

static int macaudio_slk_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = INT_MIN;
	uinfo->value.integer.max = INT_MAX;

	return 0;
}

static int macaudio_slk_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	long val = uvalue->value.integer.value[0];

	/*
	 * Reads return the lock state (0 or 1), so a saved mixer state holds
	 * one of those. Writing it back changes nothing: only the lock owner's
	 * magic value renews the lease. Accept it as a no-op so that restoring
	 * a saved state before the daemon has taken the lock is not an error.
	 */
	if (val == 0 || val == 1)
		return 0;

	if (val != SPEAKER_MAGIC_VALUE)
		return -EINVAL;

	mutex_lock(&ma->volume_lock_mutex);

	if (!ma->speaker_lock_owner) {
		mutex_unlock(&ma->volume_lock_mutex);
		return -EPERM;
	}

	/* "Speaker Protection" owns the speakers: no unlock to renew */
	if (macaudio_spk_leased(ma)) {
		mutex_unlock(&ma->volume_lock_mutex);
		return -EBUSY;
	}

	/* Serves as a notification that the lock was lost at some point */
	if (ma->speaker_volume_was_locked) {
		ma->speaker_volume_was_locked = false;
		mutex_unlock(&ma->volume_lock_mutex);
		return -ETIMEDOUT;
	}

	/* Between suspend-prepare and the freeze: the lease starts again after the resume */
	if (ma->pm_quiesced) {
		mutex_unlock(&ma->volume_lock_mutex);
		return 0;
	}

	cancel_delayed_work(&ma->lock_timeout_work);

	ma->speaker_lock_remain = ms_to_ktime(SPEAKER_LOCK_TIMEOUT);
	ma->speaker_lock_timeout = ktime_add(ktime_get(), ma->speaker_lock_remain);
	macaudio_vlimit_update(ma);

	if (ma->speaker_lock_timeout_enabled) {
		dev_dbg(ma->card.dev, "Volume limit timeout ping: %ld us left\n",
			(long)ktime_to_us(ma->speaker_lock_remain));
		schedule_delayed_work(&ma->lock_timeout_work, usecs_to_jiffies(ktime_to_us(ma->speaker_lock_remain)));
	}

	mutex_unlock(&ma->volume_lock_mutex);

	return 0;
}

static int macaudio_slk_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	uvalue->value.integer.value[0] = ma->speaker_volume_unlocked ? 1 : 0;

	return 0;
}

static int macaudio_slk_lock(struct snd_kcontrol *kcontrol, struct snd_ctl_file *owner)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	mutex_lock(&ma->volume_lock_mutex);

	/*
	 * A lease belongs to the owner that pinged for it. Whatever the
	 * previous owner had left must not unlock the volume for a new one
	 * before it has pinged itself.
	 */
	cancel_delayed_work(&ma->lock_timeout_work);
	ma->speaker_lock_remain = 0;
	ma->speaker_lock_timeout = 0;
	/* A new owner sets the volumes itself */
	macaudio_pm_volumes_drop(ma);

	ma->speaker_lock_owner = owner;
	macaudio_vlimit_update(ma);

	/*
	 * Reset the unintended lock flag when the control is first locked.
	 * At this point the state is locked and cannot be unlocked until
	 * userspace writes to this control, so this cannot spuriously become
	 * true again until that point.
	 */
	ma->speaker_volume_was_locked = false;

	mutex_unlock(&ma->volume_lock_mutex);

	return 0;
}

static void macaudio_slk_unlock(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	mutex_lock(&ma->volume_lock_mutex);
	cancel_delayed_work(&ma->lock_timeout_work);
	ma->speaker_lock_owner = NULL;
	ma->speaker_lock_remain = 0;
	ma->speaker_lock_timeout = 0;
	macaudio_vlimit_update(ma);
	mutex_unlock(&ma->volume_lock_mutex);
}

/* The speaker controls go last: a card without speakers drops them. */
#define MACAUDIO_NUM_SPEAKER_CONTROLS 4

static const struct snd_kcontrol_new macaudio_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.name = "Speaker Sample Rate",
		.info = macaudio_sss_info, .get = macaudio_sss_get,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_WRITE |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.name = "Speaker Volume Unlock",
		.info = macaudio_slk_info,
		.put = macaudio_slk_put, .get = macaudio_slk_get,
		.lock = macaudio_slk_lock, .unlock = macaudio_slk_unlock,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = MACAUDIO_SPK_OWNERSHIP_ACCESS,
		.name = "Speaker Protection",
		.info = macaudio_spk_info,
		.put = macaudio_spk_put, .get = macaudio_spk_get,
		.lock = macaudio_spk_lock, .unlock = macaudio_spk_unlock,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = MACAUDIO_SPK_OWNERSHIP_ACCESS,
		.name = "Speaker Protection Interface",
		.info = macaudio_spk_iface_info,
		.put = macaudio_spk_iface_put, .get = macaudio_spk_iface_get,
	},
};

static const struct snd_soc_dapm_route macaudio_dapm_routes[] = {
	/* Playback paths */
	{ "Speaker Playback Mux", "Primary", "PCM0 TX" },
	{ "Speaker Playback Mux", "Secondary", "PCM1 TX" },
	{ "Speaker Playback", NULL, "Speaker Playback Mux"},

	{ "Headphone Playback Mux", "Primary", "PCM0 TX" },
	{ "Headphone Playback Mux", "Secondary", "PCM1 TX" },
	{ "Headphone Playback", NULL, "Headphone Playback Mux"},
	/*
	 * Additional paths (to specific I2S ports) are added dynamically.
	 */

	/* Capture paths */
	{ "PCM0 RX", NULL, "Headset Capture" },

	/* Sense paths */
	{ "PCM2 RX", NULL, "Speaker Sense Capture" },
};

/*
 *	enable	amp		speakers	stereo	gain	safe_vol
 *
 * An SN012776's gain is not here: each amp's operating level comes from the
 * Mac's ADT, through the amp's apple,speaker-config.
 */
struct macaudio_platform_cfg macaudio_j180_cfg = {
	false,	AMP_SN012776,	SPKR_1W1T,	false,	0,	-20,
};
struct macaudio_platform_cfg macaudio_j274_cfg = {
	true,	AMP_TAS5770,	SPKR_1W,	false,	20,	-20,
};

struct macaudio_platform_cfg macaudio_j293_cfg = {
	true,	AMP_TAS5770,	SPKR_2W,	true,	15,	-20,
};

struct macaudio_platform_cfg macaudio_j313_cfg = {
	true,	AMP_TAS5770,	SPKR_1W,	true,	10,	-20,
};

/*
 * J700 (T8140, MacBook Neo): the AOP firmware owns the audio fabric, so the
 * CPU component is the T8140 AOP audio driver rather than the MCA. The jack
 * is a CS42L83 on the AOP's "cout" back-end, which the firmware runs as a
 * 125-SCLK pulse frame at 48 kHz (6 MHz) with two 32-bit slots, data
 * launched on the rising SCLK edge: DSP_A with an inverted bit clock. The
 * speakers are one MAX98360A per channel on the "spkr" back-end, with no
 * sense lines and no amplifier controls; the volume is the CPU DAI's
 * software gain, which the volume lock holds 20 dB down until a protection
 * daemon takes it, and the lease 20 dB down below the full scale until its
 * engine is granted. The "Speaker Sense" capture is the SpeakerTap, a
 * digital tap of the words the amplifier receives; there is no V/I sense.
 */
struct macaudio_platform_cfg macaudio_j700_cfg = {
	.enable_speakers = true,
	.amp = AMP_MAX98360A,
	.speakers = SPKR_1W,
	.stereo = true,
	.safe_vol = -20,
	.spk_out_widget = "Speaker",
	.sw_gain_dai = "j700-pcm-1",
	.fe_dai_names = { "j700-pcm-0", "j700-pcm-1", "j700-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.primary_bclk_ratio = 125,
};

/*
 * J873 (T8152, Mac mini M6): as on J700, the AOP firmware owns the audio
 * fabric and the CPU component is the T8152 AOP audio driver. The jack is a
 * CS42L84 on the AOP's "cout" back-end, the speaker one SN012776 on "spkr";
 * both serializers run on the firmware's clocks. The speaker frame is 250
 * BCLKs of 12 MHz (the amp is told so by its ti,bclk-ratio), with the data
 * sampled on the rising edge, as macOS programs the amp.
 */
struct macaudio_platform_cfg macaudio_j873_cfg = {
	/*
	 * No qualified speaker protection exists for J873 yet (no verified
	 * sense path, thermal model or power budget), and there is no safe
	 * level: macaudio_speakers_allowed() keeps the speaker off. The jack
	 * is unaffected.
	 */
	.enable_speakers = false,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W,
	.stereo = false,
	.safe_vol = 0,
	.fe_dai_names = { "j873-pcm-0", "j873-pcm-1", "j873-pcm-2" },
	/* jack: a one-BCLK sync pulse, data launched on the rising edge */
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.spk_dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		       SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.spk_secondary_only = true,
};

/*
 * J775c and J775d (T6050, Mac Studio M5 Max and M5 Ultra): as on J714s, the
 * AOP firmware owns the audio fabric and the CPU component is the AOP audio
 * driver. The jack is a CS42L84 on "cout", the speaker one SN012776 on
 * "spkr" in a frame of 250 BCLKs of 12 MHz.
 *
 * The speaker stays off. The ADT's audio-speaker stream lists one TX slot and
 * no RX (sense) slot, so the speaker can only be run under a protection lease
 * held by speakerguardd with a no-sense model, as on the J773g; that model
 * does not exist for these machines yet. Until it does there is no safe
 * level: macaudio_speakers_allowed() keeps the speaker off, as on the J873.
 * The jack is unaffected.
 */
struct macaudio_platform_cfg macaudio_j775_cfg = {
	.enable_speakers = false,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W,
	.stereo = false,
	.safe_vol = 0,
	.fe_dai_names = { "j873-pcm-0", "j873-pcm-1", "j873-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.spk_dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		       SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.spk_secondary_only = true,
};

/*
 * J813 (T8142, MacBook Air M5 13"): as on J873, the AOP firmware owns the
 * audio fabric and the CPU component is the T8152-style AOP audio driver.
 * The jack is a CS42L84 on "cout"; the speakers are four SN012776 (one
 * woofer and one tweeter per side) on "spkr", whose voltage and current
 * sense return on the same stream. As on the other SN012776 laptops, the
 * speakers are behind the speaker protection interfaces from the bind: the
 * volumes stay at the lock's ceiling until a protection daemon
 * (speakersafetyd) holds them, or under "Speaker Protection" at the floor
 * until an owner is granted. Each amp's level is its apple,speaker-config
 * from the boot loader.
 */
struct macaudio_platform_cfg macaudio_j813_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j873-pcm-0", "j873-pcm-1", "j873-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.spk_dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		       SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.spk_secondary_only = true,
};

/*
 * J714s (T6050, MacBook Pro 14" M5 Pro): as on J813, the AOP firmware owns
 * the audio fabric and the CPU component is the T8152-style AOP audio
 * driver. The jack is a CS42L84 on "cout"; the speakers are six SN012776
 * (two woofers and a tweeter per side) on "spkr", in the J616s frame: six
 * 32-bit playback slots and twelve 16-bit sense slots in 250 BCLKs of
 * 12 MHz. Each amp's level is its apple,speaker-config from the boot loader.
 */
struct macaudio_platform_cfg macaudio_j714_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j873-pcm-0", "j873-pcm-1", "j873-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.spk_dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		       SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.spk_secondary_only = true,
	/* Granted, the tweeters restore their DT corner, as on the J616s */
	.tweeter_hpf_granted = true,
};

/*
 * J815 (T8142, MacBook Air M5 15"): J813's AOP-fed card with six SN012776,
 * two woofers and a tweeter per side, in ADT speaker-config order (left
 * woofer 1, right woofer 1, left tweeter, right tweeter, left woofer 2,
 * right woofer 2: left in even slots, right in odd), and twelve V/I sense
 * words on the same stream. Locked to the safe level until speakersafetyd
 * holds the volume, as on J813. Each amp's level is its apple,speaker-config
 * from the boot loader (15 on all six in the J815 ADT).
 */
struct macaudio_platform_cfg macaudio_j815_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j873-pcm-0", "j873-pcm-1", "j873-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.spk_dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		       SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.spk_secondary_only = true,
};

struct macaudio_platform_cfg macaudio_j314_cfg = {
	true,	AMP_SN012776,	SPKR_2W1T,	true,	0,	-20,
};

/* Granted, the tweeters restore their DT corner (see tweeter_hpf_granted) */
struct macaudio_platform_cfg macaudio_j316_cfg = {
	true,	AMP_SN012776,	SPKR_2W1T,	true,	0,	-20,
	.tweeter_hpf_granted = true,
};

struct macaudio_platform_cfg macaudio_j37x_j47x_cfg = {
	true,	AMP_SN012776,	SPKR_1W,	false,	0,	-20,
};

/* Granted, the tweeters restore their DT corner (see tweeter_hpf_granted) */
struct macaudio_platform_cfg macaudio_j413_cfg = {
	true,	AMP_SN012776,	SPKR_1W1T,	true,	0,	-20,
	.tweeter_hpf_granted = true,
};

struct macaudio_platform_cfg macaudio_j415_cfg = {
	true,	AMP_SN012776,	SPKR_2W1T,	true,	0,	-20,
};

struct macaudio_platform_cfg macaudio_j45x_cfg = {
	false,	AMP_SSM3515,	SPKR_1W1T,	true,	9,	-20, /* TODO: gain?? */
};

struct macaudio_platform_cfg macaudio_j493_cfg = {
	true,	AMP_SN012776,	SPKR_2W,	true,	0,	-20,
};

/*
 * J614s and J616s (T6040, MacBook Pro 14" and 16" M4 Pro): the AOP firmware
 * runs MCA cluster 0, so the CPU component is the AOP audio driver
 * (front-ends j616-pcm-N, back-end spkr). One serializer carries all six
 * SN012776 amps in a 250-bit frame at 12 MHz: six 32-bit playback slots,
 * twelve 16-bit sense slots. The amps take I2S with an inverted frame clock
 * and a normal bit clock, as on the J713 (T8132: the same amp and frame).
 * Apple's device trees for the two boards describe the same amps, slots and
 * sense layout; only the speakers and their tuning differ.
 */
struct macaudio_platform_cfg macaudio_j616_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	/* No headphone codec on this card: the jack is a card of its own. */
	.route_only_fes = BIT(0),
	/* Granted, the tweeters restore their DT corner */
	.tweeter_hpf_granted = true,
};

/*
 * J614s: the same amps and frame as the J616s. Its speakers are bound like
 * the J616s's; playback still waits for a protection lease (see
 * macaudio_spk_admitted()), which speakerguardd v2 takes only once it has
 * compiled and self-checked this Mac's own tuning (AID34). The legacy
 * protection has no J614 profile, so without v2 the amps stay at the floor.
 */
struct macaudio_platform_cfg macaudio_j614_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	.route_only_fes = BIT(0),
	/* Granted, the tweeters restore their DT corner, as on the J616s */
	.tweeter_hpf_granted = true,
};

/*
 * J604 (T8132, MacBook Pro 14" M4): the J614s's six amps, two woofers and a
 * tweeter per side, on all six playback and twelve sense slots of the same
 * frame. The speakers stay at their safe level until speakersafetyd takes
 * them with the J604 profile.
 */
struct macaudio_platform_cfg macaudio_j604_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	.route_only_fes = BIT(0),
};

/*
 * J575c (Mac Studio, M4 Max): the J616s stream, six 32-bit playback and
 * twelve 16-bit sense slots in the same 250-bit frame (the firmware runs it
 * so, although the ADT's audio-speaker lists one playback slot), with one
 * SN012776 on the first two playback slots (it downmixes them) and the
 * first two sense slots (current, then voltage), which speakersafetyd's
 * J575 model reads. The front-end carries all six channels; the last four
 * are silent.
 */
struct macaudio_platform_cfg macaudio_j575c_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W,
	.stereo = false,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	/* No headphone codec on this card: the jack is a card of its own. */
	.route_only_fes = BIT(0),
	.speaker_fe_channels = 6,
};

/*
 * J773 (Mac mini, M4 Pro): one SN012776 on the AOP's speaker stream. The
 * speaker has not been brought up and has no protection profile, so keep it
 * off as on the J614s: the speaker link is not bound.
 */
struct macaudio_platform_cfg macaudio_j575c_j773_cfg = {
	.enable_speakers = false,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W,
	.stereo = false,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.fe_cpu_is_platform = true,
	.route_only_fes = BIT(0),
};

/*
 * J713 (T8132, MacBook Air 13" M4): the J616s stream, six 32-bit playback
 * and twelve 16-bit sense slots in the same 250-bit frame, with four
 * SN012776 amps, a woofer and a tweeter per side, on the first four and the
 * first eight. The front-end carries all six channels; the last two are
 * silent.
 */
struct macaudio_platform_cfg macaudio_j713_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_1W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	/* No headphone codec on this card: the jack is a card of its own. */
	.route_only_fes = BIT(0),
	.speaker_fe_channels = 6,
};

/*
 * J715 (T8132, MacBook Air 15" M4): the same stream with six SN012776 amps,
 * two woofers and a tweeter per side, on all six playback and twelve sense
 * slots. The speakers stay at their safe level as on the J713.
 */
struct macaudio_platform_cfg macaudio_j715_cfg = {
	.enable_speakers = true,
	.amp = AMP_SN012776,
	.speakers = SPKR_2W1T,
	.stereo = true,
	.safe_vol = -20,
	.fe_dai_names = { "j616-pcm-0", "j616-pcm-1", "j616-pcm-2" },
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
		   SND_SOC_DAIFMT_GATED | SND_SOC_DAIFMT_NB_IF,
	.secondary_bclk_ratio = 250,
	.fe_cpu_is_platform = true,
	/* No headphone codec on this card: the jack is a card of its own. */
	.route_only_fes = BIT(0),
	.speaker_fe_channels = 6,
};

/*
 * The J614s and J616s headphone jack, a card of its own: a CS42L84 on the
 * AOP's "jack" back-end, which the firmware runs on MCA cluster 2 as a 64-SCLK
 * frame at 48 kHz (3.072 MHz, the primary front-end's ratio) with a
 * one-SCLK FSYNC pulse, two 32-bit slots and data launched on the rising
 * SCLK edge: DSP_A with an inverted bit clock, as on the J700. A card binds
 * only once all of its codecs have probed; apart from the speaker card, a
 * codec that never answers costs the jack only. The second front-end only
 * satisfies the fixed routes: the AOP component refuses to open it.
 */
struct macaudio_platform_cfg macaudio_j616_jack_cfg = {
	.fe_dai_names = { "j616-jack-pcm-0", "j616-jack-pcm-1" },
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_IB_NF,
	.fe_cpu_is_platform = true,
	.route_only_fes = BIT(1),
};

struct macaudio_platform_cfg macaudio_fallback_cfg = {
	false,	AMP_NONE,	SPKR_NONE,	false,	0,	0,
};

/*
 * DT compatible/ID table rules:
 *
 * 1. Machines with **identical** speaker configurations (amps, models, chassis)
 *    are allowed to declare compatibility with the first model (chronologically),
 *    and are not enumerated in this array.
 *
 * 2. Machines with identical amps and speakers (=identical speaker protection
 *    rules) but a different chassis must use different compatibles, but may share
 *    the private data structure here. They are explicitly enumerated.
 *
 * 3. Machines with different amps or speaker layouts must use separate
 *    data structures.
 *
 * 4. Machines with identical speaker layouts and amps (but possibly different
 *    speaker models/chassis) may share the data structure, since only userspace
 *    cares about that (assuming our general -20dB safe level standard holds).
 */
static const struct of_device_id macaudio_snd_device_id[]  = {
	/* Model   ID      Amp         Gain    Speakers */
	/* j180    AID19   sn012776    10      1× 1W+1T */
	{ .compatible = "apple,j180-macaudio", .data = &macaudio_j180_cfg },
	/* j274    AID6    tas5770     20      1× 1W */
	{ .compatible = "apple,j274-macaudio", .data = &macaudio_j274_cfg },
	/* j293    AID3    tas5770     15      2× 2W */
	{ .compatible = "apple,j293-macaudio", .data = &macaudio_j293_cfg },
	/* j313    AID4    tas5770     10      2× 1W */
	{ .compatible = "apple,j313-macaudio", .data = &macaudio_j313_cfg },
	/* j314    AID8    sn012776    15      2× 2W+1T */
	{ .compatible = "apple,j314-macaudio", .data = &macaudio_j314_cfg },
	/* j316    AID9    sn012776    15      2× 2W+1T */
	{ .compatible = "apple,j316-macaudio", .data = &macaudio_j316_cfg },
	/* j375    AID10   sn012776    15      1× 1W */
	{ .compatible = "apple,j375-macaudio", .data = &macaudio_j37x_j47x_cfg },
	/* j413    AID13   sn012776    15      2× 1W+1T */
	{ .compatible = "apple,j413-macaudio", .data = &macaudio_j413_cfg },
	/* j414    AID14   sn012776    15      2× 2W+1T Compat: apple,j314-macaudio */
	/* j415    AID27   sn012776    15      2× 2W+1T */
	{ .compatible = "apple,j415-macaudio", .data = &macaudio_j415_cfg },
	/* j416    AID15   sn012776    15      2× 2W+1T Compat: apple,j316-macaudio */
	/* j456    AID5    ssm3515     15      2× 1W+1T */
	{ .compatible = "apple,j456-macaudio", .data = &macaudio_j45x_cfg },
	/* j457    AID7    ssm3515     15      2× 1W+1T Compat: apple,j456-macaudio */
	/* j473    AID12   sn012776    20      1× 1W */
	{ .compatible = "apple,j473-macaudio", .data = &macaudio_j37x_j47x_cfg },
	/* j474    AID26   sn012776    20      1× 1W    Compat: apple,j473-macaudio */
	/* j475    AID25   sn012776    20      1× 1W    Compat: apple,j375-macaudio */
	/* j493    AID18   sn012776    15      2× 2W */
	{ .compatible = "apple,j493-macaudio", .data = &macaudio_j493_cfg },
	/* j504    AID24   sn012776    15      2× 2W+1T Compat: apple,j314-macaudio */
	/* j514    AID28   sn012776    15      2x 2W+1T Compat: apple,j314-macaudio */
	/* j516    AID29   sn012776    15      2x 2W+1T Compat: apple,j316-macaudio */
	/* j575    AID33   sn012776    15      1x 1W    Compat: apple,j375-macaudio */
	/* j575c   -       sn012776    -       1x 1W    (AOP-fed) */
	{ .compatible = "apple,j575c-macaudio", .data = &macaudio_j575c_cfg },
	/* j575c   -       cs42l84     -       the jack, a card of its own (AOP-fed) */
	{ .compatible = "apple,j575c-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j613    AID20   sn012776    15      2x 1W+1T Compat: apple,j413-macaudio */
	/* j615    AID21   sn012776    15      2x 2W+1T Compat: apple,j415-macaudio */
	/* j604    -       sn012776    15      2x 2W+1T (AOP-fed) */
	{ .compatible = "apple,j604-macaudio", .data = &macaudio_j604_cfg },
	/* j604    -       cs42l84     -       the jack, a card of its own (AOP-fed) */
	{ .compatible = "apple,j604-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j614    AID34   sn012776    15      2x 2W+1T (AOP-fed; speakers off until bring-up) */
	{ .compatible = "apple,j614-macaudio", .data = &macaudio_j614_cfg },
	/* j614    AID34   cs42l84     -       the jack, a card of its own (AOP-fed) */
	{ .compatible = "apple,j614-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j616    AID35   sn012776    15      2x 2W+1T (AOP-fed, one serializer) */
	{ .compatible = "apple,j616-macaudio", .data = &macaudio_j616_cfg },
	/* j616    AID35   cs42l84     -       the jack, a card of its own (AOP-fed) */
	{ .compatible = "apple,j616-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j623    -       cs42l84     -       the jack only (AOP-fed) */
	{ .compatible = "apple,j623-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j624    -       cs42l84     -       the jack only (AOP-fed) */
	{ .compatible = "apple,j624-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j700    AID44   max98360a   -       2x 1W    (AOP-fed, software gain) */
	{ .compatible = "apple,j700-macaudio", .data = &macaudio_j700_cfg },
	/* j714    -       sn012776    15      2x 2W+1T (AOP-fed, V/I sense) */
	{ .compatible = "apple,j714-macaudio", .data = &macaudio_j714_cfg },
	/* k116    -       sn012776    15      2x 2W+1T (AOP-fed, V/I sense; the 16" PROLED chassis, J714's amps and frame) */
	{ .compatible = "apple,k116-macaudio", .data = &macaudio_j714_cfg },
	/* j713    -       sn012776    15      2x 1W+1T (AOP-fed, one serializer) */
	{ .compatible = "apple,j713-macaudio", .data = &macaudio_j713_cfg },
	/* The J713 jack runs on the AOP like the J616s one. */
	{ .compatible = "apple,j713-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j715    -       sn012776    15      2x 2W+1T (AOP-fed, one serializer) */
	{ .compatible = "apple,j715-macaudio", .data = &macaudio_j715_cfg },
	{ .compatible = "apple,j715-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j773    -       sn012776    15      1x 1W    (AOP-fed; speaker off until bring-up) */
	{ .compatible = "apple,j773-macaudio", .data = &macaudio_j575c_j773_cfg },
	/* j773    -       cs42l84     -       the jack, a card of its own (AOP-fed) */
	{ .compatible = "apple,j773-macaudio-jack", .data = &macaudio_j616_jack_cfg },
	/* j873    AID48   sn012776    15      1x 1W    (AOP-fed, no protection) */
	{ .compatible = "apple,j873-macaudio", .data = &macaudio_j873_cfg },
	/* j775c   -       sn012776    -       1x 1W    (AOP-fed, no sense: speaker off) */
	{ .compatible = "apple,j775c-macaudio", .data = &macaudio_j775_cfg },
	/* j775d   -       sn012776    -       1x 1W    (AOP-fed, no sense: speaker off) */
	{ .compatible = "apple,j775d-macaudio", .data = &macaudio_j775_cfg },
	/* j813    AID?    sn012776    15      2x 1W+1T (AOP-fed, V/I sense) */
	{ .compatible = "apple,j813-macaudio", .data = &macaudio_j813_cfg },
	/* j815    AID43   sn012776    15      2x 2W+1T (AOP-fed, V/I sense) */
	{ .compatible = "apple,j815-macaudio", .data = &macaudio_j815_cfg },
	/* Fallback, jack only */
	{ .compatible = "apple,macaudio"},
	{ }
};
MODULE_DEVICE_TABLE(of, macaudio_snd_device_id);

static int macaudio_snd_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct macaudio_snd_data *data;
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	const struct of_device_id *of_id;
	int ret;
	int i;

	of_id = of_match_device(macaudio_snd_device_id, dev);
	if (!of_id)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	card = &data->card;
	snd_soc_card_set_drvdata(card, data);
	dev_set_drvdata(&pdev->dev, data);
	mutex_init(&data->spk_mutex);
	mutex_init(&data->volume_lock_mutex);

	card->owner = THIS_MODULE;
	card->driver_name = "macaudio";
	card->dev = dev;
	card->dapm_widgets = macaudio_snd_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(macaudio_snd_widgets);
	card->dapm_routes = macaudio_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(macaudio_dapm_routes);
	card->controls = macaudio_controls;
	card->num_controls = ARRAY_SIZE(macaudio_controls);
	card->probe = macaudio_probe;
	card->late_probe = macaudio_late_probe;
	card->remove = macaudio_card_remove;
	card->component_chaining = true;
	card->fully_routed = true;

	if (of_id->data)
		data->cfg = of_id->data;
	else
		data->cfg = &macaudio_fallback_cfg;

	card->fixup_controls = macaudio_fixup_controls;

	ret = macaudio_parse_of(data);
	if (ret)
		return ret;

	data->speaker_id_gpios = devm_gpiod_get_array_optional(dev, "speaker-id",
							       GPIOD_IN);
	if (IS_ERR(data->speaker_id_gpios))
		return dev_err_probe(dev, PTR_ERR(data->speaker_id_gpios),
				     "Failed to get speaker ID inputs\n");
	if (data->speaker_id_gpios &&
	    (!data->speaker_id_gpios->ndescs ||
	     data->speaker_id_gpios->ndescs > MACAUDIO_MAX_SPEAKER_ID_GPIOS))
		return dev_err_probe(dev, -EINVAL, "Invalid speaker ID input count\n");

	/*
	 * Without sense the PCM2 link is gone, and a card whose front-ends
	 * are not the MCA's (the J616 jack) has no PCM2 RX widget at all.
	 * The sense route is the table's last entry.
	 */
	if (!data->has_sense)
		card->num_dapm_routes--;

	/* Admitted speakers always have a safe level (macaudio_speakers_allowed()) */
	if (!data->has_speakers) {
		card->num_controls -= MACAUDIO_NUM_SPEAKER_CONTROLS;
	} else {
		data->has_safety = true;
		macaudio_quiet_refusals(card);
	}

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->ops = &macaudio_be_ops;
			link->init = macaudio_be_init;
			link->exit = macaudio_be_exit;
		} else {
			link->ops = &macaudio_fe_ops;
			link->init = macaudio_fe_init;
		}
	}

	INIT_WORK(&data->lock_update_work, macaudio_vlimit_update_work);
	INIT_DELAYED_WORK(&data->lock_timeout_work, macaudio_vlimit_timeout_work);
	INIT_WORK(&data->sw_gain_stop_work, macaudio_sw_gain_stop_work);
	INIT_WORK(&data->lock_fail_work, macaudio_lock_fail_work);
	atomic_set(&data->sw_gain_lock_epoch, 0);
	atomic_set(&data->sw_gain_prepared_epoch, -1);
	atomic_set(&data->sw_gain_limit_failed, 0);
	data->spk_iface = MACAUDIO_SPK_IFACE_UNLOCK;

	ret = macaudio_spk_init(data);
	if (ret)
		return dev_err_probe(dev, ret, "speaker protection workers not started\n");
	ret = devm_add_action_or_reset(dev, macaudio_spk_fini, data);
	if (ret)
		return ret;

	return snd_soc_register_card(card);
}

static void macaudio_snd_platform_remove(struct platform_device *pdev)
{
	struct macaudio_snd_data *ma = dev_get_drvdata(&pdev->dev);

	/* The card's removal stops the lease timer and its works; devm frees them */
	snd_soc_unregister_card(&ma->card);
	cancel_work_sync(&ma->lock_update_work);
	cancel_delayed_work_sync(&ma->lock_timeout_work);
	cancel_work_sync(&ma->sw_gain_stop_work);
	cancel_work_sync(&ma->lock_fail_work);
	cancel_delayed_work_sync(&ma->spk_unowned_work);
}

static struct platform_driver macaudio_snd_driver = {
	.probe = macaudio_snd_platform_probe,
	.remove = macaudio_snd_platform_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = macaudio_snd_device_id,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(macaudio_snd_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple Silicon Macs machine-level sound driver");
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_KUNIT) && !defined(MODULE)
#include "macaudio-admission-test.h"
#include "macaudio-protection-test.h"
#include "macaudio-hpf-test.h"
#include "macaudio-pm-volumes-test.h"
#include "macaudio-iface-test.h"
#if IS_ENABLED(CONFIG_OF_OVERLAY)
#include "macaudio-amp-level-test.h"
#endif
#endif
