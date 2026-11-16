#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <linux/types.h>

struct device;
struct platform_device;
struct dcp_sound_cookie;

int dcp_audiosrv_prepare(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_startlink(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_stoplink(struct device *dev);
int dcp_audiosrv_unprepare(struct device *dev);
int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize);
int dcp_audiosrv_get_product_attrs(struct device *dev, void *attrs, size_t maxsize);

void dcpaud_connect(struct platform_device *pdev, bool connected);
void dcpaud_disconnect(struct platform_device *pdev);

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/* The native link methods of the DCP firmware 14.7 external role. */
enum dcp_audio_link {
	DCP_AUDIO_LINK_PREPARE,
	DCP_AUDIO_LINK_START,
	DCP_AUDIO_LINK_STOP,
	DCP_AUDIO_LINK_COMPLETE,
};

bool dcp_audiosrv_native_link(struct device *dev);
int dcp_audiosrv_open(struct device *dev);
void dcp_audiosrv_close(struct device *dev);
int dcp_audiosrv_link(struct device *dev, enum dcp_audio_link op,
		      const struct dcp_sound_cookie *cookie);
/* @gen: only a link prepared on that instance, still the open one (NULL: any) */
void dcpaud_release_link(struct platform_device *pdev, const u64 *gen);
/* Whether the link prepared last is on instance @gen, still the open one */
bool dcp_audiosrv_link_on(struct device *dev, u64 gen);
/* The prepared link still belongs to the client of the published instance. */
bool dcp_audiosrv_link_live(struct device *dev);
#endif

#endif /* __AUDIO_H__ */
