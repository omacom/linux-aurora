/* SPDX-License-Identifier: ISC */
#ifndef BRCMF_SLEEP_ROAM_H
#define BRCMF_SLEEP_ROAM_H

/*
 * BCM4378/3 on J313 (PCI 14e4:4425 rev 03, firmware 18.20.383, loaded as
 * brcmfmac4378b1-pcie) answers assoc_info with a firmware error while a
 * roam or a resume is still committing the association. fwil reports that
 * as -EBADE (-52). The same windows drop in-flight flowring frames without
 * a txstatus, so waiting for one only stalls the event worker.
 *
 * BRCMF_BCM4378_CHIP_ID matches BRCM_CC_4378_CHIP_ID in brcm_hw_ids.h.
 * Callers assert that.
 */

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/errno.h>
#else
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t u32;
#define EBADE 52
#endif

#define BRCMF_BCM4378_CHIP_ID		0x4378

/* fwil maps every negative firmware status onto this errno. */
#define BRCMF_ASSOC_FW_ERROR		(-EBADE)

/*
 * The first assoc_info query is immediate. Each -EBADE after a connect
 * or roam success is retried. Seven 25 ms sleeps add at most 175 ms.
 * The 769 ms figure on this machine is when the one-shot query ran
 * after s2idle exit, not a measured firmware delay. Any other error
 * is returned without another query.
 */
#define BRCMF_ASSOC_INFO_ATTEMPTS	8
#define BRCMF_ASSOC_INFO_RETRY_MS	25

static inline bool brcmf_is_bcm4378(u32 chip)
{
	return chip == BRCMF_BCM4378_CHIP_ID;
}

/* Other chips keep the historical txstatus wait. */
static inline bool brcmf_flowring_wait_txstatus(u32 chip)
{
	return !brcmf_is_bcm4378(chip);
}

/*
 * @failures is how many assoc_info queries have already failed.
 * Returns true when another query is worth issuing.
 */
static inline bool
brcmf_assoc_info_should_retry(u32 chip, int err, unsigned int failures,
			      bool link_expected)
{
	if (!brcmf_is_bcm4378(chip))
		return false;
	if (!link_expected)
		return false;
	if (err != BRCMF_ASSOC_FW_ERROR)
		return false;
	return failures + 1 < BRCMF_ASSOC_INFO_ATTEMPTS;
}

#endif /* BRCMF_SLEEP_ROAM_H */
