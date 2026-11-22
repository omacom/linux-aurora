# Release 18 package recipes

**Kernel** (`kernel/`): the release 16 recipe and config, built from `6f63677751854d57bcfac172314957bc2197c05f`. The only kernel change since release 16 is the M3 Max (T6031) groundwork, which is off by default. The T6030 performance marker names this source.

**Native OpenGL for the M3 Pro** (`native-pro/`): source `d616d0650d6fa80f72a750a0121442ab917ed6cf`, package release 8.
- **Indexed draws:** draws with an index offset are corrected (#101).
- **Multisampling:**
  - 2x and 4x color attachments resolve to single-sample images with matching image addressing.
  - Persistent renderbuffers keep their individual samples across flushes and repeated resolves.
  - Two RGBA8 attachments keep their samples through background reloads and end-of-tile stores.
  - Masked partial clears preserve the uncleared attachment and pixels.
- **Glyph correction:** unchanged from release 17.
- **Session check:** `mesa-m3-pro-check` admits only the release 16 and release 18 kernels, by their exact markers. It then requires the exact T6030 runtime identity, as before.

This remains experimental native OpenGL support, without a full conformance claim.

**Vulkan for the M3 Pro** (`pro-vulkan/`): the separate Borealis package keeps the release 16 binaries and compression settings.

**Other Macs:** M1/M2, the Air HAL200 profiles and the Neo keep their existing selection.
