# Release 19 package recipes

**Kernel** (`kernel/`): the release 18 recipe and config, built from `5ec1028c81526e1ed31db2b97e47edf8663e83cb`. Its prepared config is identical to release 18's. The T6030 performance marker names this source.

**Native OpenGL for the M3 Pro** (`native-pro/`): source `d616d0650d6fa80f72a750a0121442ab917ed6cf` and its source archive are unchanged from release 18. Package release 9 changes only `mesa-m3-pro-check`, which now admits the release 16, 18 and 19 kernels by their exact markers. It then requires the exact T6030 runtime identity, as before.

This remains experimental native OpenGL support, without a full conformance claim.

**Vulkan for the M3 Pro** (`pro-vulkan/`): the separate Borealis package keeps the release 16 binaries and compression settings.

**Other Macs:** M1/M2, the Air HAL200 profiles and the Neo keep their existing selection.
