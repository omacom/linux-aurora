# M3 Pro GPU performance profile

The experimental T6030 profile pairs native Asahi OpenGL with the legacy Borealis Vulkan driver. The current vkQuake demo1 result is approximately 230–232 FPS at an actual 1920×1080 render size, with the original game executable and rendering settings. M1 Air remains faster, approximately 300–340 FPS with run variation. This profile is not a general GPU conformance claim or an M3 firmware migration.

## Kernel and driver selection

The kernel candidate is `ac34611f8de50386b6c08872c07f857fdbc24cf2`, built as `7.1.12-2-20261012.932-sep-ARCH`. Mixed render/compute batches wait for the preceding engine's firmware stamp while retaining resources through the final completion checks. The opt-in profile selects mixed batching, render and compute batch limits of four, CPU preparation overlap and a 25 µs completion wait. Effective pipeline depth remains one. Consistent performance requests and early tiling remain disabled.

The native OpenGL prefix stays `/opt/mesa-m3/native-pro`. The Vulkan prefix is `/opt/mesa-m3/perf-r8-3c74c2b3130f-r1`, with ICD `private-icd.json`. Vulkan source is `3c74c2b3130f31cfdf57455097c96966a2487eee`. Both prefixes require the legacy T6030 identity: G15S revision 0x11, one die, two clusters, USC3 and HAL0. They must not select a HAL200 driver on a 14.x installation.

Set `ASAHI_M3_EXPERIMENTAL=1`, `AGX_MESA_DEBUG=nopromote` and `VK_LOADER_LAYERS_DISABLE=~implicit~`. Enable only `HK_M3_COMPRESSED_RGBA16F`, `HK_M3_COMPRESSED_R8` and `HK_M3_COMPRESSED_SPLIT_DEPTH` for this profile. General compressed depth, broad compressed 2D, native/memory preambles, fragment scheduling, texture-index experiments, tile cleanup and native input attachments remain disabled. Keep native GL libraries first in the desktop's library search path and select Vulkan through its own ICD.

## Attachment compression

RGBA16F single-sample attachment compression raised the controlled timedemo from approximately 176 to 225 FPS. Narrow R8 attachment compression increased repeated medians from 223.95 to 230.7 FPS, about 3%. The profile excludes unsupported dimensions, layers, samples, transfers, mutable formats and storage images. In particular, the game's RGB10 image with STORAGE usage stays uncompressed.

Preserve the game's gamma 0.9, contrast 1.4, transparency resolve and compute water rendering. Reducing image quality or replacing the timedemo is not a performance improvement.

## Artifact identity

| Artifact | SHA256 |
| --- | --- |
| `linux-aurora-7.1.12.aurora2-20261012.932-aarch64.pkg.tar.zst` | `91b6e4b35281dee4b3e33f767923d2460f206173c11dcea63f56daac1bd37b51` |
| `linux-aurora-headers-7.1.12.aurora2-20261012.932-aarch64.pkg.tar.zst` | `0ca01c976bd04bb65b51fae27c09ca35550c0cfe384e5340b4f65d4124de6220` |
| `mesa-m3-perf-r8-color-26.1.4.perfr8.20261012-1-aarch64.pkg.tar.zst` | `b8b216b7d8c3d29ac232c1ec1e67424b1fe6f01a35634c525ec8b35039fdf45c` |
| Vulkan library | `087c89edb739a232987bccf09a0335336a8439ce5c6f4dfae56aad3d5734ff07` |

These are private experimental artifacts, not assets of the public latest release. Preserve a verified recovery kernel and bootloader backup during opt-in installation.

## Remaining limits

Fullscreen image reads and writes remain expensive relative to M1; shader exponent math and post-idle fence signaling do not explain the remaining gap. Physical memory-fabric clock state and firmware controller handling still need qualification. Concurrency above depth one needs correct resource ownership and synchronization. Intermittent AOP `-62` timeouts remain a separate qualification issue.

M1/M2 keep their existing driver paths. M3 Air exact 26.6.2/HAL200 is a distinct native OpenGL-only profile, with no Vulkan-25 claim. The AGX3 runtime for G17P/T8140/HAL200 does not admit this G15S Pro.
