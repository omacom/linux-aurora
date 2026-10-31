.. SPDX-License-Identifier: GPL-2.0-only

J616s ISP modes and capture protocol
==================================

Sources and scope
-----------------

The source reference is Ryan's m1n1 PR 20, branch ``j616s`` at
``9be380c8a5a94609fdbe62d693c7b2121478ad8f``:

* ``proxyclient/m1n1/fw/isp/isp_sensor.py`` discovers IMX958 and queries
  ``CH_CAMERA_CONFIG_GET`` for every configuration reported by ``CH_INFO_GET``.
* ``isp_capture.py:T6040PreviewConfiguration`` contains the successful
  processed capture command sequence. ``isp_pixels.py`` decodes its P010
  output as video-range BT.709.
* ``docs/j616s-bringup.md`` records all 13 configuration queries, completed
  1280x720 P010 captures (including an 80-frame ring-wrap trial) and a later
  single completed 1920x1080 P010 frame. Intermittent startup/streaming stalls
  remain unresolved. Neither the requested AE limits nor the host capture
  rate is a measured sensor FPS.

Both processed output sizes use firmware configuration **5**, whose reported
input dimensions are 1920x2160. The branch does not establish that this input
height is an independently deliverable 2160-row colour image, nor that config
5 is the highest-resolution configuration the sensor offers. It contains no
completed colour result from another configuration. RAW trials produced
stable pixels after stop but no completed RAW report; combined RAW/YUV was
refused at the allocation limit. No sensor full-resolution claim follows.

Linux PR 231, branch ``j616s/linux-integration-next`` at
``a4e7ee86bc5eed8ff9dab61ef9656adfe000b207``, has the corresponding
``isp-profile.[ch]`` and IPC/buffer preparation. Its T6040 profile is pinned
to **25G76** and remains gated. It is a protocol reference, not an admission
profile for the current firmware.

This driver change builds on the working T6040 H17 implementation in Linux
PR 384, now in ``integration/aurora``. That PR reports 26.6.x firmware capture
at 1920x1080 NV12 and use of config-5 1080p/720p presets. P010 and additional
firmware-derived output modes still need capture validation on 26.6.2.
Only our source and synthetic protocol tests are included; no Apple binary,
setfile contents, calibration table or firmware image is added.

Sensor tuning and noise reduction
---------------------------------

NV12 setup loads model-wide sensor tuning using ``CH_SET_FILE_LOAD``
(0x0111). T8140/IMX558 selects ``apple/isp_1925_03XX.dat``; T6040/IMX958
selects ``apple/isp_2224_01XX.dat``. The T6040 lens-shading fixup is applied
only to the copied firmware buffer. A missing file allows streaming with a
warning. An invalid file, interrupted read or failed firmware upload fails
stream setup instead of silently continuing. A debug message identifies a
successful upload without dumping its contents.
This confirms transport completion only: the NV12 path does not decode the
setfile reply fields, so it does not prove that every coefficient was applied.

The NV12 command sequence already sends ``CH_CNR_START`` (0x0a2f),
``CH_MBNR_ENABLE`` (0x0a3a, use case 0, mode 1, chroma enabled),
``APPLE_CH_TEMPORAL_FILTER_START`` (0xc100),
``APPLE_CH_MOTION_HISTORY_START`` (0xc102) and
``APPLE_CH_TEMPORAL_FILTER_ENABLE`` (0xc113). The temporal-start record has
one fixed argument of 1 and another supplied by ``apple,temporal-filter``
(default 0; J700 supplies 1). The latter's meaning is not established here;
it must not be treated as a boolean enable or an output-pool count.
Command submission alone does not establish equivalence to a host-side
motion-compensated filter or measure the firmware's resulting noise reduction.

The P010 preview still uses its qualified minimal sequence, which skips
these explicit NR commands and setfile upload. An earlier NV12 stream may
have left tuning in the resident firmware. Compare cold-boot P010 with
NV12-to-P010 transitions when assessing image quality.

Sensor setfiles are distinct from individual camera-module factory
calibration (iCCl). The driver has no iCCl parser, serial binding or upload
command. The installer preserves raw factory records in the local fallback
archive; this does not mean the ISP consumes them. No factory record or
vendor payload belongs in the kernel source or a distributed firmware package.

The separate ANE video denoiser is not implemented. The board parameter facts
are auto mode 2, total-gain entry/exit thresholds 2304/1792 on J700 and
3584/2816 on J616s, an FPS threshold of 30 and a 30-frame ramp-down. The FPS
comparison direction, gain units and ANE/ISP interchange need a confirmed
interface before implementation.

Public source audit
-------------------

The pinned PR and its discussion were checked again for the RAW layout and
per-configuration timing. The reusable code establishes the processed path;
it does not contain a RAW Bayer packing/CFA contract or the 13 reply records:

* ``isp_capture.py`` accepts only 1280x720 and 1920x1080 processed geometry.
  Its generic pool-3 builder describes a single address, not the pixel packing,
  CFA order or RAW receiver command sequence. The qualified preview uses pool 9.
* ``isp_pixels.py`` contains only an MSB-aligned P010 decoder, with no Bayer decoder.
* ``isp_sensor.py`` reads the sensor ID/count and queries an index at runtime.
  It has neither saved per-index replies nor a frame-rate decoder.
* ``isp_profile.py`` and ``fixtures/isp_t6040_25g76.json`` describe boot segments,
  allocation geometry and channels. The fixture has no camera-info or config replies.
* ``isp_vid.py`` and ``experiments/isp.py`` use the older fixed 720p NV12 receiver.
  ``isp_cmd.py`` includes legacy sensor-0x248 examples and a temporary 3.75 FPS
  AE request (0x3c0), alongside a comment describing the older 30 FPS request.
  These are not IMX958 per-mode timing records. The qualified T6040 helper instead
  requests 30/15 FPS AE limits; neither set supplies a measured nominal-rate table.
* ``isp_command.py`` implements ownership, command/reply and deadline handling.
  ``src/isp.c`` prepares boot-time power and heap handoff, not a RAW capture client.
* ``docs/j616s-bringup.md`` lines 752-789 explicitly distinguish a stable RAW
  snapshot after stop from a completed buffer report and say orchestration remains
  private. Commit ``743e4e75`` adds that diagnostic RAW evidence as documentation;
  it does not add a RAW receiver implementation. Lines 832-841 qualify one
  completed 1080p P010 frame, with measured FPS still unqualified.

These findings do not deny the recorded RAW snapshot. They mean a V4L2 Bayer
FOURCC, frame-completion contract and per-config rates cannot be derived from
these files without additional runtime replies or capture evidence. In particular,
the old 0x103 SIF example must not be treated as an IMX958 Bayer-format declaration.

AOP endpoints and separate audio stream
-----------------------------------------

The modern ``fw/aop/client.py`` map in PR 20 is 0x20 SPUApp, 0x21 wakehint,
0x22 audio, 0x23 voice trigger, 0x24 accelerometer, 0x25 gyroscope, 0x26 lid
angle, 0x27 ALS and four endpoints at 0x28-0x2b with unqualified roles. The
separate ENDPOINTS13 map assigns different roles and is not the T6040 map.

The documented HQ audio sequence uses the advertised aop-audio service at
endpoint 0x22, EPIC version 2, checked attachment/property replies and owned
DART mappings. It first establishes LP readiness using adpx buffer property 300,
attachments lpai/pdm0/adpx, lpai pw1, lai attachment and lai/lpai runn. HQ then
attaches hpai, requests pw1, prepares stream-10 NS RX0 DMA and starts DMA before
requesting hpai pwrd. Matching reports and two full copies precede RX stop and
return through pw1 to idle. These are audio commands, not camera mode selectors.

The qualified ISP preview helper contains no AOP command. This camera change
therefore does not replay the AOP audio stream or alter its Linux driver. The
ISP channel and command maps below describe the camera path independently.

Mode and format selection
-------------------------

On T6040/IMX958, probe queries every advertised configuration, bounded at 64,
using the 0x120-byte H17 ``CH_CAMERA_CONFIG_GET`` exchange. The complete list
is available through dynamic debug at probe and cached in full for
``VIDIOC_LOG_STATUS`` (``v4l2-ctl --log-status``), without sending firmware
commands. Collect this diagnostic dump before starting a stress run. It includes
input/output dimensions and the sensor clock word; that word is not FPS.

The board's existing config-5 1920x1080 and 1280x720 presets stay first and
NV12 stays the default. Additional distinct output sizes are appended only
when the reply's 16-bit and repeated 32-bit dimensions agree, the dimensions
are nonzero, output is even and within input, and output is at most 16384
in either dimension. The full reported output becomes the crop and output
size for its own config index. Input dimensions alone never add a mode.
Thus a larger consistent firmware output can be selected by ``S_FMT`` and
listed by ``ENUM_FRAMESIZES``; no invented full-resolution preset is added.
These consistency checks cannot establish image quality, supported FPS or
whether a configuration needs additional sensor-specific commands.

``V4L2_PIX_FMT_P010`` is offered only for T6040/IMX958. The config-5
1280x720 and 1920x1080 processed previews are enabled when
the runtime input geometry matches the source profile. An unspecified P010
size selects 1280x720 first. Larger P010 modes require a consistent firmware
output reply. P010 uses two little-endian 16-bit component planes with ten useful
MSB-aligned bits, and equal 64-byte-aligned Y/UV strides. UV immediately
follows Y, as required for contiguous P010. Both NV12 and P010 use the
standard single-planar V4L2 API with one memory plane containing both
component planes. The firmware still receives two component addresses.

At 1920x1080, stride is 3840, Y is 4147200 bytes, UV is 2073600 bytes and
``sizeimage`` is 6220800. At 1280x720, stride is 2560 and ``sizeimage`` is
2764800. The proxy's independently page-aligned plane extents were
0x3f8000/0x1fc000 at 1080p and 0x1c4000/0xe4000 at 720p; inserting those
inter-plane gaps into a V4L2 contiguous P010 buffer would misdescribe it.
Linux passes its actual contiguous extents to the rendered pool instead.

P010 uses the proxy's eight metadata slots (0x8000 bytes each) and two
rendered slots. The queue limits userspace to two rendered buffers, matching
the pool capacity. Completion matches either both full submitted addresses
or both zero-extended low-32-bit plane addresses, plus plane count, pool 9
and a fresh 64-bit submission tag; an old tag
cannot release a new lease. Buffer completion is published to videobuf2
only after the firmware report ACK and doorbell have been published.
NV12 pool sizes and the T8140/Neo default formats, presets and command path
retain their existing behavior.

The inherited ``multiplanar`` module parameter and its inconsistent queue-type
hooks are removed. All sensors use the shipped single-planar V4L2 API; the
non-contiguous NV12M alias is not advertised. No driver
module/kernel parameter, platform flag or per-platform Kconfig option selects
support. NV12 stays first and is the initial format because browsers/WebRTC
and many applications do not support P010 or simply choose the first format.
Applications can select P010 explicitly with ``VIDIOC_S_FMT``.

Endpoints
---------

These are the seven shared-memory ISP channels, not AOP/RTKit endpoints.
The table records the 25G76 source fixture. The current driver negotiates
channel names, types, sources, counts and 64-bit addresses on each boot;
it does not impose the captured ring addresses or counts on 26.6.2.

================= ==== ====== ===== =======================================
Channel           Type Source Slots Purpose
================= ==== ====== ===== =======================================
TERMINAL          2    0      768   Firmware log reports / terminal ACKs
IO                0    1      8     Host commands and in-place replies
DEBUG             0    1      8     Discovered; no capture commands sent
BUF_H2T           0    2      64    Host metadata/output descriptor batches
BUF_T2H           1    3      64    Firmware buffer completion/return reports
SHAREDMALLOC      1    3      8     Firmware allocation/free requests
IO_T2H            1    3      8     Discovered; no capture handler installed
================= ==== ====== ===== =======================================

A source word selects a doorbell bit, so IO and DEBUG share bit 1; the
three reply channels share bit 3. Each channel descriptor is 0x100 bytes
with a u64 ring address at 0x50. Each slot is 0x40 bytes / eight u64 words.
Publish the payload before arg0, which transfers slot ownership. The
existing IPC parser bounds rings against the allocation, descriptor table,
boot/command storage and other rings. The threaded IRQ services allocations,
terminal messages and buffer reports while a command waits for its ACK.

Command stream
--------------

IO commands have a zero/status u32 followed by the opcode u32, then the
parameters below. Sizes include that eight-byte header. ``ch`` is zero.

================ =================================== ==== ===================
Opcode           Command                             Size Parameters
================ =================================== ==== ===================
0x0003           CONFIG_GET                          0x1c Camera channel count
0x3206           SET_DSID_CLR_MULTI_BC_REG_BASE       0x50 8 slots, range 0x2fc
0x0000           START                               0x0c Proxy 0x11, Linux 0
0x010d           CH_INFO_GET                         0x190 ch
0x0106           CH_CAMERA_CONFIG_GET                0x120 ch, index (all)
0x0024           FLICKER_SENSOR_SET                  0x0c 0
0x013b           CH_SBS_ENABLE                       0x10 ch, 1
0x0107           CH_CAMERA_CONFIG_SELECT             0x10 ch, selected config
0x010e           CH_BUFFER_RECYCLE_MODE_SET          0x10 ch, 1 (empty only)
0x010f           CH_BUFFER_RECYCLE_START             0x0c ch
0x080c           CH_CROP_SCL1_SET                    0x1c ch, x, y, width, height
0x0b09           CH_OUTPUT_CONFIG_SCL1_SET           0x38 ch, output geometry
0x0208           CH_AE_FRAME_RATE_MAX_SET            0x10 ch, fps * 256
0x020a           CH_AE_FRAME_RATE_MIN_SET            0x10 ch, min_fps * 256
0x0117           CH_BUFFER_POOL_CONFIG_SET (meta)    0x9c ch, pool 0, count 8
0x0117           CH_BUFFER_POOL_CONFIG_SET (output)  0x9c ch, pool 9, count 2
0x0125           CH_LOCAL_RAW_BUFFER_ENABLE          0x10 ch, 1
0x0b0d           CH_PREVIEW_STREAM_SET               0x10 ch, 1
0x0138           CH_MASTER_SLAVE_SYNC_MODE_SET       0x10 ch, 0
0x0100           CH_START                            0x0c ch
0x0101           CH_STOP                             0x0c ch
0x0104           CH_BUFFER_RETURN                    0x0c ch
================ =================================== ==== ===================

The DSID bases are 0x2201d4000, 0x2221d4000, 0x2241d4000, 0x2261d4000,
then four U64_MAX sentinels. Linux retains its existing boot-time PMP and
logging commands in addition to discovery; it does not replay the proxy's
boot allocations or private power orchestration.

For config 5, crop is (0, 0, output width, output height). The output command
parameters after ch are width, height, Rec.709=1, **format=0x12**, Y stride,
UV stride, 0, 0, height, 0, width. NV12 continues to use format 0.
Metadata pool sizes/strides are 0x8000, one data block; the output pool
contains actual Y/UV sizes and strides, two data blocks, no compression.

Before CH_START, BUF_H2T submits metadata and output buffers as a 16-byte
header (type 1, count) and 64-byte descriptors. Each descriptor contains
four u64 addresses, four u32 flags (0x40000000 for each used plane),
u32 plane count, u32 pool and u64 tag. BUF_T2H returns completion reports;
render status 0x10000000 means a completed frame. Submission status is
0x30000000 and return ACK status is 0x80000000. Requeue only a returned lease.
CH_STOP and CH_BUFFER_RETURN retire the stream; resident firmware remains
running for the existing reopen path.

The proxy used AE max/min 7680/3840 (30/15 fps). Linux retains its V4L2 rate
and exposure-priority controls: the default holds 30/30, and enabling
``exposure_auto_priority`` allows 30/15. These are requested limits, not FPS
measurements. P010 follows the minimal proxy sequence without adding
setfile-load, CNR, MBNR, SIF-format or temporal-filter commands; NV12 keeps
its existing calibrated path. A previously loaded setfile may still be
resident when switching streams.

26.6.2 comparison
-----------------

The 25G76 firmware UUID differs from the 26.6.2 image. Matching struct sizes
is source-level ABI evidence, not a new live qualification of P010.

* Boot block: both use 0x290 bytes, IPC address at 0x08, shared base/size at
  0x10/0x18, extra heap at 0x20/0x28, platform at 0x30, queue offset+1 at
  0x50, 0x40 at 0x68, camera scheme at 0xd0 and the no-optical-card flag
  at 0x120. The current H17 path checks descriptor flag bit 1.
* Memory: 25G76 pinned TEXT/DATA sizes 0xca8000/0x1344000, remap 1<<40,
  shared base 0x1fec000, and a 256 MiB shared aperture. Linux uses the
  bootloader's current TEXT/DATA/main-memory reservations, backs the missing
  main-memory tail, and retains T6040's **0xde600000** shared-size descriptor.
  No pinned segment size, heap request, IOVA or firmware hash is ported.
* IPC: 0x100-byte descriptors, 0x40-byte slots, 64-bit addresses and the
  seven names agree. Counts and addresses remain negotiated, with the
  existing overlap checks retained. The driver uses its three-DART IOMMU
  domain and bootloader address-filter setup, not proxy DART table writes.
* Commands: DSID's 0x50-byte eight-slot layout,
  CH_INFO_GET's 0x190-byte record, scaler-1 crop/output, 0x9c-byte pools,
  local RAW, preview, sync and channel start/stop layouts agree with the
  current H17 builders. START is an important parameter difference: the
  proxy sends 0x11, while the working Linux H17 boot path sends 0. Linux
  retains its current boot command; issuing START a second time after
  selecting P010 would violate the proxy's guarded startup lifecycle. The new config query explicitly exchanges 0x120,
  rather than the unused legacy 0xdc current-config helper. Header/channel/
  index checks and duplicated geometry checks guard decoding. The H17
  appended bytes remain uninterpreted.
* Metadata: CH_INFO_GET reports 0x4d00 on both paths. The proxy rounds its
  metadata backing/pool to 0x8000; P010 does the same even on a 4 KiB host.
  NV12 retains the current driver's 0x4d00 pool and 16-buffer capacity.
  Neither path configures a T6040 capture-metadata pool or capture watchdog.
* Startup: Linux retains its once-per-boot resident-firmware admission,
  current reset-vector check and H17 handshake; the 25G76 SHA/UUID admission
  and recovery model from PR 231 are not substituted. Linux's command
  timeout/IRQ service remain in use; the proxy's 30-second inactivity /
  40-second wall startup budget is not copied.

Validation on hardware must record the current config list first, then
exercise both P010 presets, requeue, stop/reopen and a longer run. If a larger
firmware output appears, test its format and command requirements separately.
Do not label input dimensions, a successful query, or a requested rate as a
completed full-resolution stream or measured FPS.

Open qualification and performance checks
-----------------------------------------

``tools/testing/selftests/media/apple-isp-stress.c`` exercises long runs,
repeated start/stop and format/size switches through MMAP. It logs completed
frames, delivered FPS, maximum inter-frame gap, sequence gaps and buffer errors.
Use the established nominal rate as its explicit rate argument. A driver-generated
sequence is not a firmware frame counter, so timing and sequence checks alone
cannot prove the sensor dropped no frames internally.

The P010 path requeues even the last returned lease into an otherwise empty
queue, publishes completion ACKs before releasing output or metadata leases,
acquires message validity before reading payload and bounds each IRQ ring drain
to one ring traversal so terminal/allocation traffic cannot starve completions.
Command replies must match the requested reply length, zero status and original
header. A timeout or malformed ACK latches failure; no late ACK can authorize
another command. The existing forced coprocessor stop runs before uncertain
leases are released. Resident firmware then needs a reboot before another capture.
These corrections address source-visible stalls/races. They are not evidence
that the intermittent stalls observed by the USB proxy have been reproduced
or cured on the current firmware.

No verified IMX958 RAW Bayer packing/order, RAW completion path or nominal FPS
for every configuration is present in the public source. The recorded RAW trial
reads stable memory after stop but receives no completed RAW report. A Bayer
FOURCC cannot be chosen from that evidence. The RAW format and an exhaustive
per-config FPS table remain open pending the original capture evidence and
runtime command replies; neither is inferred from the P010 output or input height.
The 30/15 values in the preview are AE requests, not per-mode nominal measurements.

The T6040 device tree's current fixed TEXT/DATA reservations should eventually
come from a bootloader handoff describing the loaded image and mapping lifetimes.
They are firmware-build-specific. This change does not implement that handoff
or replace those reservations with 25G76 addresses.
