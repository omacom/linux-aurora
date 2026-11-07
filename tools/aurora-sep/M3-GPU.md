# M3 GPU quick start

For the **13-inch and 15-inch M3 MacBook Air (J613/J615)** with the supported
14.8.3 stub, the matched installer prepares the experimental profile.

**Current status:** the matched J613 exact26.6.2/25G83 profile has a reported
cold boot, native desktop and passing GPU checker. J615 current14 and the
explicit J615 native25 profile are packaged, with physical qualification
tracked in [#26](https://github.com/omacom/linux-aurora/issues/26). Current14
and exact26.6.2 profiles remain separate; an installed package or working panel
alone does not establish acceleration. Native25 supports experimental OpenGL
and reports Vulkan unavailable.

1. Enable the experimental GPU profile:

   ```sh
   curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --m3-gpu
   ```

2. Reboot and log into your desktop normally.

3. Run:

   ```sh
   aurora-m3-gpu-check
   ```

Successful acceleration must report **PASS Apple GPU OpenGL (red/blue readback)** and, on the legacy
profile, **PASS Apple GPU Vulkan (compute readback)**. This runs real GPU
work through the desktop session and checks its current Mesa profile,
render node and loaded libraries. An installed package or working panel
alone does not prove acceleration. Use `--details` only when reporting a
failure. The check does not qualify compositor scanout or GPU conformance.

If boot or login fails, select **Aurora previous (GPU off)** in Limine or
the retained previous kernel in GRUB; report the exact error with the
same installer's `--m3-report` output.

The matched installer keeps a GPU-off fallback. `--m3-gpu-experiment`
only installs one-shot tools and is unnecessary for this persistent route.
The command and installed checker are available in **Aurora 2026.10.10.1**;
12.6 supports Air 13 with `--m3-gpu-persistent` and its Mesa probe instead.

| Mac | Route |
| --- | --- |
| Air 13-inch J613 / Air 15-inch J615 | `--m3-gpu` with the new matched bundle |
| Pro 16-inch J516S | Default supported display/GPU handoff install |
| Pro 14-inch J514S | First activate with `--m3-handoff` |
| Base-M3 Pro/iMac, M3 Max | GPU handoff support remains pending |

An existing Pro desktop can run the downloaded installer with
`--m3-gpu-check` without installing or changing packages.

## Firmware profiles

System firmware 26.6.2 does **not** select GPU firmware. The legacy
OS-firmware label can be 14.7 while its actual stub and GPU image are
14.8.3. The installer reads exact GPU compatibility when available and
checks the actual stub/iBoot when an unarmed legacy boot omits it. Clean
`v1.6.1` stage 1 is admitted only under the matched legacy Air contract,
with the exact stub/iBoot and no log overlap. It does not flash stage 1.

J613 already booted from its own exact 26.6.2/25G83 volume group with an
existing matched `v1.6.1-m3next.stage1` handoff may use
`--m3-profile=j613-25g83`. This selects `/opt/mesa-m3/25g83` and
`j613-25g83-hal200`. Its checker reports OpenGL readback and **Vulkan
unavailable**. J615 native25 requires its own explicit option and matched stage1, described
below. Native scratch shaders are
refused; power calibration remains experimental. Legacy 14/Pro Vulkan
stays separate. Firmware and calibration remain specific to each Mac;
the installer migrates neither firmware nor stage 1.

## J613 native25 update

Aurora 2026.10.10.1 includes the GPU memory-region and register-window
corrections, leaves MTP startup to Linux through U-Boot, and separates Neo
component registration from the existing display driver. The native25 Mesa
hook allows the display node to use the apple kmsro driver while its render
node uses Asahi. No manual DTB edit, Neo initcall blacklist or Mesa loader
override is part of this installation.

On a J613 already using its own exact 26.6.2/25G83 volume group and matched
stage 1, update with:

```sh
curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --m3-profile=j613-25g83
```

Reboot, log in and run `aurora-m3-gpu-check`. The native25 profile reports
OpenGL readback and Vulkan unavailable. Keyboard/trackpad recovery, suspend,
night light and extended stability still require hardware verification on
this update. J615 native25 requires its own explicit experimental profile.

## J615 native25

The 15-inch M3 Air requires its own exact 26.6.2/25G83 volume group and
J615-capable stage1 `v1.6.1-m3air25.stage1`. A matched release must list J615
native25 capability and contain the admitted kernel, m1n1 and Mesa package pair.
Other bundles refuse activation.

```sh
curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --m3-profile=j615-25g83
```

This selects experimental native OpenGL and sets the independent J615 boot
switch. Vulkan hardware support is unavailable. `--m3-gpu` and the J613 option
do not enable this profile on a J615. Firmware and calibration remain specific
to this Mac; neither the installer nor this option migrates firmware or stage1.
J615 hardware qualification is pending. The retained **Aurora previous (GPU off)**
entry remains available. After reboot and desktop login, run
`aurora-m3-gpu-check --details` and collect the installer's `--m3-report`.

## J613/J615 temperature-guarded performance ceiling

The T8122 default ceiling remains state 2. With the state-2 fallback repair in
this driver, an explicit `asahi.t8122_pstate_cap=5 asahi.m3_thermal=on` permits
states up to 5 only while the governor has usable die-temperature readings.
Do not apply this higher ceiling to an older kernel: its implementation can
also raise the boot and sensor-failure cap.

The repaired driver starts at state 2 and immediately returns there for missing,
failed, stale, stuck or implausible temperature readings, or at the critical
temperature. A lower requested ceiling still wins. `m3_thermal=off` and `hold`
do not permit states above 2. Non-T8122 cap selection is unchanged.

J615 now supplies `macsmc_soc_die` from the six Tg GPU-family keys and the six
CPU die keys also used on J613. All twelve must be discovered before the J615
zone is registered; a later read failure in any sensor fails the whole reading.
Do not substitute a partial `macsmc_hwmon.soc_die_keys` list. These are raw sensor
readings, not a calibration or a hardware safety certification.

First boot with `asahi.t8122_pstate_cap=5 asahi.m3_thermal=hold` and confirm that
the firmware respects the runtime cap of 2 under GPU load. Before relying on
`m3_thermal=on`, check the boot log for all twelve keys, a runtime range of 2..5
and usable temperatures during GPU work. Keep a bootable previous kernel.
Verify rendering and temperature response on the running kernel; passing the
host policy tests alone does not qualify hardware behavior.
