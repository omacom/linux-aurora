# MacBook Neo GPU installation

For an existing Arch/Omarchy installation on a MacBook Neo (J700/T8140), use the installer from its matched release packet:

```sh
bash install-aurora-sep.sh --neo-gpu
```

Reboot and log in again. The installer puts the Neo kernel, J700 ESP bootloader and `mesa-neo` into one checked package transaction. It preserves this Neo's U-Boot, firmware and per-machine calibration. Native OpenGL and Borealis Vulkan use `/opt/mesa-neo`; M1–M3 keep their existing graphics profiles.

Check acceleration from the graphical session:

```sh
/opt/mesa-neo/bin/mesa-neo-probe
```

The probe checks the renderer, OpenGL color/depth and indexed drawing, multiple render targets, and Vulkan compute results. A package build or shader test does not replace these checks on the Neo itself.

Selection requires the installed `j700-g17p-hal200` marker, `/etc/mesa-neo/gpu-experiment`, and a T8140 render node whose queried parameters match G17P, USC3, HAL200, the synchronization features and virtual address range. A native graphical session also requires a connected display on the Neo display controller. A simpledrm-only desktop, missing intent or an ABI mismatch selects software rendering. Existing graphical sessions must restart after installation.

To select software rendering:

```sh
sudo rm -f /etc/mesa-neo/gpu-experiment
```

Log out and back in. An ordinary installer update does not restore removed intent; `--neo-gpu` explicitly enables it again. `/etc/mesa-neo/disable` and `~/.config/mesa-neo/disable` also disable native selection. `--uninstall` restores the saved original bootloader files and configuration.

This Neo profile is experimental. External display support is not enabled in its device tree. OpenGL/Vulkan conformance and suspend/resume require tests on the installed hardware.
