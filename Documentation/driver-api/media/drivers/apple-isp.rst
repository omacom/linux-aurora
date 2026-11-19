.. SPDX-License-Identifier: GPL-2.0

Apple ISP sensor tuning
=======================

Stream setup loads model-wide sensor tuning using ``CH_SET_FILE_LOAD``
(0x0111). A missing file allows streaming with a warning. An invalid file,
interrupted read or failed firmware upload fails stream setup instead of
silently continuing. A debug message identifies a successful upload without
dumping its contents.

Sensor setfiles are distinct from individual camera-module factory
calibration (iCCl). The driver has no iCCl parser, serial binding or upload
command. No factory record or vendor payload belongs in the kernel source
or a distributed firmware package. The separate ANE video denoiser is not
implemented by this driver.

Command submission alone does not establish the resulting noise reduction
or qualify a firmware metadata layout. In particular, these error-handling
changes do not qualify J613 firmware 26.6.2 preview or its image quality.
