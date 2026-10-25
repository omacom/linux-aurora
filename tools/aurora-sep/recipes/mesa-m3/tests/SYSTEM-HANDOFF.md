# Legacy mesa-m3 to system Mesa

With native Mesa installed in `/usr`, the legacy file opt-out still forced
`LIBGL_ALWAYS_SOFTWARE=1`. This prevented an accelerated login even when the
system driver worked. The legacy package owns this hook, so the native Mesa
package must not overwrite it or add an unconditional late unset.

This change allows a file opt-out on J613/T8122 and J516S/T6030 to leave system
Mesa selected. It requires one accessible Asahi render node, the matching
native display, a readable healthy kernel journal, no unmatched same-boot
legacy login attempt, and no conflicting environment or later user settings.
A bounded helper then opens the exact EGL device using only `/usr` libraries
and verifies its native renderer and loaded implementation paths. It creates
a context but makes no draw, swap or compute calls.

With `--system-provider`, the user-setup check also refuses settings the
clean-environment helper cannot see: an `LD_LIBRARY_PATH` entry holding its
own libEGL, libGLESv2, libGL, libGLX, libglapi or libgbm, and
`GBM_BACKEND`, `__GLX_VENDOR_LIBRARY_NAME` and the Mesa GL/GLES/GLSL version
overrides. It splits `LD_LIBRARY_PATH` on `:` and `;` as glibc does, and a
relative or empty entry (the current directory) is unknown. The admission
returns within a 20-second timeout, so a stalled journal, home directory or
scanner refuses instead of holding the login. The helper keeps its own
5-second timeout.

A failed check keeps the existing software fallback. `mesa_m3=off` and
`nomodeset` remain recovery choices. Other boards, including J615, remain
outside this system handoff. Explicit legacy profile selection is unchanged.
No opt-in flags, attempt records or success markers are added or removed.
Only the exact software-fallback pair previously exported by this hook can
be cleared. The state remains `inactive`, `reason=opt-out`, `fallback=none`
when system Mesa admission succeeds.

The helper proves only that the system EGL device and a context open. It does
not allocate GBM buffers, render or present. A system-Mesa login that passes
admission and then fails later is not recorded: this path writes no attempt,
so the legacy success-marker check cannot see it, and the next login is
admitted again. Recover with `mesa_m3=off` on the kernel command line or by
removing the opt-out file. Separate system-Mesa attempt and success markers
would make that recovery automatic.

Run the host checks from this directory (a writable TMPDIR may be needed):

```sh
sh test-session-env.sh
bash test-session-env.sh
sh test-user-setup.sh
python3 test-system-provider.py
cc -std=c11 -Wall -Wextra -Werror -pedantic -O2 test-system-board.c -ldl -o /tmp/test-system-board
/tmp/test-system-board
```

Validation on J516S with kernel `7.2.11-mixtape-77648eb77168-g15` and the native
Mesa source `97a65d845f39418aecc693dcaf9dcb14a05849f6`: the helper ran as the
desktop user without opt-in variables and returned
`Apple M3 Pro (G15S B1)`, exit 0. No new kernel log messages, package changes
or reboot. This is device/context admission, not a packaged login test,
rendering qualification, CTS result or support claim for other boards.

Before an image cutover, build the updated legacy package and verify a fresh
login alongside the actual pinned Omacom Mesa packages. Existing legacy users
and unqualified profiles must retain their recovery path; clean system-Mesa
images can omit the legacy package. The Mesa source and package PRs remain
separate from this hook update.
