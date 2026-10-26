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
relative or empty entry (the current directory) is unknown. A plain
`$LD_LIBRARY_PATH` reference counts as empty, because an earlier file may
have cleared it; `${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}` passes. The admission
returns within a 20-second timeout, so a stalled journal, home directory or
scanner refuses instead of holding the login. The helper keeps its own
5-second timeout.

A failed check keeps the existing software fallback. `mesa_m3=off` and
`nomodeset` remain recovery choices. Other boards remain outside this system
handoff. Explicit legacy profile selection is unchanged.
Only the exact software-fallback pair previously exported by this hook can
be cleared. The state remains `inactive`, `reason=opt-out`, `fallback=none`
when system Mesa admission succeeds.

## J615 and automatic selection

The 15-inch Air (J615, T8122) is admitted with the same checks as the J613, in
the hook and in `mesa-m3-system-check`. A root that names both Airs, or one of
them twice, is refused.

The file opt-out is no longer required. In a Hyprland session on J613, J615
or J516S, the hook runs the same bounded admission where its `/opt/mesa-m3`
gate ends inactive, except `not-supported` and `user-setup` (the session's
own Mesa setup still decides). An active `/opt/mesa-m3` is never replaced:
moving those sessions to system Mesa would need a maintainer opt-in. When
admission passes, the session keeps system Mesa instead of the software
fallback. The state is `inactive`, `reason=system-mesa`, `fallback=none`; its
detail names the admitted board and the gate's own decision. Any refusal
leaves the gate's decision and fallback exactly as before.

An automatic admission records its attempt first, as an `/opt/mesa-m3` login
does. `mesa-m3-session-confirm` confirms it in the same way, and an
unconfirmed login sends the next login of that boot to the software fallback
instead of a login loop. When the attempt cannot be written, the admission
is dropped: the gate's decision and fallback stay, and the detail says the
attempt was not recorded. The file opt-out path still writes no attempt.

Without a file opt-out, the 25G83 native profile stays on its own gate: an
`/etc/mesa-m3/t8122-profile` selection, or a render node with
`apple,j613-25g83-gpu-handoff`, is not admitted automatically.

The helper proves only that the system EGL device and a context open. It does
not allocate GBM buffers, render or present.

Recovery after a system-Mesa login that passes admission and then fails:

- Automatic (`reason=system-mesa`): the unconfirmed attempt sends the next
  login of the same boot to software. To retry the GPU in that boot, remove
  `${XDG_STATE_HOME:-~/.local/state}/mesa-m3/attempt` and log in again; a
  reboot makes the previous attempt inapplicable. To stay in software, boot
  with `mesa_m3=off`.
- File opt-out (`reason=opt-out`): this path writes no attempt, so the
  success-marker check cannot see the failure and the next login is admitted
  again. To force software rendering immediately, boot with `mesa_m3=off`.
  Removing the opt-out file restores normal selection, which may retry the
  system Mesa automatically.

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
