.. SPDX-License-Identifier: GPL-2.0

=========
Kernel CI
=========

The GitHub Actions workflows in ``.github/workflows/kernel-ci*.yml`` check,
build, test and boot this kernel for every change to ``integration/aurora``.
The tools they run live here, so every step can also be run by hand.

What runs when
==============

Builds take time and, on some runners, money. Pull requests get the checks
that run on free GitHub-hosted runners on every push; the team's pull
requests are also built on every push, and everyone's kernel is built again
in the merge queue, from exactly the tree that will land, unless a run at
the pull request's head already tested exactly that tree (see below).

====================  =================================================
Event                 Jobs
====================  =================================================
pull request push     plan, commits, compile, device trees, host tests,
                      CI self-test
trusted author push   all of the above, then package, KUnit and boot
                      (not on drafts)
``ci:build`` label    all of the above, then package, KUnit and boot
merge queue           everything, or the checks only when an eligible
                      run at the head being merged tested exactly the tree
                      being merged
push to the branch    package (reused from a branch or queue run, else
                      built), attestation
nightly               package, KUnit and boot, rebuilt from scratch
manual run            everything, never reusing an earlier result
====================  =================================================

``ci / result`` is the one check branch protection and the merge queue
require. ``result.py`` knows which jobs each kind of run must pass, so a
skipped job never counts as a pass when it was due.

The jobs
========

plan (``plan.py``, ``lookup-build.py``)
  Decides between a light and a full run from the event, then names the
  build by its inputs: the kernel tree (leaving out ``.github`` and
  ``tools/aurora-ci``, which never reach the package) and the parts of
  omarchy-pkgs that build it (the linux-aurora recipe, ``build/``,
  ``bin/build`` and the helpers it sources). Results are stored as
  artifacts under that key, so the same inputs are never built or tested
  twice: the push after a merge reuses the queue's packages, and a change to
  the CI alone reuses the last build. The nightly run rebuilds from scratch,
  because the Arch Linux ARM packages the builder installs change on their
  own.

commits (``check-commits.py``)
  Every commit needs a ``Signed-off-by`` trailer (blocking). checkpatch
  findings are reported but do not fail the job. A change under
  ``include/uapi`` gets a warning: Mesa and other userspace built against
  those headers needs a matching update before the kernel ships. Ranges of
  more than 100 commits (rebases onto a new upstream) are reported only.

compile (``compile-check.py``)
  Compiles, with ``W=1`` and the shipped configuration, the objects a change
  touches (plus a sample of the objects that include a changed header), at
  the merge base and at the head, and reports warnings that are new. It also
  compiles that set at every commit of the change (up to 20), so a commit
  that does not build on its own fails the job and bisection keeps working.
  Object selection comes from kbuild, not from guessing at file names. Rust
  objects, linking and modpost are left to the package build.

device trees (``dt-check.py``, ``dt-requirements.txt``)
  When a change touches device trees, DT bindings or dtc, builds every Apple
  DTB at the base and the head, schema-checks the DTBs whose output changed
  and the bindings that changed, and reports only problems the change adds.
  DTBs that fail to build and invalid or dropped bindings block; schema and
  style findings are reported. A change that touches no DT files finishes in
  under a second.

host tests (``host-tests.py``, ``host-tests.json``)
  The hermetic test suites under ``tools/``: Python and Rust tests of driver
  logic built against the real sources, and the tools' own tests. Each entry
  in ``host-tests.json`` says how to run it; ``known_failure`` marks a suite
  that fails today, with the reason, so it is reported without blocking;
  ``excluded`` lists the test-like files that cannot run here (hardware,
  root, network) and why. A unit test fails when a new test-like file is in
  neither list. Suites run without sudo, with ``TMPDIR`` outside the home
  directory, and must leave the tree as clean as they found it.

CI self-test
  The unit tests in ``tests/``, actionlint, and zizmor (pedantic persona,
  online audits) for the workflows.

package (``prepare-recipe.py``, ``inspect-package.py``, ``compare-packages.py``)
  Builds the kernel with the recipe that ships it: omarchy-pkgs'
  ``pkgbuilds/linux-aurora``, run through omarchy-pkgs' own ``bin/build`` in
  its builder container. ``prepare-recipe.py`` changes three lines (the
  commit, a CI subrelease on ``pkgrel`` so a CI package never has a
  published version, and the kernel source, which is read from the checkout
  instead of cloned again) and refuses to guess if the recipe changed shape.

  ``inspect-package.py`` checks what installs rely on: a ``-ARCH`` kernel
  release, a little-endian arm64 Image with the configured page size, device
  trees, ``modules.dep``, ``pkgbase``, every module symbol resolving, and
  Rust enabled. ``compare-packages.py`` diffs the result against the target
  branch's package: configuration, device trees, modules, image size and
  contents. A device tree that stops being built fails the job unless
  ``retired-dtbs.txt`` lists it with a reason.

KUnit (``run-kunit.py``, ``aurora.kunitconfig``, ``kunit-expected.json``)
  Every KUnit suite for the Apple, Asahi and Aurora code, built into one
  arm64 kernel with the shipped kernel's 16K pages and run in QEMU. Results
  are compared with ``kunit-expected.json``, which lists the cases that skip
  by design and the kernel warnings known to occur inside passing cases;
  anything new, changed or missing fails.

boot (``boot-test.py``, ``boot-expected.json``)
  Boots the packaged kernel, not a test build, in a QEMU ``virt`` guest with
  a busybox initramfs, loads every module the package ships, and fails on a
  crash or warning in the log, a bad taint, a release mismatch, or a module
  that fails to load for a reason other than missing hardware. The guest has
  none of the Apple hardware, so this proves the kernel boots and every
  module initialises; driver behaviour is tested on Macs.

  Without KVM, QEMU emulates a Neoverse N1 rather than ``-cpu max``: ``max``
  enables whatever the QEMU version emulates, and on QEMU 8 that stalls this
  kernel's boot and faults in page clearing. A named model behaves the same
  on every QEMU and has the 16K granule.

Who can start a build
=====================

Light runs execute a pull request's code on GitHub-hosted runners with a
read-only token, as every public repository's CI does. Full runs start from
a trusted author or a maintainer action:

- a push to a pull request, from a fork too, once it is out of draft, by
  anyone vouched for in ``.github/VOUCHED.td`` (the kernel team), when the
  pusher is vouched for too. Access to the repository or the omacom
  organization alone does not count;
- queueing a pull request, pushing to the branch, or a manual run;
- applying ``ci:build`` (which needs triage access). On a branch of this
  repository the label applies to every later push; on a fork it applies to
  the head it was applied to, and has to be applied again after a new push.

``VOUCHED.td`` lists the team: one ``github:<login>`` per line, sorted.
``-github:<login> <reason>`` denounces an author, which the label does not
override. The plan job reads the list from the target
branch through the API, so a pull request that edits it changes nothing until
it merges. To add someone, open a pull request adding their line.

GitHub holds every run of a pull request from an outside collaborator until
a maintainer approves it. ``kernel-ci-approve.yml`` (``approve-runs.py``)
releases the held ``kernel-ci.yml`` run of the current head when the author
is vouched for, so they need no clicks either. It runs on
``pull_request_target`` and fetches only the target branch's files; no pull
request code runs with its write token. Anyone else's runs stay held, and
once approved they run the checks only unless a maintainer applies
``ci:build``.

On a ``pull_request`` event GitHub runs the workflow files from the pull
request itself, so these rules decide cost, not security: a fork could edit
them. For the same reason a pull request's run is not a source of results
before review, and a manual run of another branch never is: both run code
nobody has reviewed yet. Earlier packages, verification records and
baselines are reused from ``kernel-ci.yml`` runs on ``integration/aurora``
and its merge queue branches (``lookup-build.py``), so no one can plant a
package that a later run would skip testing and attest.

The one exception is the merge queue itself. A queued pull request has been
approved after its last push, and a push takes it out of the queue, so a
run of that pull request at the head being merged (the queue commit's
second parent) ran the reviewed code, provided it ran on the merge GitHub
makes with ``integration/aurora``: a run of the same head in another pull
request, say one onto a branch with other workflows, runs that branch's
workflows. So the queue takes the packages and verification of a run
instead of building and booting again only when:

- the run is ``kernel-ci.yml`` or ``kernel-ci-label.yml`` on a pull request
  event in this repository, at that head, and passed at its first attempt
  (never re-run);
- GitHub recorded exactly one call of ``kernel-ci-jobs.yml`` for it, from the
  run's own workflow file (ref ``refs/pull/<number>/merge`` of the pull request
  the queue is merging, path ending in the commit it ran), and that commit's
  tree is exactly the queue commit's tree, CI files and expected results
  included;
- the run has exactly one live package and one live verification record for
  the build key, and the record names the key, the run and that commit.

The package job downloads that very artifact by its ID. A later queue
entry also carries the entries ahead of it, so its tree normally differs
and it builds; an equal tree is taken. The queue still runs the checks and,
when the branch has a baseline, compares the packages with it, but uploads
no reusable package, metadata or verification record, so nothing from a
pull request becomes evidence a later run trusts. Anything else, or
evidence that cannot be read, leaves the usual lookup, which trusts only
the branch and the queue.

What this trusts: GitHub's record of which pull request and commit the run
ran, and, within that run, the verification record, which is the run's own
word. Nothing shows which job of the run wrote it, or that it verified
those exact package bytes, so code at the reviewed head that ran in that
run could have written it. And for these pull requests any trusted build
and test still needed happens after the merge: the push takes the usual
path for the key (a branch or queue run's packages and verification, else a
build, and KUnit and boot unless a branch or queue run already verified
the key), so a failure there comes after the change has merged.

What keeps untrusted code off
paid runners is GitHub's setting to
require approval before running workflows from outside collaborators, and,
for a runner like Namespace's, a runner group limited to this repository.

Runners
=======

Every job runs on GitHub-hosted runners, which cost nothing for a public
repository. ``package`` and ``KUnit`` use the repository variable
``KERNEL_CI_RUNNER`` when it is set: a JSON ``runs-on`` value such as
``["namespace-profile-arm64-linux-apple-12x32"]``, the machine omarchy-pkgs
builds kernels on. Without it they use ``ubuntu-24.04-arm``, where the
package build takes about 45 minutes.

Repository variables do not reach a fork's pull request run, so a fork would
always get that 45 minute build. When both the author and whoever triggered
the run are vouched for, ``plan.py`` names the Namespace runner for the fork
instead. Any other fork stays on GitHub's runner, a maintainer's ``ci:build``
label included. This only routes honest pull requests: a pull request's own
copy of the workflow can ask for any runner, so what keeps untrusted code off
Namespace is still the approval and runner group described above.

Repository setup
================

These need an administrator:

- a ``ci:build`` label;
- ``.github/VOUCHED.td`` kept to the team (a pull request, like any other);
- branch protection for ``integration/aurora`` requiring ``ci / result``;
- the merge queue (merge method: merge commit), so the tree that merges is
  the tree that was tested;
- workflow approval for all outside collaborators (Settings, Actions,
  General), which ``kernel-ci-approve.yml`` relies on;
- optionally ``KERNEL_CI_RUNNER`` for faster builds.

Running the tools locally
=========================

All tools are Python 3.10+ standard library scripts; run them with
``python3 -I`` from the top of the tree. ``python3 -I -m unittest discover -s
tools/aurora-ci/tests`` runs the unit tests.

Build a package the way CI does (needs Docker and an omarchy-pkgs checkout)::

  python3 -I tools/aurora-ci/prepare-recipe.py \
      --recipe ../omarchy-pkgs/pkgbuilds/linux-aurora --kernel . \
      --commit "$(git rev-parse HEAD)" --revision 1
  (cd ../omarchy-pkgs && bin/build --mirror edge --arch aarch64 --package linux-aurora)

Check and compare packages (any directory holding the ``linux-aurora`` and
``linux-aurora-headers`` ``.pkg.tar.zst`` files)::

  python3 -I tools/aurora-ci/inspect-package.py --packages new/ --output new.json --unpacked root/
  python3 -I tools/aurora-ci/compare-packages.py --base old.json --head new.json \
      --retired tools/aurora-ci/retired-dtbs.txt

Boot it (needs ``qemu-system-aarch64`` and a static aarch64 busybox)::

  python3 -I tools/aurora-ci/boot-test.py --root root/ --busybox /bin/busybox \
      --expected tools/aurora-ci/boot-expected.json --out boot/

Run KUnit (``--install-toolchain`` sets up Ubuntu; elsewhere, install LLVM
18, Rust 1.85.0 with rust-src and bindgen-cli 0.71.1)::

  python3 -I tools/aurora-ci/run-kunit.py --output ../kunit-build

Run the host suites, or one group or suite::

  python3 -I tools/aurora-ci/host-tests.py --output ../host-tests --jobs 4
  python3 -I tools/aurora-ci/host-tests.py --output ../host-tests --group tools
  python3 -I tools/aurora-ci/host-tests.py --list

Check a range of commits the way a pull request is checked::

  python3 -I tools/aurora-ci/check-commits.py --base origin/integration/aurora --head HEAD
  python3 -I tools/aurora-ci/compile-check.py --tree . --base origin/integration/aurora \
      --head HEAD --config ../omarchy-pkgs/pkgbuilds/linux-aurora/config \
      --fragment arch/arm64/configs/asahi.config --out ../compile --per-commit --llvm=-18
  python3 -I tools/aurora-ci/dt-check.py --tree . --base origin/integration/aurora \
      --head HEAD --out ../dt-check

Verifying a package
===================

Packages built for a commit on ``integration/aurora`` carry a build
provenance attestation::

  gh attestation verify linux-aurora-<version>-aarch64.pkg.tar.zst --repo omacom/linux-aurora
