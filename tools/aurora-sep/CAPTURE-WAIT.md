# Bounded idle verification

An idle verification no longer tears down a healthy ARMED capture every 60 seconds. It can wait for a finger within a fixed five-minute deadline measured from the start of `await_capture`. The deadline is not refreshed by ARMED reports, transitions, IRQs or a late frame. Cancellation or suspend still invalidates the operation immediately at the next session check.

Five minutes is a proposed host policy: it leaves margin beyond the earlier combined candidate's attended 185-second idle wait, while bounding a caller that remains alive but never cancels. It is not a measured SEP nonce lifetime or a claim that a sensor remains reliable or power-efficient for five minutes. The earlier combined result does not qualify this changed branch; maintainers can choose a different finite bound after reviewing the caller policy.

Enrollment retains its absolute 60-second deadline. Verification also retains the existing 60-second bound while initially arming and after capture progress begins or an armed sensor leaves ARMED. Returning to ARMED cannot restart that progress budget. The five-minute limit overrides it if sooner, including when a status read returns a ready frame at or after the limit. Deadline expiry is now checked before any ARMED renewal or ready-frame acceptance. This tightens the old boundary ordering: a first ARMED report at or after the arming deadline cannot revive it, and a late frame cannot bypass an expired enrollment or progress deadline. A zero-length frame, patch loss or status failure retains its existing outcome. Frame processing, nonce checks, persistence and settling delays are unchanged.

## Caller lifetime

`pam_fprintd` supplies an independent timeout, 30 seconds by default, and calls D-Bus `VerifyStop` when its verification window expires. The Omarchy sudo and polkit entrypoints use `pam_fprintd.so` without a timeout override, so their ordinary no-touch wait remains governed by PAM. Administrators may change that setting, including disabling its timeout. This kernel patch does not change PAM policy or promise password fallback for every administrator-defined stack.

The Omarchy lock uses a PAM context for fingerprint verification, aborts that context on unlock or its reachability watchdog, and may schedule a new attempt after a completed one. A longer-lived lock prompt can therefore contain multiple bounded attempts. The kernel limit applies to one capture, not to the lifetime of that UI.

In fprintd, `VerifyStop`, release and disappearance of the claiming D-Bus client cancel the underlying operation. The Aurora libfprint driver's cancellation callback issues `AURORA_BIO_CANCEL`, and closing its device description also cancels the kernel session. A kernel capture timeout is a terminal `AURORA_BIO_STATUS_TIMEOUT`, not the retry status that fprintd automatically restarts. A custom client that stays connected and never cancels can no longer hold one kernel capture forever; it still owns its D-Bus claim until it releases it or disconnects. The fprintd `--no-timeout` service option concerns idle daemon exit, not the PAM verification deadline.

The wait checks session validity before and after each SPI status read. IRQ mode has a 250 ms backstop; polling mode sleeps 2 ms between reads. These bound the interval between checks when hardware calls return, not a malfunctioning SPI transaction. The patch does not establish a hard deadline for a blocked transport call.

## Offline verification

`python3 -m unittest discover -s tools/aurora-sep/tests -p test_capture_wait.py -v` compiles the production wait method with synthetic clocks and hardware leaves. Thirty-one Rust cases cover delayed touch beyond 185 seconds, both wait modes, enrollment and progress bounds, fixed verification expiry, frames just before and at expiry, a status read crossing expiry, cancellation priority, epoch changes, transport errors and spurious IRQs. Additional exact/just-before-boundary cases cover initial arming, enrollment and progress, including status reads crossing their deadlines. Four compiled negative controls restore the old minute-long idle timeout, remove cancellation checks, remove the fixed verification bound and move progress expiry behind renewal/frame acceptance; each must fail its named behavioral assertion.

These are host tests. Physical five-minute expiry, idle power, firmware or nonce lifetime, and actual suspend behavior on this changed branch remain unqualified. No sensor is opened by the tests.
