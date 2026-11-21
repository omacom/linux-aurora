# SPDX-License-Identifier: GPL-2.0-only
"""The xART guard's first-setup window spans provisioning and activation.

Under xart_guard_pins on a cold-boot Mac, the install's first Touch ID setup
gets a window that pins the SEP-xART lockers it rewrites (KUnit:
apple_sep_xart_guard). On a MacBook Air M1 (J313) the enclave rewrites them
at COPY_KEYBAG, after the create, and during the first activation, after its
LOAD_KEYBAG. So the window opens before the create, is recorded in the list
before the unfinished-create intent counts the list, is handed to the first
activation once the keybag is provisioned, and is shut by every exit before
that.
"""
from pathlib import Path
import re
import unittest

APPLE = Path(__file__).resolve().parents[3] / "drivers/soc/apple"


def function(text, name):
    """The source of `fn name`, from its signature to its closing brace."""
    match = re.search(rf"\bfn {name}\b", text)
    if match is None:
        raise AssertionError(f"fn {name} not found")
    opening = text.index("{", match.end())
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]


class FirstSetupWindowTest(unittest.TestCase):
    def setUp(self):
        self.sks = (APPLE / "sks.rs").read_text()
        self.sbio = (APPLE / "sbio.rs").read_text()
        self.provision = function(self.sks, "sks_provision_identity_keybag")
        self.activate = function(self.sbio, "activate_touchid")

    def test_provisioning_opens_the_window_before_the_intent_and_the_create(self):
        steps = [
            "self.sks_req_create_identity_keybag(",
            "store.open_first_setup()",
            "SetupWindow {",
            "store.guard_listed_now()",
            "KeybagCreateIntent::new(",
            "intent.sending()",
            "crate::xart_guard::keybag_create_window(",
            "self.sks_req_copy_keybag(handle)",
            "window.leave_open_for_activation()",
            '"sks: identity keybag provisioned\\n"',
        ]
        places = [self.provision.find(step) for step in steps]
        self.assertNotIn(-1, places, steps)
        self.assertEqual(places, sorted(places), steps)
        for step in steps:
            self.assertEqual(self.provision.count(step), 1, step)
        # Only the drop guard shuts it during provisioning.
        self.assertNotIn("close_first_setup", self.provision)
        # Nothing else opens it.
        self.assertEqual(
            sum((APPLE / f).read_text().count("store.open_first_setup()")
                for f in ("sks.rs", "sbio.rs", "sep.rs", "refkey.rs")), 1)

    def test_the_holder_shuts_the_window_on_drop(self):
        holder = self.sks[self.sks.index("impl Drop for SetupWindow"):]
        holder = function(holder, "drop")
        self.assertIn("if self.open", holder)
        self.assertIn("store.close_first_setup()", holder)
        hand_over = function(self.sks, "leave_open_for_activation")
        self.assertIn("self.open = false", hand_over)

    def test_the_first_activation_holds_the_window_to_its_return(self):
        hold = self.activate.find(
            "let _window = crate::sks::SetupWindow::for_activation(self);")
        self.assertNotEqual(hold, -1)
        # After the early returns of a started or failed activation, before
        # the first fallible step.
        self.assertLess(self.activate.find("self.touchid_failed.load(Relaxed)"), hold)
        self.assertLess(hold, self.activate.find("store::Store::open()"))
        self.assertLess(hold, self.activate.find("self.sks_recover("))
        self.assertNotIn("drop(_window)", self.activate)
        self.assertNotIn("close_first_setup", self.activate)

    def test_no_activation_after_a_failed_registration_keeps_it_open(self):
        bringup = function(self.sbio, "finish_keystore_bringup")
        failed = bringup[bringup.index("if let Err(e) = self.register_bio()"):]
        failed = failed[:failed.index("\n        }\n") + 10]
        self.assertIn("store.close_first_setup()", failed)


if __name__ == "__main__":
    unittest.main()
