#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""The deferred store wait ends bring-up, and remove() joins that wait."""

import unittest
from pathlib import Path

SEP_RS = Path(__file__).resolve().parents[3] / "drivers" / "soc" / "apple" / "sep.rs"


def function_body(text, name):
    marker = "fn %s" % name
    start = text.find(marker)
    if start < 0:
        raise AssertionError("no %s" % marker)
    # Next fn at column 4 is the following method. Good enough for these two.
    rest = text[start + len(marker):]
    nxt = rest.find("\n    fn ")
    if nxt < 0:
        nxt = len(rest)
    return rest[:nxt]


class StoreWaitTest(unittest.TestCase):
    def setUp(self):
        self.text = SEP_RS.read_text()

    def test_deferred_attach_failures_end_bringup(self):
        body = function_body(self.text, "store_wait_tick")
        gone = body.split("the SEP node is gone", 1)[1].split("shutting_down", 1)[0]
        self.assertIn("diag::bringup_ended()", gone)
        self.assertIn("finish_bringup()", gone)
        failed = body.split("could not boot the SEP", 1)[1].split("arm_settle", 1)[0]
        self.assertIn("diag::attach(false)", failed)
        self.assertIn("diag::bringup_ended()", failed)
        self.assertIn("finish_bringup()", failed)

    def test_remove_joins_store_wait_before_dropping_the_mailbox(self):
        body = function_body(self.text, "remove")
        wait = body.find("store_wait_work")
        mailbox = body.find("*self.mbox.lock() = None")
        self.assertGreater(wait, 0)
        self.assertGreater(mailbox, wait)


if __name__ == "__main__":
    unittest.main()
