# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Extract unchanged production function bodies for host boundary tests."""
import re


def function(source, name):
    match = re.search(r"^[A-Za-z_][\w *]*\b" + re.escape(name) +
                      r"\([^;{}]*?\)\n\{", source, re.M)
    if not match:
        raise ValueError(name)
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"
