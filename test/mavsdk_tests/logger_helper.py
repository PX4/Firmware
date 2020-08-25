#!/usr/bin/env python3

import re
import sys
import os
from enum import Enum
from functools import lru_cache


class color(Enum):
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    DARKCYAN = '\033[36m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    GRAY = '\033[90m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'


def colorize(text: str, c: color) -> str:
    if _supports_color():
        return str(c.value) + text + color.RESET.value
    else:
        return text


def maybe_strip_color(text: str) -> str:
    """Remove ANSI and xterm256 color codes.

    From https://stackoverflow.com/a/30500866/8548472
    """
    if not _supports_color():
        return re.sub(r'\x1b(\[.*?[@-~]|\].*?(\x07|\x1b\\))', '', text)
    else:
        return text


@lru_cache()
def _supports_color() -> bool:
    """Returns True if the running system's terminal supports color.

    From https://stackoverflow.com/a/22254892/8548472
    """
    supported_platform = \
        (sys.platform != 'Pocket PC') and \
        (sys.platform != 'win32' or 'ANSICON' in os.environ)

    # isatty is not always implemented.
    is_a_tty = hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()

    return supported_platform and is_a_tty
