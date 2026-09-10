"""Highlight task failures without changing native MoveIt log configuration."""

import logging
import os
import sys


class ConsoleFormatter(logging.Formatter):
    """Color failures red and retry warnings yellow; keep ordinary INFO uncolored."""

    def __init__(self, color=False):
        """Choose whether to include ANSI terminal color sequences."""
        super().__init__("[mtc_prototype] [%(levelname)s] %(message)s")
        self.color = color

    def format(self, record):
        """Retain severity text even when terminal colors are disabled."""
        message = super().format(record)
        if not self.color:
            return message
        if record.levelno >= logging.ERROR:
            return f"\033[1;31m{message}\033[0m"
        if record.levelno >= logging.WARNING:
            return f"\033[33m{message}\033[0m"
        return message


def make_logger():
    """Configure the shared console handler with explicit environment overrides."""
    mode = os.environ.get("TRUNKING_LOG_COLOR", "auto").lower()
    color = "NO_COLOR" not in os.environ and (
        mode == "always" or (mode == "auto" and sys.stdout.isatty())
    )
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(ConsoleFormatter(color))
    # All package modules, including trajectory cost callbacks, share this handler.
    logging.basicConfig(level=logging.INFO, handlers=[handler])
    return logging.getLogger("dual_fr3_trunking_mtc.mtc_prototype")
