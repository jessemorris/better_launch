import logging

from textual.widgets import Static
from textual.color import Color


class LogEntry(Static):
    # https://coolors.co/d621ff-ef476f-ffd166-2a6eff-858585
    colormap = {
        "DEBUG": Color(133, 133, 133).css,
        "INFO": Color(42, 110, 255).css,
        "WARNING": Color(255, 209, 102).css,
        "ERROR": Color(34, 97, 231).css,
        "CRITICAL": Color(214, 33, 255).css,
    }

    def __init__(
        self,
        record: logging.LogRecord,
        format: str = "{name:8s} {msg}",
    ):
        self.record = record
        display = format.format(**record.__dict__)

        super().__init__(
            display,
        )

        self.styles.background = LogEntry.colormap.get(record.levelname, "INFO")
