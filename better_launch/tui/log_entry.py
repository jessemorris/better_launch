import logging

from textual.widgets import Static
from textual.containers import HorizontalGroup
from textual.color import Color
from rich.color import Color as RichColor


class LogEntry(HorizontalGroup):
    # https://coolors.co/d621ff-ef476f-ffd166-2a6eff-858585
    colormap = {
        "DEBUG": Color(133, 133, 133).css,
        "INFO": Color(42, 110, 255).css,
        "WARNING": Color(255, 209, 102).css,
        "ERROR": Color(34, 97, 231).css,
        "CRITICAL": Color(214, 33, 255).css,
    }

    iconmap = {
        "DEBUG": "⚑",  # "›»§🔍",
        "INFO": "✓",  # "●@#i🏷️",
        "WARNING": "▲",  # "🚧",
        "ERROR": "⨯",  # "✗!🛑",
        "CRITICAL": "🔥",  # "🔥⚡",
    }

    def __init__(
        self,
        record: logging.LogRecord,
    ):
        super().__init__()
        self.record = record

    def compose(self):
        r = self.record

        source = Static(f"{r.name}:", id="source")
        r_color = getattr(r, "sourcecolor_int", 1)
        source.styles.color = Color.from_rich_color(RichColor.from_ansi(r_color))
        yield source

        icon = Static(LogEntry.iconmap.get(r.levelname, "INFO"), id="icon")
        icon.styles.color = LogEntry.colormap.get(r.levelname, "INFO")
        yield icon

        msg = Static(r.msg, id="message")
        yield msg
