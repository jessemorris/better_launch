import logging

from textual.widgets import Static
from textual.containers import HorizontalGroup
from textual.color import Color

from utils.colors import get_contrast_color


class LogEntry(HorizontalGroup):
    _source_colormap = {}

    # https://coolors.co/d621ff-ef476f-ffd166-2a6eff-858585
    colormap = {
        "DEBUG": Color(133, 133, 133).css,
        "INFO": Color(42, 110, 255).css,
        "WARNING": Color(255, 209, 102).css,
        "ERROR": Color(34, 97, 231).css,
        "CRITICAL": Color(214, 33, 255).css,
    }

    iconmap = {
        "DEBUG": "⚑",  # "›»§🔍"
        "INFO": "✓",  # "●@#i🏷️"
        "WARNING": "▲",  # "🚧"
        "ERROR": "⨯",  # "✗!🛑"
        "CRITICAL": " 🔥",  # "🔥⚡" (initial space gets swallowed)
    }

    def __init__(
        self,
        record: logging.LogRecord,
    ):
        super().__init__()
        self.record = record

    def get_color_for_source(self, source: str) -> Color:
        if source not in LogEntry._source_colormap:
            LogEntry._source_colormap[source] = Color(*get_contrast_color())
        return LogEntry._source_colormap[source]

    def compose(self):
        r = self.record

        source = Static(f"{r.name}:", id="source")
        # TODO could also use r.rgb, but its existence depends on where the log comes from
        source.styles.color = self.get_color_for_source(r.name)
        yield source

        icon = Static(LogEntry.iconmap.get(r.levelname, "INFO"), id="icon")
        icon.styles.color = LogEntry.colormap.get(r.levelname, "INFO")
        yield icon

        msg = Static(r.msg, id="message")
        yield msg
