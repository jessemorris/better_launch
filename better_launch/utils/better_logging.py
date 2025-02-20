from typing import Any, Callable
import re
import logging
from datetime import datetime

from .colors import get_contrast_color


default_log_colormap = {
    #  0m: resets all colors and attributes.
    # 20m: resets only attributes (underline, etc.), leaving colors unchanged.
    # 39m: resets only foreground color, leaving attributes unchanged.
    # 49m: resets only background color, leaving attributes unchanged. 
    logging.DEBUG: "\x1b[38;20m",
    logging.INFO: "\x1b[34;20m",
    logging.WARNING: "\x1b[33;20m",
    logging.ERROR: "\x1b[31;20m",
    logging.CRITICAL: "\x1b[31;1m",
}


# Taken from ROS2's logging.handlers. Since that module replaces itself this function is not 
# accessible from the outside.
def _with_per_logger_formatting(cls):
    """Add per logger formatting capabilities to the given logging.Handler."""
    class _trait(cls):
        """A logging.Handler subclass to enable per logger formatting."""

        def __init__(self, *args, **kwargs):
            super(_trait, self).__init__(*args, **kwargs)
            self._formatters = {}

        def setFormatterFor(self, logger, formatter):
            """Set formatter for a given logger instance or logger name."""
            logger_name = logger if isinstance(logger, str) else logger.name
            self._formatters[logger_name] = formatter

        def unsetFormatterFor(self, logger):
            """Unset formatter for a given logger instance or logger name, if any."""
            logger_name = logger if isinstance(logger, str) else logger.name
            if logger_name in self._formatters:
                del self._formatters[logger_name]

        def format(self, record):  # noqa
            if record.name in self._formatters:
                formatter = self._formatters[record.name]
                return formatter.format(record)
            return super(_trait, self).format(record)

    _trait.__name__ = cls.__name__
    _trait.__doc__ = cls.__doc__
    return _trait


class PrettyLogFormatter(logging.Formatter):
    default_screen_format = "[{levelcolor}{levelname}{colorreset}] [{sourcecolor}{name}{colorreset}] [{asctime}]\n{message}"
    default_file_format = "[{levelname}] [{asctime}] {message}"

    def __init__(
        self,
        format: str = default_screen_format,
        timestamp_format: str = "%Y-%m-%d %H:%M:%S.%f",
        *,
        defaults: dict[str, Any] = None,
        roslog_pattern: str = r"%%(\w+)%%([\d.]+)%%(.*)",
        pattern_info: list[str] = ("levelname", "created", "msg"),
        level_colormap: dict[int, str] = None,
        color_per_source: bool = True,
        disable_colors: bool = False,
    ):
        """A specialized formatter that will try to extract various details from messages logged by ROS2 nodes and reformat them.

        The following additional keys can be used for formatting messages:
        * {levelcolor}: colors subsequent characters based on the message's severity.
        * {sourcecolor}: colors subsequent characters based on the message's source.
        * {colorreset}: subsequent characters will use the default color again.

        Parameters
        ----------
        format : str, optional
            The format this instance will use when formatting messages.
        timestamp_format : _type_, optional
            The format to use for timestamps (we like human readable here).
        defaults : dict[str, Any], optional
            Defaults the formatter may use when formatting strings.
        roslog_pattern : str, optional
            The pattern used for matching incoming log messages. The pattern's regex groups will be associated with `pattern_info`.
        pattern_info : list[str], optional
            Formatting keys that can be extracted from incoming log messages using `roslog_pattern`, one for each regex group.
        level_colormap : dict[int, str], optional
            Color overrides for severity levels.
        color_per_source : bool, optional
            If True, every distinct log source (according to `LogRecord.name`) will be associated with a different color ({sourcecolor}). Otherwise the color will be chosen per formatter instance; that is, this formatter instance will always use the same color.
        disable_colors : bool, optional
            If True, no color codes will be inserted when formatting messages.
        """
        super().__init__(format, timestamp_format, "{", True, defaults=defaults)

        self.converter = datetime.fromtimestamp
        self.roslog_pattern = re.compile(roslog_pattern)
        self.pattern_info = pattern_info
        self.level_colormap = default_log_colormap | level_colormap if level_colormap else {}
        self.color_per_source = color_per_source
        self.disable_colors = disable_colors
        self.registered_colors = {}

    def get_color(self, source: str) -> tuple[int, int, int]:
        """Return the color associated with the provided source.

        If `color_per_source` is True this will return an individual color for each source. Otherwise it will always return the same color.

        Parameters
        ----------
        source : str
            The name of the source.

        Returns
        -------
        tuple[int, int, int]
            An RGB color triplet.
        """
        if not self.color_per_source:
            source = "self"

        if source not in self.registered_colors:
            self.registered_colors[source] = get_contrast_color()

        return self.registered_colors[source]

    def format(self, record):
        match = self.roslog_pattern.match(record.msg)

        if match:
            for idx, key in enumerate(self.pattern_info):
                setattr(record, key, match.group(idx + 1))

            if "levelname" in self.pattern_info and "levelno" not in self.pattern_info:
                record.levelno = logging.getLevelName(record.levelname)

            elif "levelno" in self.pattern_info and "levelname" not in self.pattern_info:
                record.levelname = logging.getLevelName(record.levelno)

        if self.disable_colors:
            record.rgb = (255, 255, 255)
            record.levelcolor = ""
            record.sourcecolor = ""
            record.colorreset = ""
        else:
            r, g, b = self.get_color(record.name)
            record.rgb = (r, g, b)
            record.levelcolor = self.level_colormap.get(record.levelno, "")
            record.sourcecolor = f"\x1b[38;2;{r};{g};{b}m"
            record.colorreset = "\x1b[0m"

        return super().format(record)

    def formatTime(self, record, datefmt=None):
        try:
            dt: datetime = self.converter(record.created)
            if datefmt:
                return dt.strftime(datefmt)
            return dt.strftime(self.default_time_format)
        except:
            return record.created


class RecordForwarder(logging.Handler):
    def __init__(self, level: int = logging.INFO):
        """A log handler that forwards any records it receives to callbacks.

        Parameters
        ----------
        level : int, optional
            The minimum logging level this handler accepts.
        """
        super().__init__(level)
        self.formatter = PrettyLogFormatter(disable_colors=True)
        self.listeners = []

    def add_listener(self, callback: Callable[[logging.LogRecord], None]):
        self.listeners.append(callback)

    def emit(self, record):
        # The formatter will extract information like levelname and set it on the record
        self.format(record)
        for cb in self.listeners:
            cb(record)


RecordForwarder = _with_per_logger_formatting(RecordForwarder)


class StubbornHandler(logging.Handler):
    """This handler resists the common changes ROS2 attempts to make for its logging so that our formatters can work properly.
    
    The ROS2 launch system assigns the same formatter to many sources using special wrappers. However, we don't want our log forwarders and formatters to be replaced just like that. 

    Parameters
    ----------
    actual_handler : logging.Handler
        The handler which will actually handle any incoming log records.
    """
    def __init__(self, actual_handler: logging.Handler, level: int = logging.INFO):
        super().__init__(level)
        self.actual_handler = actual_handler

    def setFormatterFor(self, logger, formatter):
        return

    def unsetFormatterFor(self, logger):
        return

    def emit(self, record):
        self.actual_handler.emit(record)

    def format(self, record):
        return self.actual_handler.format(record)
