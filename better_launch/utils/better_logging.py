from typing import Any, Callable, Literal, Iterable
import os
import re
import logging
from datetime import datetime

import better_launch.ros.logging as roslog
from .colors import get_contrast_color


# Log format string for ROS so that we can identify and reformat its log messages.
ROSLOG_PATTERN_ROS = "%%{severity}%%{time}%%{message}"

# Regular expression matching ROSLOG_PATTERN_ROS. The named groups will be matched to log
# Record attributes via their group names. 
ROSLOG_PATTERN_BL = r"%%(?P<levelname>\w+)%%(?P<created>[\d.]+)%%(?P<msg>[\s\S]*)"


Colormode = Literal["default", "severity", "source", "none", "rainbow"]


default_log_colormap = {
    #  0m: resets all colors and attributes.
    # 20m: resets only attributes (underline, etc.), leaving colors unchanged.
    # 39m: resets only foreground color, leaving attributes unchanged.
    # 49m: resets only background color, leaving attributes unchanged.
    "INFO": "\x1b[92;20m",
    "WARNING": "\x1b[93;20m",
    "ERROR": "\x1b[91;20m",
    "CRITICAL": "\x1b[95;20m",
    "DEBUG": "\x1b[36;20m",
}


# A nice amber for all logging sources
default_source_color = 222


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
    default_screen_format = "[{levelcolor_start}{levelname}{levelcolor_end}] [{sourcecolor_start}{name}{sourcecolor_end}] [{asctime}]\n{message}"
    default_file_format = "[{levelname}] [{asctime}] {message}"

    def __init__(
        self,
        format: str = default_screen_format,
        timestamp_format: str = "%Y-%m-%d %H:%M:%S.%f",
        *,
        defaults: dict[str, Any] = None,
        roslog_pattern: str = None,
        source_colors: str | int | Iterable[int] | dict[str, Any] = default_source_color,
        log_colors: str | int | Iterable[int] | dict[str, Any] = None,
        no_colors: bool = False,
    ):
        """A specialized formatter that will try to extract various details from messages logged by ROS2 nodes and reformat them.

        The following additional keys can be used for formatting messages:
        * {sourcecolor_start}: colors subsequent characters based on the message's source until a {sourcecolor_end} is encountered.
        * {levelcolor_start}: colors subsequent characters based on the message's severity until a {levelcolor_end} is encountered.

        Colors can be specified in 3 ways:
        * an ANSI/VT100 escape sequence, typically starting with `\\e` or `\\x1b`.
        * an integer, referring to a color from the *vte* 256 colors table.
        * a tuple of 3 integers representing an RGB value.

        Parameters
        ----------
        format : str, optional
            The format this instance will use when formatting messages.
        timestamp_format : _type_, optional
            The format to use for timestamps (we like human readable here).
        defaults : dict[str, Any], optional
            Defaults the formatter may use when formatting strings.
        roslog_pattern : str, optional
            The pattern used for matching incoming log messages. The pattern should define named groups that will be matched to logging.Record attributes via their names. 
        source_colors : str | int | Iterable[int] | dict[str, Any], optional
            Colors to use when formatting `sourcecolor_start` tags based on the source of the log report. If a string, integer or iterable, use this as the color for all sources. If None, use a different color for every source. Pass a dict to specify custom colors for a set of sources.
        log_colors : str | int | Iterable[int] | dict[str, Any], optional
            Colors to use when formatting `levelcolor_start` tags based on the log report's severity. If a string, integer or iterable, use this as the color for all sources. If None, use the default log colors from `default_log_colormap`. Pass a dict to override the colors for select log levels identified by name.
        no_colors: bool, optional
            If True, all colors will be disabled.
        """
        super().__init__(format, timestamp_format, "{", True, defaults=defaults)

        self.converter = datetime.fromtimestamp

        if not roslog_pattern:
            roslog_pattern = ROSLOG_PATTERN_BL
        self.roslog_pattern = re.compile(roslog_pattern)

        self.source_colors = {}
        if isinstance(source_colors, dict):
            self.source_colors.update(source_colors)
        elif source_colors is not None:
            self.source_colors["*"] = source_colors

        self.log_colors = dict(default_log_colormap)
        if isinstance(log_colors, dict):
            self.log_colors.update(log_colors)
        elif log_colors is not None:
            self.log_colors["*"] = log_colors

        if no_colors:
            self.source_colors["*"] = ""
            self.log_colors["*"] = ""

    def get_source_color(self, source: str) -> str:
        """Return the color associated with the provided source."""
        if "*" in self.source_colors:
            source = "*"
        elif source != "*" and source not in self.source_colors:
            self.source_colors[source] = get_contrast_color()

        color = self.source_colors.get(source, "")
        color_start = self.format_color(color)
        color_end = "\x1b[0m" if color_start else ""

        return color_start, color_end

    def get_loglevel_color(self, level: int | str) -> tuple[str, str]:
        if "*" in self.log_colors:
            level = "*"
        elif isinstance(level, int):
            level: str = logging.getLevelName(level)

        color = self.log_colors.get(level, "")
        color_start = self.format_color(color)
        color_end = "\x1b[0m" if color_start else ""

        return color_start, color_end

    def format_color(self, color: Any) -> str:
        if isinstance(color, str):
            return color

        if isinstance(color, int):
            return f"\x1b[38;5;{color}m"

        if isinstance(color, (tuple, list)):
            return f"\x1b[38;2;{color[0]};{color[1]};{color[2]}m"

        return ""

    def formatTime(self, record, datefmt=None):
        try:
            dt: datetime = self.converter(record.created)
            if datefmt:
                return dt.strftime(datefmt)
            return dt.strftime(self.default_time_format)
        except Exception:
            return record.created

    def format(self, record):
        match = self.roslog_pattern.match(record.msg)

        if match:
            for key, val in match.groupdict().items():
                # If we replace an existing key, make sure the value type matches the original,
                # otherwise we will get some problems with formatting down the line
                key_type = type(getattr(record, key, ""))
                val = key_type(match.group(key))
                setattr(record, key, val)

            if "levelname" in match.groupdict() and "levelno" not in match.groupdict():
                record.levelno = logging.getLevelName(record.levelname)

            elif (
                "levelno" in match.groupdict() and "levelname" not in match.groupdict()
            ):
                record.levelname = logging.getLevelName(record.levelno)

        record.sourcecolor_start, record.sourcecolor_end = self.get_source_color(
            record.name
        )
        record.levelcolor_start, record.levelcolor_end = self.get_loglevel_color(
            record.levelno
        )

        return super().format(record)


class RecordForwarder(logging.Handler):
    def __init__(self, formatter, level: int = logging.INFO):
        """A log handler that forwards any records it receives to callbacks.

        Parameters
        ----------
        level : int, optional
            The minimum logging level this handler accepts.
        """
        super().__init__(level)
        self.formatter = formatter
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


LogSink = Literal["screen", "log", "own_log", "none"]


def configure_logger(
    logger: logging.Logger,
    config: LogSink | Iterable[LogSink] = None,
    screen_formatter: logging.Formatter = None,
    log_formatter: logging.Formatter = None,
) -> None:
    if not config:
        config = {"screen"}
    elif isinstance(config, str):
        config = {config}
    else:
        config = set(config)

    for sink in config:
        if sink == "screen":
            screen_handler = roslog.launch_config.get_screen_handler()
            if screen_handler not in logger.handlers:
                if not screen_formatter:
                    screen_formatter = roslog.launch_config.screen_formatter
                
                screen_handler.setFormatterFor(logger, screen_formatter)
                logger.addHandler(screen_handler)

        elif sink == "log":
            common_log_handler = roslog.launch_config.get_log_file_handler()
            if common_log_handler not in logger.handlers:
                if not log_formatter:
                    log_formatter = roslog.launch_config.file_formatter
                
                common_log_handler.setFormatterFor(logger, log_formatter)
                logger.addHandler(common_log_handler)

        elif sink == "own_log":
            own_log_handler = roslog.launch_config.get_log_file_handler(logger.name + ".log")
            if own_log_handler not in logger.handlers:
                if not log_formatter:
                    log_formatter = roslog.launch_config.file_formatter

                own_log_handler.setFormatterFor(logger, log_formatter)
                logger.addHandler(own_log_handler)


def init_logging(
    log_config: roslog.LaunchConfig,
    screen_log_format: str = None,
    file_log_format: str = None,
    colormode: Colormode = "default",
) -> None:
    if not screen_log_format:
        screen_log_format = PrettyLogFormatter.default_screen_format

    if not file_log_format:
        file_log_format = PrettyLogFormatter.default_file_format

    if colormode == "default":
        src_color = default_source_color
        log_color = None
    elif colormode == "severity":
        src_color = ""
        log_color = None
    elif colormode == "source":
        src_color = None
        log_color = ""
    elif colormode == "none":
        src_color = ""
        log_color = ""
    elif colormode == "rainbow":
        src_color = None
        log_color = None
    else:
        raise ValueError("Invalid colormode " + colormode)

    # We'll handle formatting and color ourselves, just get the nodes to comply
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = ROSLOG_PATTERN_ROS
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "0"

    log_config.level = logging.INFO
    log_config.screen_formatter = PrettyLogFormatter(
        format=screen_log_format, source_colors=src_color, log_colors=log_color
    )
    log_config.file_formatter = PrettyLogFormatter(format=file_log_format)
