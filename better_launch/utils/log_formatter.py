from typing import Any
import re
import random
import logging
from datetime import datetime


default_colormap = {
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


class RosLogFormatter(logging.Formatter):
    default_screen_format = "[{levelcolor}{levelname}{colorreset}] [{formattercolor}{name}{colorreset}] [{asctime}]\n{message}"
    default_file_format = "[{levelname}] [{asctime}] {message}"

    def __init__(
        self,
        format: str = default_screen_format,
        timestamp_format: str = "%Y-%m-%d %H:%M:%S.%f",
        *,
        defaults: Any = None,
        roslog_pattern: str = r"%%([\w]+)%%([\d.]+)%%(.*)",
        colormap: dict[int, str] = None,
        disable_colors: bool = False,
    ):
        super().__init__(format, timestamp_format, "{", True, defaults=defaults)

        self.converter = datetime.fromtimestamp
        self.roslog_pattern = re.compile(roslog_pattern)
        self.colormap = colormap if colormap is not None else dict(default_colormap)
        self.mycolor = random.randint(1, 255)
        self.disable_colors = disable_colors

    def format(self, record):
        match = self.roslog_pattern.match(record.msg)

        if match:
            record.levelname = match.group(1)
            record.levelno = logging.getLevelName(record.levelname)
            record.created = float(match.group(2))
            record.msg = match.group(3)

        if self.disable_colors:
            record.levelcolor = ""
            record.formattercolor = ""
            record.colorreset = ""
        else:
            record.levelcolor = self.colormap.get(record.levelno, "")
            record.formattercolor = f"\x1b[38;5;{self.mycolor}m"
            record.colorreset = "\x1b[0m"

        return super().format(record)

    def formatTime(self, record, datefmt=None):
        dt: datetime = self.converter(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime(self.default_time_format)
