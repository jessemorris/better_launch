from typing import Any
import re
import logging
from datetime import datetime


default_colormap = {
    logging.DEBUG: "\x1b[38;20m",
    logging.INFO: "\x1b[34;20m",
    logging.WARNING: "\x1b[33;20m",
    logging.ERROR: "\x1b[31;20m",
    logging.CRITICAL: "\x1b[31;1m",
}


class RosLogFormatter(logging.Formatter):
    def __init__(
        self,
        format: str = "[{levelcolor}{levelname}{colorreset}] [{name}] [{asctime}]\n{message}",
        roslog_pattern: str = r"%%([\w]+)%%([\d.]+)%%(.*)",
        timestamp_format: str = "%Y-%m-%d %H:%M:%S.%f",
        *,
        defaults: Any = None,
        colormap: dict[int, str] = None,
    ):
        super().__init__(format, timestamp_format, "{", True, defaults=defaults)
        
        self.converter = datetime.fromtimestamp
        self.roslog_pattern = re.compile(roslog_pattern)
        self.colormap = colormap or default_colormap

    def format(self, record):
        match = self.roslog_pattern.match(record.msg)

        if match:
            record.levelname = match.group(1)
            record.levelno = logging.getLevelName(record.levelname)
            record.created = float(match.group(2))
            record.msg = match.group(3)

        record.levelcolor = self.colormap.get(record.levelno, "")
        record.colorreset = "\x1b[0m"

        return super().format(record)

    def formatTime(self, record, datefmt=None):
        dt: datetime = self.converter(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime(self.default_time_format)
