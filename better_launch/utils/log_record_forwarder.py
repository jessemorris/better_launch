from typing import Callable
import logging

from utils.log_formatter import RosLogFormatter


class LogRecordForwarder(logging.Handler):
    def __init__(self, level=0):
        super().__init__(level)
        self.formatter = RosLogFormatter(disable_colors=True)
        self.listeners = []

    def add_listener(self, callback: Callable[[logging.LogRecord], None]):
        self.listeners.append(callback)

    def emit(self, record):
        # The formatter will extract information like levelname and set it on the record
        self.format(record)
        for cb in self.listeners:
            cb(record)

    def setFormatter(self, fmt):
        # Direct access to self.formatter is still allowed
        raise RuntimeError("setFormatter is disabled for TextualLogHandler")
