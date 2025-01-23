import logging
from textual.app import App

from utils.log_formatter import RosLogFormatter


class TextualLogHandler(logging.Handler):
    def __init__(self, app: App, level=0):
        super().__init__(level)
        self.app = app
        self.formatter = RosLogFormatter(disable_colors=True)

    def emit(self, record):
        # The formatter will extract information like levelname and set it on the record
        self.format(record)
        self.app.call_from_thread(self.app._on_logging_event, record)

    def setFormatter(self, fmt):
        raise RuntimeError("setFormatter is disabled for TextualLogHandler")
