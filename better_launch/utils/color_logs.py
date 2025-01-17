import logging


class ColorFormatter(logging.Formatter):
    def __init__(
        self,
        fmt=None,
        datefmt=None,
        style="%",
        validate=True,
        *,
        defaults=None,
        level_color_map=None,
    ):
        super().__init__(fmt, datefmt, style, validate, defaults=defaults)

        if level_color_map is None:
            level_color_map = {
                logging.DEBUG: "\x1b[38;20m",
                logging.INFO: "\x1b[39;0m",
                logging.WARNING: "\x1b[33;20m",
                logging.ERROR: "\x1b[31;20m",
                logging.CRITICAL: "\x1b[31;1m",
            }

        self.formatters = {
            level: logging.Formatter(
                color + fmt + "\x1b[0m",
                datefmt,
                style,
                validate,
                defaults=defaults,
            )
            for level, color in level_color_map.items()
        }

    def format(self, record):
        formatter = self.formatters.get(record.levelno)
        if formatter:
            return formatter.format(record)
        
        return super().__format__(record)
