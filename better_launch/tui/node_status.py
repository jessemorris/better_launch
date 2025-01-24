import re

from textual.widgets import Label
from textual.color import Color
from textual.reactive import reactive


class NodeStatus(Label):
    ALIVE = 2
    ERROR = 1
    DEAD = 0
    UNKNOWN = -1

    # https://coolors.co/64c05d-ffd166-ef476f-858585
    colormap = {
        ALIVE: Color(100, 192, 93).css,
        ERROR: Color(255, 209, 102).css,
        DEAD: Color(239, 71, 111).css,
        UNKNOWN: Color(133, 133, 133).css,
    }

    status = reactive(ALIVE)

    def __init__(
        self, keybind: str, name: str, namespace: str = None, status: int = ALIVE
    ):
        if not namespace:
            match: re.Match = re.match(r"/?(.*)/([\w\d+-_ ]+)", name)
            if not match:
                raise ValueError(f"Could not parse node name '{name}' - missing namespace")
            namespace = match.group(1)
            name = match.group(2)

        # TODO some of these are already defined in the base
        # TODO separate this into a dataclass and a label implementation
        self.keybind = keybind or ""
        self.name = name
        self.namespace = namespace.strip("/")
        self.status = status
        self.has_errors = False
        self.mute = False

        # TODO should use a Horizontal here so we can format the keybind separately
        super().__init__(self.keybind + " " + self.name, id=self.fullname)

    @property
    def fullname(self):
        return f"/{self.namespace}/{self.name}"

    def watch_status(self, status: int):
        self.styles.color = self.colormap.get(status, self.UNKNOWN)
