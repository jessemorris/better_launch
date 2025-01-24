from textual.widgets import Label
from textual.color import Color
from textual.reactive import reactive


class NodeStatus:
    ALIVE = 2
    ERROR = 1
    DEAD = 0
    UNKNOWN = -1

    def __init__(
        self, name: str, namespace: str = None, status: int = ALIVE
    ):
        self.name = name
        self.namespace = namespace or ""
        self.status = status
        self.has_errors = False
        self.mute = False

    @property
    def fullname(self):
        if self.namespace:
            return f"/{self.namespace.strip('/')}/{self.name}"
        return "/" + self.name.strip("/")

    def __repr__(self):
        return self.fullname


class NodeStatusLabel(Label):
    # https://coolors.co/64c05d-ffd166-ef476f-858585
    colormap = {
        NodeStatus.ALIVE: Color(100, 192, 93).css,
        NodeStatus.ERROR: Color(255, 209, 102).css,
        NodeStatus.DEAD: Color(239, 71, 111).css,
        NodeStatus.UNKNOWN: Color(133, 133, 133).css,
    }

    _status = reactive(NodeStatus.UNKNOWN)

    def __init__(self, keybind: str, node: NodeStatus):
        self.keybind = keybind or ""
        self.node_details = node

        super().__init__(self.keybind + " " + node.fullname, id=node.fullname.replace("/", "_"))

    @property
    def status(self):
        return self.node_details.status

    @status.setter
    def status(self, status: int):
        self.node_details.status = status
        self._status = status

    def watch_status(self, status: int):
        self.styles.color = self.colormap.get(status, self.UNKNOWN)
