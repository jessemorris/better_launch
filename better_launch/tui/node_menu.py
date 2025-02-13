import asyncio
from textual import work
from textual.binding import Binding
from textual.widgets import Label, Static, Button
from textual.containers import VerticalScroll, HorizontalGroup
from textual.screen import ModalScreen

from elements import AbstractNode, Composer, Component


class NodeLabel(HorizontalGroup):
    DEFAULT_CSS = """
    NodeLabel {
        width: 100%;
        padding: 0 1;

        #keybind {
            color: orange;
            width: 3;
        }

        #node {
            height: 1;
            width: 100%;
        }
    }
    """

    def __init__(self, node: AbstractNode, keybind: str, **kwargs):
        super().__init__(**kwargs)
        self.node = node
        self.keybind = keybind

    def compose(self):
        yield Static(f"[u]{self.keybind}[/u] ", id="keybind")

        suffix = ""
        if self.node.is_lifecycle_node:
            suffix += "L"
        if isinstance(self.node, Composer):
            suffix += "O"
        if isinstance(self.node, Component):
            suffix += "C"

        if suffix:
            suffix = " (" + suffix + ")"

        yield Label(f"{self.node.name}{suffix}", id="node")

    def on_mount(self):
        self.update_node_state()

    @work
    async def update_node_state(self):
        while True:
            label = self.query_one("#node")
            if self.node.is_running:
                self.styles.background = "green"
            else:
                self.styles.background = "red"

            await asyncio.sleep(0.5)            


class NodeInfoScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss")
    ]

    DEFAULT_CSS = """
    NodeInfoScreen {
        background: rgba(0, 0, 0, 127);
        align: center middle;
        padding: 0;

        #info {
            width: 100%;
            height: 100%;
            margin: 1;
        }

        #close {
            height: auto;
            width: 100%;
            border: none;
        }
    }
    """

    def __init__(self, node: AbstractNode, **kwargs):
        self.node = node
        super().__init__(**kwargs)

    def compose(self):
        info_text = self.node.get_info_sheet()
        yield(VerticalScroll(Static(info_text), id="info"))
        yield(Button("Close", variant="primary", id="close"))

    def on_mount(self):
        self.query_one("#close").focus()

    def on_button_pressed(self, event):
        self.dismiss()
