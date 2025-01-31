from textual.binding import Binding
from textual.widgets import ListView, ListItem, Label
from textual.containers import VerticalGroup
from textual.screen import ModalScreen

from .node_status import NodeStatus


class NodeMenu(ModalScreen):
    """Custom widget that extends Container."""

    BINDINGS = [
        Binding("esc", "dismiss")
    ]

    DEFAULT_CSS = """
    NodeMenu {
        background: rgba(0, 0, 0, 0.3);
        align: center middle;

        ListView {
            max_width: 30;
            width: auto;
            height: auto;
            padding: 1 2;
        }
    }
    """

    def __init__(self, node, *, name = None, id = None, classes = None):
        self.node = node
        super().__init__(name, id, classes)

    def compose(self):
        # TODO is None?
        yield Label("Hello")

        items = [
            ListItem(Label(action)) 
            for action in ["info", "kill"]
        ]
        yield ListView(*items)

    def on_list_view_selected(self, item: ListItem):
        # TODO item.name is not valid for ListItem
        if item.name == "toggle mute":
            self.node.mute = not self.node.mute
        elif item.name == "kill":
            # TODO
            pass
        else:
            # Unknown command
            return

        self.dismiss()
