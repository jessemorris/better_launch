from textual.widgets import ListView, ListItem, Label

from .node_status import NodeStatus


class NodeMenu(ListView):
    """Custom widget that extends Container."""

    DEFAULT_CSS = """
    NodeMenu {
        align: center middle;
        width: 40;
        padding: 2 4;
        display: none;
    }
    """
    def __init__(self, id: str = None):
        super().__init__(*[
            # TODO add icons
            ListItem(Label(item))
            for item in [
                "toggle mute", 
                "kill",
            ]
        ], id=id)

        self.node = None

    def show_for_node(self, node: NodeStatus):
        # TODO update indicators 
        self.node = node
        self.display = True

    def on_list_view_selected(self, item: ListItem):
        if item.name == "toggle mute":
            self.node.mute = not self.node.mute
        elif item.name == "kill":
            # TODO
            pass
        
        # Unknown command
