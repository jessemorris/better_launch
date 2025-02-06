from textual.binding import Binding
from textual.widgets import Label, Static, Button
from textual.containers import VerticalScroll
from textual.screen import ModalScreen

from better_launch import BetterLaunch
from elements import Node, LifecycleNode, Composer


class NodeLabel(Label):
    def __init__(self, node: Node, keybind: str):
        self.node = node
        self.keybind = keybind
        super().__init__(f"[u]{keybind}[/u] {node.name}")
        # TODO watch node status


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

    def __init__(self, node: Node, **kwargs):
        self.node = node
        super().__init__(**kwargs)

    def compose(self):
        node = self.node

        # ROS2 prints a lot of useless stuff and avoids the things that are interesting most of
        # the time, like who is actually subscribed where. Let's fix this!
        shared_node = BetterLaunch.wait_for_instance().shared_node

        # Additional information about the node subclass
        node_type_info = ""
        if isinstance(node, LifecycleNode):
            node_type_info = f"Stage:     {node.current_stage.name.capitalize()}\n"
        elif isinstance(node, Composer):
            components = [f"  - {c}\n" for c in node.loaded_components]
            node_type_info = f"\n[bold]Components:[/bold]\n{components}\n"

        # Topics the node is publishing
        pubs = shared_node.get_publisher_names_and_types_by_node(
            node.name, node.namespace
        )
        pubs.sort()
        pubs_text = ""
        for topic, types in pubs:
            pubs_text += f"\n  {topic} [{', '.join(types)}]"

        # Topics the node is subscribed to
        subs = shared_node.get_subscriber_names_and_types_by_node(
            node.name, node.namespace
        )
        subs.sort()
        subs_text = ""
        for topic, types in subs:
            subs_text += f"\n  {topic} [{', '.join(types)}]"

        info_text = f"""\
[bold]{node.name} ({node.__class__.__name__})[/bold]
Status:    {'[green]alive[/green]' if node.is_running else '[red]dead[/red]'}
Namespace: {node.namespace}
{node_type_info}\

[bold]Process:[/bold]
  PID:     {node.pid}
  Command: {node.cmd}
  Args:    {node.node_args}
  Env:     {node.env}

[bold]Publishers:[/bold] {pubs_text}

[bold]Subscriptions:[/bold] {subs_text}
"""

        yield(VerticalScroll(Static(info_text), id="info"))
        yield(Button("Close", variant="primary", id="close"))

    def on_mount(self):
        self.query_one("#close").focus()

    def on_button_pressed(self, event):
        self.dismiss()
