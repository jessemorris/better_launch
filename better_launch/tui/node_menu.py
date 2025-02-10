import asyncio
from textual import work
from textual.binding import Binding
from textual.widgets import Label, Static, Button
from textual.containers import VerticalScroll, HorizontalGroup
from textual.screen import ModalScreen

from better_launch import BetterLaunch
from elements import AbstractNode, Node, Composer, Component


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
        node = self.node

        # ROS2 prints a lot of useless stuff and avoids the things that are interesting most of
        # the time, like who is actually subscribed where. Let's fix this!
        shared_node = BetterLaunch.wait_for_instance().shared_node

        # Additional information about the node subclass
        if isinstance(node, Component):
            process_id = f"{node.composer.pid} (Composer)"
            env_info = node.composer.env
        else:
            process_id = node.pid
            env_info = node.env

        lifecycle_info = ""
        if node.is_lifecycle_node:
            lifecycle_info = f"Stage:     {node.lifecycle.current_stage.name.capitalize()}\n"

        component_info = ""
        if isinstance(node, Composer):
            components = "\n".join([f"  - {c}" for c in node._loaded_components])
            component_info = f"\n[bold]Components:[/bold]\n{components}\n"

        if node.is_running:
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

            # Provided services
            services = shared_node.get_service_names_and_types_by_node(
                node.name, node.namespace
            )
            services.sort()
            services_text = ""
            for srv, types in services:
                services_text += f"\n  {srv} [{', '.join(types)}]"
        else:
            pubs_text = ""
            subs_text = ""
            services_text = ""

        info_text = f"""\
[bold]{node.name} ({node.__class__.__name__})[/bold]
Status:    {'[green]alive[/green]' if node.is_running else '[red]dead[/red]'}
Package:   {node.package}
Namespace: {node.namespace}
{lifecycle_info}\
{component_info}\

[bold]Process:[/bold]
  PID:     {process_id}
  Command: {node.executable}
  Args:    {node.node_args}
  Env:     {env_info}

[bold]Publishers:[/bold] {pubs_text}

[bold]Subscriptions:[/bold] {subs_text}

[bold]Services:[/bold] {services_text}
"""

        yield(VerticalScroll(Static(info_text), id="info"))
        yield(Button("Close", variant="primary", id="close"))

    def on_mount(self):
        self.query_one("#close").focus()

    def on_button_pressed(self, event):
        self.dismiss()
