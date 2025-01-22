import os
import time
import re
import logging
import asyncio
import pyperclip

from textual import work
from textual.binding import Binding
from textual.reactive import reactive
from textual.app import App, ComposeResult
from textual.widgets import ListView, ListItem, Label, Header, Footer, Static, 
from textual.containers import Horizontal, Container
from textual.color import Color

from ros2node.api import get_node_names

from ros import logging as roslog
from ros.ros_adapter import ROSAdapter
from utils.log_formatter import RosLogFormatter


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
            match: re.Match = re.match(r"/?(.+)/([\w\d]+)", name)
            if not match:
                raise ValueError(f"Could not parse node name '{name}' - missing namespace")
            namespace = match.group(1)
            name = match.group(2)

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


class LogEntry(Label):
    # https://coolors.co/d621ff-ef476f-ffd166-2a6eff-858585
    colormap = {
        "DEBUG": Color(133, 133, 133).css,
        "INFO": Color(42, 110, 255).css,
        "WARNING": Color(255, 209, 102).css,
        "ERROR": Color(34, 97, 231).css,
        "CRITICAL": Color(214, 33, 255).css,
    }

    def __init__(
        self,
        record: logging.LogRecord,
        format: str = "{name:8s} {msg}",
    ):
        self.record = record
        display = format.format(**record.__dict__)

        super().__init__(
            display,
            id=f"{record.name}_{record.created}",
        )

        self.styles.background = LogEntry.colormap.get(record.levelname, "INFO")


class TextualLogHandler(logging.Handler):
    def __init__(self, app: App, level=0):
        super().__init__(level)
        self.formatter = RosLogFormatter(disable_colors=True)

    def emit(self, record):
        # The formatter will extract information like levelname and set it on the record
        self.format(record)
        self.app.call_from_thread(self.app._on_logging_event, record)


# Inspired by xqms' rosmon: https://github.com/xqms/rosmon
class BetterLaunchUI(App):
    TITLE = "BetterLaunch"

    BINDINGS = [
        Binding("f1", "toggle_sidebar", "Toggle sidebar"),
        Binding("f9", "mute_all", "Mute all"),
        Binding("f10", "unmute_all", "Unmute all"),
        Binding("/", "search_node", "Node search"),
        Binding("escape", "close_node_menu", show=False)
    ]

    DEFAULT_CSS = """
    Screen {
        #sidebar {
            width: auto;
            height: 1fr;
            max_width: 40%;
            background: $panel;
        }
        #log_output {
            width: 1fr;
            height: 1fr;
        }
    }
    """

    def __init__(
        self,
        ros_adapter: ROSAdapter,
        disable_colors: bool = False,
        max_log_length: bool = 1000,
    ):
        super().__init__(ansi_color=not disable_colors)

        self.nodes = {}
        self.backlog = []
        self.paused = False

        self.ros_adapter = ros_adapter
        self.max_log_length = max_log_length

        # Make sure all ros loggers follow a parsable format
        os.environ["RCUTILS_COLORIZED_OUTPUT"] = "0"
        os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = (
            "%%{severity}%%{time}%%{message}"
        )

        roslog.launch_config.screen_handler = TextualLogHandler(self)

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        # Could use a Tree view instead
        self.sidebar = ListView(id="sidebar")
        self.logview = ListView(id="logview")

        with Horizontal():
            yield self.sidebar
            yield self.logview

        self.node_menu = NodeMenu()
        yield self.node_menu

        yield Footer(show_command_palette=False)

    def on_mount(self):
        self.check_nodes_status()

    # TODO thread or asyncio? fine for now
    @work(thread=True)
    def check_nodes_status(self):
        live_nodes = set(
            n.full_name for n in get_node_names(node=self.ros_adapter.ros_node)
        )

        for item in self.sidebar.children:
            if not isinstance(item, NodeStatus):
                continue

            # TODO ignoring lifecycle node status for now
            if item.fullname in live_nodes:
                if item.has_errors:
                    item.status = NodeStatus.ERROR
                else:
                    item.status = NodeStatus.ALIVE
            else:
                item.status = NodeStatus.DEAD

        time.sleep(1.0)

    def on_list_view_selected(self, selected: ListView.Selected):
        if selected.list_view == self.sidebar:
            self.open_node_menu(selected.item.get_child_by_type(NodeStatus))
        elif selected.list_view == self.logview:
            self.copy_log_entry(selected.item.get_child_by_type(LogEntry))
    
    def open_node_menu(self, node: NodeStatus):
        if not self.sidebar.display:
            self.action_toggle_sidebar()
        
        self.node_menu.show_for_node(node)
        self.node_menu.focus()

    def copy_log_entry(self, entry: LogEntry):
        text = "[{created}] [{name}] {message}".format(entry.record.__dict__)
        pyperclip.copy(text)

        self.sub_title = "Copied to clipboard!"

        async def clear_notification():
            await asyncio.sleep(3.0)
            self.sub_title = self.SUB_TITLE

        self.call_later(clear_notification())

    def _get_next_node_key(self):
        num_items = self.sidebar.children
        if num_items < 10:
            # Node number starting at 0
            return str(num_items)
        if num_items < 36:
            # Next lowercase character
            return chr(97 + num_items - 10)

        return None

    def _on_logging_event(self, record):
        # This will be called by our logging handler

        if record.name not in self.nodes:
            keybind = self._get_next_node_key()
            node = NodeStatus(record.name)
            self.nodes[record.name] = node
            self.sidebar.append(node)

            if keybind:
                self.bind(keybind, "open_node_menu", description=record.name)
            
        if self.nodes[record.name].mute:
            # Logger is muted, message will not be recorded
            return

        if self.paused:
            # Save the raw messages and add them once we get unmuted
            self.backlog.append(record)
        else:
            self._log(record)

    def _log(self, *records):
        self.logview.extend(ListItem([LogEntry(r) for r in records]))

        # restrict number of list items
        num_entries = len(self.logview.children)
        if num_entries > self.max_log_length:
            self.logview.remove_items(range(0, num_entries - self.max_log_length + 1))

        if not self.logview.is_vertical_scrollbar_grabbed:
            self.logview.scroll_end()

    def action_toggle_sidebar(self):
        self.sidebar.display = not self.sidebar.display

    def action_mute_all(self):
        if not self.paused:
            self.paused = True
        else:
            # set all loggers disabled
            for node in self.nodes:
                node.mute = True

    def action_unmute_all(self):
        if self.paused:
            # Add messages we missed while muted
            if self.backlog:
                self._log(*self.backlog)
                self.backlog.clear()
            self.paused = False
        else:
            # set all loggers enabled
            for node in self.nodes:
                node.mute = False

    def action_search_node(self):
        # TODO open search dialog
        pass

    def action_close_node_menu(self):
        self.node_menu.display = False


if __name__ == "__main__":
    BetterLaunchUI().run()
