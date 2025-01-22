import os
import time
import asyncio
import pyperclip

from textual import work
from textual.app import App, ComposeResult
from textual.widgets import ListView, ListItem, Header, Footer 
from textual.containers import Horizontal
from textual.binding import Binding
from textual.reactive import reactive

from ros2node.api import get_node_names

from ros import logging as roslog
from ros.ros_adapter import ROSAdapter

from ui import NodeStatus, NodeMenu, LogEntry, TextualLogHandler


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
