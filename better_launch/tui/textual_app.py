from typing import Callable
import os
import time
import logging
import asyncio
import pyperclip

from textual import work
from textual.app import App, ComposeResult
from textual.widgets import ListView, ListItem, Header, Footer, DataTable
from textual.containers import Horizontal
from textual.binding import Binding
from textual.reactive import reactive

from ros2node.api import get_node_names

from ros.ros_adapter import ROSAdapter
import ros.logging as roslog
from utils.better_logging import LogRecordForwarder

from .log_entry import LogEntry
from .node_menu import NodeMenu
from .node_search import NodeSearch
from .node_status import NodeStatus, NodeStatusLabel


# Inspired by xqms' rosmon: https://github.com/xqms/rosmon
class BetterUI(App):
    TITLE = "BetterLaunch"

    BINDINGS = [
        Binding("f1", "toggle_sidebar", "Nodes"),
        Binding("f9", "toggle_mute", "Mute/Unmute"),
        Binding("/", "search_node", "Node search"),
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

    @classmethod
    def setup_logging(cls):
        # Make sure all ros loggers follow a parsable format
        os.environ["RCUTILS_COLORIZED_OUTPUT"] = "0"
        os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "%%{severity}%%{time}%%{message}"

        if "OVERRIDE_LAUNCH_SCREEN_FORMAT" in os.environ:
            del os.environ["OVERRIDE_LAUNCH_SCREEN_FORMAT"]

        # Install the log handler
        roslog.launch_config.screen_handler = LogRecordForwarder()

    def __init__(
        self,
        launch_func: Callable, 
        disable_colors: bool = False,
        max_log_length: bool = 1000,
    ):
        if disable_colors:
            os.environ["NO_COLOR"] = "1"
        
        super().__init__()

        self.nodes = {}
        self.backlog = []
        self.mute = False
        self.max_log_length = max_log_length

        self.launch_func = launch_func

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
        self.run_launch_function()

    def on_exit(self):
        #self.ros_adapter.shutdown()
        print("UI terminating")

    @work(thread=True)
    def run_launch_function(self):
        def log_to_ui(record: logging.LogRecord):
            if self.is_running:
                self.call_later(self.on_log_record, record)

        log_handler = roslog.launch_config.screen_handler
        if not isinstance(log_handler, LogRecordForwarder):
            raise RuntimeError("Something modified the logging handler, UI cant't start")
        
        log_handler.add_listener(log_to_ui)
        self.launch_func()

        # TODO join ros_adapter thread, call self.exit, call bl.shutdown

    # TODO thread or asyncio? fine for now
    @work(thread=True)
    def check_nodes_status(self):
        live_nodes = set(
            n.full_name for n in get_node_names(node=self.ros_adapter.ros_node)
        )

        for item in self.sidebar.children:
            if not isinstance(item, NodeStatusLabel):
                continue

            # TODO ignoring lifecycle node status for now
            node = item.node_details
            if node.fullname in live_nodes:
                if node.has_errors:
                    item.status = NodeStatus.ERROR
                else:
                    item.status = NodeStatus.ALIVE
            else:
                item.status = NodeStatus.DEAD

        time.sleep(1.0)

    def on_list_view_selected(self, selected: ListView.Selected):
        if selected.list_view == self.sidebar:
            self.open_menu_for_node(selected.item.get_child_by_type(NodeStatusLabel))
        elif selected.list_view == self.logview:
            self.copy_log_entry(selected.item.get_child_by_type(LogEntry))
    
    def open_menu_for_node(self, node: NodeStatusLabel):
        if not self.sidebar.display:
            self.action_toggle_sidebar()
        
        # TODO add new bindings for menu commands while menu is open
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
        num_items = len(self.sidebar.children)
        if num_items < 10:
            # Node number starting at 0
            return str(num_items)
        if num_items < 36:
            # Next lowercase character
            return chr(97 + num_items - 10)

        return None

    def on_log_record(self, record):
        # This will be called by our logging handler

        if record.name not in self.nodes:
            keybind = self._get_next_node_key()
            node = NodeStatusLabel(keybind, NodeStatus(record.name))
            self.nodes[record.name] = node
            self.sidebar.append(node)

            if keybind:
                self.bind(keybind, "open_node_menu", description=record.name)
            
        if self.nodes[record.name].node_details.mute:
            # Logger is muted, message will not be recorded
            return

        if self.mute:
            # Save the raw messages and add them once we get unmuted
            self.backlog.append(record)
        else:
            self._log(record)

    def _log(self, *records):
        # TODO wrap lines
        self.logview.extend([ListItem(LogEntry(r)) for r in records])

        # restrict number of list items
        num_entries = len(self.logview.children)
        if num_entries > self.max_log_length:
            self.logview.remove_items(range(0, num_entries - self.max_log_length + 1))

        if not self.logview.is_vertical_scrollbar_grabbed:
            self.logview.scroll_end()

    def action_toggle_sidebar(self):
        self.sidebar.display = not self.sidebar.display

    def action_toggle_mute(self):
        if self.mute:
            # Add messages we missed while muted
            if self.backlog:
                self._log(*self.backlog)
                self.backlog.clear()
            self.mute = False
        else:
            self.mute = True

    def action_search_node(self):
        # TODO open search dialog
        pass

    def action_close_node_menu(self):
        self.node_menu.display = False
