from typing import Callable, cast
import os
import logging
import pyperclip

from textual import work
from textual.app import App, ComposeResult
from textual.widgets import ListView, ListItem, Header, Footer
from textual.containers import Horizontal
from textual.binding import Binding
from textual.reactive import reactive

from ros2node.api import get_node_names

from better_launch import BetterLaunch
import ros.logging as roslog
from utils.better_logging import LogRecordForwarder

from elements import Node, LifecycleNode, Composer, LifecycleStage
from .log_entry import LogEntry
from .node_menu import NodeLabel, NodeInfoScreen
from .choice_dialog import ChoiceDialog


# Inspired by xqms' rosmon: https://github.com/xqms/rosmon
class BetterUI(App):
    TITLE = "BetterLaunch"

    BINDINGS = [
        Binding("ctrl+q", "quit", "Quit"),
        Binding("space", "toggle_mute", "Mute"),
        Binding("f1", "toggle_sidebar", "Sidebar"),
        Binding("f2", "toggle_log_names", "Names"),
        Binding("f3", "toggle_log_icons", "Icons"),
    ]

    DEFAULT_CSS = """
        #sidebar {
            width: 20%;
            min_width: 20;
            max_width: 40%;
            height: 1fr;
            background: $panel;

            .-highlight {
                text-style: italic bold;
            }
        }
        #logview {
            width: 1fr;
            height: 1fr;

            LogEntry {
                #source {
                    width: 20;
                    height: 1;
                    text-align: right;
                }
                #icon {
                    width: 3;
                    height: 1;
                    text-align: center;
                }
                #message {
                    width: 100%;
                    text-align: left;
                    padding-left: 1;
                }
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
        self.exit_reason = ""

        self.backlog = []
        self.max_log_length = max_log_length
        self.node_keymap = {}

        self.mute = False
        self.show_log_sources = True
        self.show_log_icons = True

        self.launch_func = launch_func

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        # Could use a Tree view instead
        self.sidebar = ListView(id="sidebar")
        # TODO OptionList is more performant
        self.logview = ListView(id="logview")
        self.logview.can_focus = False

        with Horizontal():
            yield self.sidebar
            yield self.logview

        yield Footer(show_command_palette=False)

    def on_mount(self):
        self.run_launch_function()

    def exit(self, reason: str = "UI terminated"):
        # This might be invoked more than once
        if not self.exit_reason:
            self.exit_reason = reason

        super().exit()

        bl = BetterLaunch.instance()
        if bl and not bl.is_shutdown:
            bl.shutdown(reason)

    async def _shutdown(self):
        # Passing a message to super.exit will be treated as an error, so we instead handle any
        # post-shutdown stuff here
        await super()._shutdown()

        if self.exit_reason:
            from rich.console import Console

            Console().print(f"[bright_green]BetterUI exit: {self.exit_reason}[/bright_green]")

    @work(thread=True, exit_on_error=True, group="launch_func")
    def run_launch_function(self):
        def log_to_ui(record: logging.LogRecord):
            if self.is_running:
                self.call_later(self.on_log_record, record)

        log_handler = roslog.launch_config.screen_handler
        if not isinstance(log_handler, LogRecordForwarder):
            raise RuntimeError(
                "Something modified the logging handler, UI cant't start"
            )

        log_handler.add_listener(log_to_ui)
        self.launch_func()

        bl = BetterLaunch.wait_for_instance()
        self.sub_title = os.path.basename(bl._launchfile)
        self.call_later(self.add_nodes_to_sidebar)
        bl.spin()
        self.exit("BetterLaunch terminated")

    def add_nodes_to_sidebar(self):
        bl = BetterLaunch.wait_for_instance()
        for idx, n in enumerate(bl.all_nodes()):
            key = self._get_node_key(idx)
            item = ListItem(NodeLabel(n, key))
            if key:
                self.node_keymap[key] = idx
            self.sidebar.append(item)

    def _get_node_key(self, idx: int):
        if idx < 10:
            # Node number starting at 1
            return str(idx + 1)
        if idx == 10:
            # Easier for the user
            return 0
        if idx < 36:
            # Next lowercase character
            return chr(97 + idx - 10)

        return None

    def on_key(self, event):
        idx = self.node_keymap.get(event.key)
        if idx is not None:
            # Select the node and open its menu (see open_menu_for_node)
            self.sidebar.index = idx
            self.sidebar.action_select_cursor()

    def on_log_record(self, record: logging.LogRecord):
        # This will be called by our logging handler
        if self.mute:
            # Save the raw messages and add them once we get unmuted
            self.backlog.append(record)
        else:
            self._log(record)

    def _log(self, *records):
        if self._exit:
            return
        
        self.logview.extend([ListItem(LogEntry(r)) for r in records])

        # restrict number of list items
        num_entries = len(self.logview.children)
        if num_entries > self.max_log_length:
            self.logview.remove_items(range(0, num_entries - self.max_log_length + 1))

        if not self.logview.is_vertical_scrollbar_grabbed:
            self.logview.scroll_end()

    def on_list_view_selected(self, selected: ListView.Selected):
        if selected.list_view == self.sidebar:
            self.open_menu_for_node(selected.item.get_child_by_type(NodeLabel))
        elif selected.list_view == self.logview:
            self.copy_log_entry(selected.item.get_child_by_type(LogEntry))

    def open_menu_for_node(self, label: NodeLabel):
        node = label.node

        def on_lifecycle_choice(choice: str):
            cast(LifecycleNode, node).transition(LifecycleStage[choice.upper()])

        def on_kill_choice(choice: str):
            if choice == "yes":
                node.shutdown("Shutdown by user")

        def on_node_menu_choice(action: str = None):
            if action == "info":
                self.push_screen(NodeInfoScreen(node))

            elif action == "lifecycle":
                valid_stages = list(LifecycleStage)
                valid_stages.remove(node.current_stage)
                self.push_screen(
                    ChoiceDialog(valid_stages, f"Transition {node.name} to"),
                    on_lifecycle_choice,
                )

            elif action == "components":
                # TODO should provide info on each component like subscribers and publishers
                self.notify("components not implemented yet", timeout=2.0)

            elif action == "kill":
                self.push_screen(
                    ChoiceDialog(["yes", "cancel"], f"Kill {node.name}?"),
                    on_kill_choice,
                )

        if isinstance(node, LifecycleNode):
            choices = ["info", "lifecycle", "kill"]
        elif isinstance(node, Composer):
            choices = ["info", "components", "kill"]
        else:
            choices = ["info", "kill"]

        title = f"{node.name} ({node.__class__.__name__})"
        self.push_screen(ChoiceDialog(choices, title), on_node_menu_choice)

    def copy_log_entry(self, entry: LogEntry):
        if pyperclip.is_available():
            text = "[{created}] [{name}] {message}".format(**entry.record.__dict__)
            pyperclip.copy(text)
            self.notify("Copied to clipboard!", timeout=2.0)
        else:
            self.notify(
                "pyperclip failed, see documentation", severity="error", timeout=3.0
            )

        self.logview.index = None

    def action_toggle_sidebar(self):
        self.sidebar.display = not self.sidebar.display

    def action_toggle_log_names(self):
        show = not self.show_log_sources
        self.show_log_sources = show

        if show:
            self.stylesheet.add_source(
                "#logview #source {display: block;}", read_from="toggle_log_names"
            )
        else:
            self.stylesheet.add_source(
                "#logview #source {display: none;}", read_from="toggle_log_names"
            )

        self.refresh_css()

    def action_toggle_log_icons(self):
        show = not self.show_log_sources
        self.show_log_sources = show

        if show:
            self.stylesheet.add_source(
                "#logview #icon {display: block;}", read_from="toggle_log_icons"
            )
        else:
            self.stylesheet.add_source(
                "#logview #icon {display: none;}", read_from="toggle_log_icons"
            )

        self.refresh_css()

    def action_toggle_mute(self):
        if self.mute:
            # Add messages we missed while muted
            if self.backlog:
                self._log(*self.backlog)
                self.backlog.clear()
            self.mute = False
        else:
            self.mute = True

    def action_close_node_menu(self):
        self.node_menu.display = False

    def action_quit(self):
        def on_quit_choice(reply: str):
            if reply == "yes":
                self.exit("terminated by user")

        self.push_screen(
            ChoiceDialog(["yes", "no"], "Quit launcher and terminate all nodes?"),
            on_quit_choice,
        )
