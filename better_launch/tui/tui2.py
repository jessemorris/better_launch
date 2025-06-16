import os
from typing import Literal, Callable
from enum import IntEnum, auto
import logging
import threading
from dataclasses import dataclass

from prompt_toolkit import Application
from prompt_toolkit.output.color_depth import ColorDepth
from prompt_toolkit.application.current import get_app
from prompt_toolkit.layout.containers import HSplit, Window, ConditionalContainer
from prompt_toolkit.layout.controls import FormattedTextControl
from prompt_toolkit.layout.layout import Layout
from prompt_toolkit.filters import Condition
from prompt_toolkit.key_binding import KeyBindings, KeyPressEvent
from prompt_toolkit.widgets import TextArea
from prompt_toolkit.auto_suggest import AutoSuggest, Suggestion
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.buffer import Buffer
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.shortcuts import set_title

from better_launch import BetterLaunch
from better_launch.elements import (
    AbstractNode,
    ForeignNode,
    Component as ComponentNode,
    discover_ros2_nodes,
    LifecycleStage,
)
import better_launch.ros.logging as roslog

from better_launch.tui.footer_menu import FooterMenu


class AppMode(IntEnum):
    STANDARD = auto()
    CONFIRM_EXIT = auto()
    SEARCH_NODE = auto()
    NODE_MENU = auto()
    NODE_INFO = auto()
    NODE_LIFECYCLE = auto()
    CONFIRM_NODE_TAKEOVER = auto()
    CONFIRM_NODE_RESTART = auto()
    CONFIRM_NODE_KILL = auto()
    LOG_LEVEL = auto()


@dataclass
class LogLevel:
    name: str
    level: int
    style: str


_log_levels = {
    "INFO": LogLevel("INFO", logging.INFO, "ansibrightgreen"),
    "WARNING": LogLevel("WARNING", logging.WARNING, "yellow"),
    "ERROR": LogLevel("ERROR", logging.ERROR, "ansibrightred"),
    "CRITICAL": LogLevel("CRITICAL", logging.CRITICAL, "ansibrightmagenta"),
    "DEBUG": LogLevel("DEBUG", logging.DEBUG, "ansibrightblue"),
    "MUTE": LogLevel("MUTE", 999, "grey"),
}


logging.addLevelName(999, "MUTE")


class BetterTui:
    def __init__(
        self,
        launch_func: Callable,
        *,
        manage_foreign_nodes: bool = False,
        color_depth: Literal[1, 4, 8, 24] = 8
    ):
        self.launch_func = launch_func
        self.manage_foreign_nodes = manage_foreign_nodes
        self.color_depth = {
            1: ColorDepth.DEPTH_1_BIT,
            4: ColorDepth.DEPTH_4_BIT,
            8: ColorDepth.DEPTH_8_BIT,
            24: ColorDepth.DEPTH_24_BIT,
        }[color_depth]

        self.history = InMemoryHistory()
        self.bindings = KeyBindings()

        self.mode = AppMode.STANDARD
        self.footer_text: str = ""
        self.nodes_snapshot: list[AbstractNode] = []
        self.selected_node: AbstractNode = None

        self.prev_log_level = _log_levels["MUTE"]
        self.log_level = _log_levels["INFO"]
        self.muted = False

        self.title: FormattedTextControl = None
        self.footer_window: Window = None
        self.search_field: TextArea = None
        self.search_buffer: Buffer = None
        self.footer_menu: FooterMenu = None

        self._setup_key_bindings()

    def run(self):
        layout = self._make_layout()
        self._switch_mode(AppMode.STANDARD)
        app = Application(
            layout=layout,
            key_bindings=self.bindings,
            full_screen=False,
            color_depth=self.color_depth,
        )

        def _run_launch_func(self) -> None:
            """Runs the launch function passed to the constructor and sets up the log capturing mechanism."""
            self._launch_func()

            bl = BetterLaunch.wait_for_instance()
            set_title(os.path.basename(bl.launchfile))
            bl.spin()
            self.quit()

        launch_thread = threading.Thread(target=_run_launch_func)

        with patch_stdout():
            launch_thread.start()
            app.run()

    def quit(self, reason: str) -> None:
        bl = BetterLaunch.instance()
        if bl:
            bl.shutdown(reason)
        get_app().exit()

    # Some common helpers
    def _is_footer_visible(self) -> bool:
        return self.mode not in (AppMode.SEARCH_NODE,)

    def _is_search_visible(self) -> bool:
        return self.mode in (AppMode.SEARCH_NODE,)

    def _is_menu_visible(self) -> bool:
        return self.mode in (
            AppMode.CONFIRM_EXIT,
            AppMode.SEARCH_NODE,
            AppMode.NODE_MENU,
            AppMode.LOG_LEVEL,
        )

    def set_log_level(self, level: LogLevel) -> None:
        self.prev_log_level = self.log_level
        self.log_level = level
        handler: logging.Handler = roslog.launch_config.get_screen_handler()
        handler.setLevel(level.level)

    def _menu_cancel(self) -> None:
        self.mode = AppMode.STANDARD
        get_app().layout.focus(self.footer_window)

    # Setup user interactions
    def _setup_key_bindings(self) -> None:
        bind = self.bindings.add

        mode_standard = Condition(lambda: self.mode == AppMode.STANDARD)
        menu_visible = Condition(self._is_menu_visible)

        @bind("c-c")
        async def _(event: KeyPressEvent):
            self._switch_mode(AppMode.CONFIRM_EXIT)

        @bind("space", filter=~Condition(self._is_search_visible))
        def _(event: KeyPressEvent):
            self.muted = not self.muted
            level = _log_levels["MUTE"] if self.muted else self.prev_log_level
            self.set_log_level(level)

        @bind("f1", filter=mode_standard)
        def _(event: KeyPressEvent):
            self._switch_mode(AppMode.SEARCH_NODE)

        @bind("f9", filter=mode_standard)
        def _(event: KeyPressEvent):
            self._switch_mode(AppMode.LOG_LEVEL)

        # Menu interactions
        @bind("escape", filter=menu_visible, eager=True)
        def _(event: KeyPressEvent):
            self._switch_mode(AppMode.STANDARD)

        @bind("enter", filter=menu_visible)
        def _(event: KeyPressEvent):
            if not self.footer_menu.items:
                self._menu_cancel()
            else:
                self._handle_menu_accept(self.footer_menu.selected)

        @bind("tab", filter=menu_visible)
        def _(event: KeyPressEvent):
            self.footer_menu.select_next()

        @bind("s-tab", filter=menu_visible)
        def _(event: KeyPressEvent):
            self.footer_menu.select_prev()

        for i in range(10):
            has_item = lambda k=i: k < len(self.footer_menu.items)

            @bind(
                str(i),
                filter=menu_visible
                & ~Condition(self._is_search_visible)
                & Condition(has_item),
            )
            def _(event: KeyPressEvent):
                self.footer_menu.select((i - 1) % 10)
                self._handle_menu_accept(self.footer_menu.selected)

    def _switch_mode(self, mode: AppMode) -> None:
        self.mode = mode

        if mode == AppMode.STANDARD:
            self.footer_text = " [^C] Quit | [space] Mute | [F1] Find  [F9] Log Level"
            self._menu_cancel()

        elif mode == AppMode.CONFIRM_EXIT:
            self.footer_text = "Shutdown nodes and quit?"
            self.footer_menu.set_items(["yes", "no"])

        elif mode == AppMode.SEARCH_NODE:
            self.footer_text = ""

            if self.manage_foreign_nodes:
                self.nodes_snapshot = discover_ros2_nodes(True)
            else:
                bl = BetterLaunch.instance()
                self.nodes_snapshot = bl.all_nodes(
                    include_components=True, include_launch_service=True
                )

            items = [("", n.name, n) for n in self.nodes_snapshot]
            self.footer_menu.set_items(items)

            self.search_buffer.text = ""
            get_app().layout.focus(self.search_field)

        elif mode == AppMode.NODE_MENU:
            # Contains the format, node name, and a reference to the AbstractNode
            # (see SEARCH_NODE above)
            item = self.footer_menu.get_selected_item()
            node = item[-1]
            self.selected_node = node

            if isinstance(node, ForeignNode):
                choices = ["info", "takeover", "kill"]
            elif isinstance(node, ComponentNode):
                choices = ["info", "restart", "unload"]
            else:
                choices = ["info", "restart", "kill"]

            if node.check_lifecycle_node():
                choices.insert(1, "lifecycle")

            self.footer_text = node.fullname
            self.footer_menu.set_items(choices)

        elif mode == AppMode.NODE_INFO:
            # Simply print to our captured stdout
            cols = get_app().output.get_size().columns
            bar = "\n" + "=" * cols + "\n"
            text = self.selected_node.get_info_sheet()
            print(bar + text + bar)
            
            self._menu_cancel()

        elif mode == AppMode.NODE_LIFECYCLE:
            self.footer_text = "Choose target state for " + self.selected_node.fullname

            valid_stages = list([s.name for s in LifecycleStage])
            active = valid_stages.index(self.selected_node.lifecycle.current_stage.name)
            self.footer_menu.set_items(valid_stages, active)

        elif mode == AppMode.CONFIRM_NODE_TAKEOVER:
            self.footer_text = f"Restart {self.selected_node.fullname} for takeover?"
            self.footer_menu.set_items(["yes", "no"])

        elif mode == AppMode.CONFIRM_NODE_RESTART:
            self.footer_text = f"Restart {self.selected_node.fullname}?"
            self.footer_menu.set_items(["yes", "no"])

        elif mode == AppMode.CONFIRM_NODE_KILL:
            self.footer_text = f"Terminate {self.selected_node.fullname}?"
            self.footer_menu.set_items(["yes", "no"])

        elif mode == AppMode.LOG_LEVEL:
            self.footer_text = "Select log level"
            items = [(l.style, l.name) for l in _log_levels.values()]
            self.footer_menu.set_items(items, self.log_level)

    def _handle_menu_accept(self, idx: int) -> None:
        item = self.footer_menu.get_selected_item()

        if self.mode == AppMode.CONFIRM_EXIT:
            if item == "yes":
                self.quit("user request")
                return

        elif self.mode == AppMode.SEARCH_NODE:
            self._switch_mode(AppMode.NODE_MENU)

        elif self.mode == AppMode.NODE_MENU:
            action = self.footer_menu.get_selected_item()

            if action == "info":
                self._switch_mode(AppMode.NODE_INFO)

            elif action == "lifecycle":
                self._switch_mode(AppMode.NODE_LIFECYCLE)

            elif action == "takeover":
                self._switch_mode(AppMode.CONFIRM_NODE_TAKEOVER)

            elif action == "restart":
                self._switch_mode(AppMode.CONFIRM_NODE_RESTART)

            elif action in ("kill", "unload"):
                self._switch_mode(AppMode.CONFIRM_NODE_KILL)

        elif self.mode == AppMode.NODE_LIFECYCLE:
            target_stage = LifecycleStage[item]
            self.selected_node.lifecycle.transition(target_stage)

        elif self.mode == AppMode.CONFIRM_NODE_TAKEOVER:
            if item == "yes":
                self.selected_node.takeover(kill_after=3.0)

        elif self.mode == AppMode.CONFIRM_NODE_RESTART:
            if item == "yes":
                self.selected_node.shutdown("restarting node", timeout=None)
                self.selected_node.start()

        elif self.mode == AppMode.CONFIRM_NODE_KILL:
            if item == "yes":
                self.selected_node.shutdown("terminated by user")

        elif self.mode == AppMode.LOG_LEVEL:
            if isinstance(item, tuple):
                item = item[1]

            level = _log_levels[item]
            self.set_log_level(level)

        self._menu_cancel()

    def _make_layout(self) -> Layout:

        def on_search_update(_) -> None:
            new_text = self.search_buffer.text

            if new_text.startswith("/"):
                nodes = [n.fullname for n in self.nodes_snapshot]
            else:
                nodes = [n.name for n in self.nodes_snapshot]

            matches = [x for x in nodes if new_text.lower() in x.lower()]
            self.footer_menu.update_items(matches)

        self.title = FormattedTextControl("")
        self.footer_window = Window(FormattedTextControl(lambda: self.footer_text))

        self.search_field = TextArea(
            prompt="Search: ",
            height=1,
            multiline=False,
            wrap_lines=False,
        )
        self.search_buffer = self.search_field.buffer
        self.search_buffer.on_text_changed += on_search_update

        self.footer_menu = FooterMenu([])

        footer_visible = Condition(self._is_footer_visible)
        search_visible = Condition(self._is_search_visible)
        menu_visible = Condition(self._is_menu_visible)

        return Layout(
            HSplit(
                [
                    Window(self.title, height=1),
                    ConditionalContainer(
                        self.footer_window,
                        footer_visible,
                    ),
                    ConditionalContainer(
                        self.search_field,
                        search_visible,
                    ),
                    ConditionalContainer(
                        Window(self.footer_menu, height=1),
                        menu_visible,
                    ),
                ]
            )
        )
