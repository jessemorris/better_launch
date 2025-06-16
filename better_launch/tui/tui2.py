from typing import Literal
from enum import IntEnum, Enum, auto
import logging

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

from footer_menu import FooterMenu


class NodeAutoSuggest(AutoSuggest):
    def get_suggestion(self, buffer, document):
        history = buffer.history
        # Consider only the last line for the suggestion.
        text = document.text.rsplit("\n", 1)[-1]

        if text.strip():
            if text.startswith("/"):
                # TODO search nodes based on full name
                full_node_name = "/namespace/abc"
                if full_node_name.startswith(text):
                    return Suggestion(full_node_name[len(text) :])
            else:
                # TODO Search based on node names
                node_name = "abc"
                if node_name.startswith(text):
                    return Suggestion(node_name[len(text) :])

        return None


class AppMode(IntEnum):
    STANDARD = auto()
    CONFIRM_EXIT = auto()
    SEARCH_NODE = auto()
    NODE_MENU = auto()
    LOG_LEVEL = auto()


_log_levels = [
    ("ansibrightgreen", "INFO"),
    ("yellow", "WARNING"),
    ("ansibrightred", "ERROR"),
    ("ansibrightmagenta", "CRITICAL"),
    ("ansibrightblue", "DEBUG"),
]


class BetterTui:
    def __init__(self, *, color_depth: Literal[1, 4, 8, 24] = 8):
        self._color_depth = {
            1: ColorDepth.DEPTH_1_BIT,
            4: ColorDepth.DEPTH_4_BIT,
            8: ColorDepth.DEPTH_8_BIT,
            24: ColorDepth.DEPTH_24_BIT,
        }[color_depth]

        self.history = InMemoryHistory()
        self.bindings = KeyBindings()

        self.mode = AppMode.STANDARD
        # TODO remove test values
        self.nodes_snapshot: list[str] = [
            "node1",
            "node2",
            "another_node",
            "robert",
            "günter",
            "hildegard",
            "walter",
            "freud",
            "johnny keats",
            "gladstone",
            "sol weintraub",
        ]
        self.selected_node: str = None

        self.log_level = 0
        self.muted = False

        self.title: FormattedTextControl = None
        self.footer_window: Window = None
        self.search_field: TextArea = None
        self.search_buffer: Buffer = None
        self.footer_menu: FooterMenu = None

        self._setup_bindings()

    def run(self):
        layout = self._make_layout()
        app = Application(
            layout=layout,
            key_bindings=self.bindings,
            full_screen=False,
            color_depth=self._color_depth,
        )

        with patch_stdout():
            app.run()

    def quit(self) -> None:
        # TODO shutdown nodes
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

    def _menu_cancel(self) -> None:
        self.mode = AppMode.STANDARD
        get_app().layout.focus(self.footer_window)

    def _handle_menu_accept(self, idx: int, answer: str) -> None:
        if self.mode == AppMode.CONFIRM_EXIT:
            if answer == "yes":
                self.quit()
                return

        elif self.mode == AppMode.SEARCH_NODE:
            self.mode = AppMode.NODE_MENU
            # TODO get proper node reference
            self.selected_node = self.footer_menu.get_selected_item()
            # TODO items depend on node type
            self.footer_menu.set_items([
                "info",
                "restart",
                "kill",
            ])

        elif self.mode == AppMode.NODE_MENU:
            # TODO do something with the node
            pass

        elif self.mode == AppMode.LOG_LEVEL:
            # TODO configure logger
            self.log_level = idx

        self._menu_cancel()

    # TUI setup
    def _setup_bindings(self) -> None:
        bind = self.bindings.add

        menu_visible = Condition(self._is_menu_visible)

        @bind("c-c")
        async def _(event: KeyPressEvent):
            self.mode = AppMode.CONFIRM_EXIT
            self.footer_menu.set_items(["yes", "no"])
            event.app.invalidate()

        @bind("space", filter=~Condition(self._is_search_visible))
        def _(event: KeyPressEvent):
            # TODO configure our logger or stdout
            self.muted = not self.muted

        @bind("f1")
        def _(event: KeyPressEvent):
            if self.mode == AppMode.SEARCH_NODE:
                return

            self.mode = AppMode.SEARCH_NODE
            self.footer_menu.set_items(self.nodes_snapshot)
            self.search_buffer.text = ""
            event.app.layout.focus(self.search_field)

        @bind("f9")
        def _(event: KeyPressEvent):
            if self.mode == AppMode.LOG_LEVEL:
                return

            self.mode = AppMode.LOG_LEVEL
            self.footer_menu.set_items(_log_levels, self.log_level)

        # Menu interactions
        @bind("escape", filter=menu_visible, eager=True)
        def _(event: KeyPressEvent):
            self._menu_cancel()

        @bind("enter", filter=menu_visible)
        def _(event: KeyPressEvent):
            if not self.footer_menu.items:
                self._menu_cancel()

            else:
                answer = self.footer_menu.get_selected_item()
                self._handle_menu_accept(self.footer_menu.selected, answer)

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
                answer = self.footer_menu.get_selected_item()
                self._handle_menu_accept(self.footer_menu.selected, answer)

    def _make_layout(self) -> Layout:

        def on_search_update(_) -> None:
            new_text = self.search_buffer.text
            matches = [x for x in self.nodes_snapshot if new_text.lower() in x.lower()]
            self.footer_menu.update_items(matches)

        self.title = FormattedTextControl("")
        self.footer_window = Window(FormattedTextControl(self._get_toolbar))

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

    def _get_toolbar(self) -> str:
        if self.mode == AppMode.CONFIRM_EXIT:
            return "Shutdown nodes and quit?"

        elif self.mode == AppMode.SEARCH_NODE:
            # Not shown
            return ""

        elif self.mode == AppMode.NODE_MENU:
            # TODO get full node name
            return self.selected_node

        elif self.mode == AppMode.LOG_LEVEL:
            return "Select log level"

        # AppMode.STANDARD
        return " [^C] Quit | [space] Mute | [F1] Find  [F9] Log Level"


if __name__ == "__main__":
    from threading import Thread
    import time

    def gen():
        i = 0
        while True:
            i += 1
            print(i)
            time.sleep(1)

    t = Thread(target=gen, daemon=True)
    t.start()

    tui = BetterTui()
    tui.run()
