from typing import Callable
import os
import time
import logging
import threading
from collections import deque
import pytermgui as ptg
import pyperclip

from better_launch import BetterLaunch
import ros.logging as roslog
from utils.better_logging import LogRecordForwarder


class LogEntry(ptg.Splitter):
    styles = ptg.StyleManager.merge(
        ptg.Container.styles,
        debug="grey",
        info="blue",
        warning="yellow",
        error="red",
        critical="purple bold",
    )

    chars = {
        "separator": " ",
        "debug": "⚑",  # "›»§🔍",
        "info": "✓",  # "●@#i🏷️",
        "warning": "▲",  # "🚧",
        "error": "⨯",  # "✗!🛑",
        "critical": "🔥",  # "🔥⚡",
    }

    def __init__(self, record: logging.LogRecord):
        self.record = record

        level = record.levelname.lower()
        style = self.styles[level]
        icon = style(self._get_char(level))

        source_label = ptg.Label(
            record.name + ": ", parent_align=ptg.HorizontalAlignment.RIGHT
        )
        source_label.relative_width = 0.2

        icon_label = ptg.Label(icon)
        icon_label.static_width = 1

        message_label = ptg.Label(record.msg)

        super().__init__(source_label, icon_label, message_label)


class LogView(ptg.Window):
    def __init__(self, max_lines: int = 1000, **kwargs):
        super().__init__(**kwargs)
        # Sneakily replace our widget list with a double ended queue
        self._widgets = deque(maxlen=max_lines)

    def on_log_record(self, record: logging.LogRecord):
        # TODO is it okay to defer the update?
        # TODO implement muting
        self._add_widget(LogEntry(record), run_get_lines=True)

    def copy_to_clipboard(self):
        if not self.selected:
            return

        record = self.selected.record
        text = "[{created}] [{name}] {message}".format(record.__dict__)
        pyperclip.copy(text)
        
        self.manager.toast(f"[blue]Copied to clipboard[/blue]", delay=700)

    def handle_key(self, key: str) -> bool:
        if super().handle_key(key):
            return True

        if key == ptg.Keys.ENTER:
            self.copy_to_clipboard()
            return True

        return False
        
    def on_left_click(self, event: ptg.MouseEvent) -> bool:
        self.copy_to_clipboard()
        return True


class NodesList(ptg.Window):
    def __init__(self, nodes: list["Node"], **kwargs):
        super().__init__(**kwargs)
        self.nodes = nodes

        self.bind("i", self._show_node_info)
        self.bind("k", self._confirm_node_kill)

        self.set_widgets(ptg.Label(node.name) for node in nodes)
        self._add_widget(self._create_footer())
        self.center(store=False)

    def _create_footer(self):
        # TODO
        pass

    def _show_node_info(self):
        if self.selected_index is None:
            return

        node = self.nodes[self.selected_index]

        # ROS2 prints a lot of useless stuff and avoids the things that are interesting most of
        # the time, like who is actually subscribed where. Let's fix this!
        shared_node = BetterLaunch.wait_for_instance().shared_node
        
        pubs = shared_node.get_publisher_names_and_types_by_node(node.name, node.namespace)
        pubs.sort()
        pubs_text = ""
        for topic, types in pubs:
            pubs_text += f"\n  {topic} [{', '.join(types)}]"

        subs = shared_node.get_subscriber_names_and_types_by_node(node.name, node.namespace)
        subs.sort()
        subs_text = ""
        for topic, types in subs:
            subs_text += f"\n  {topic} [{', '.join(types)}]"

        info_text = f"""\
[bold]{node.name}[/bold]
Status:    {'[green]alive[/green]' if node.is_running else '[red]dead[red]'}
Namespace: {node.namespace}

[bold]Process:[/bold]
  PID:     {node.pid}
  Command: {node.cmd}
  Args:    {node.node_args}
  Env:     {node.env}

[bold]Publishers:[/bold] {pubs_text}

[bold]Subscriptions:[/bold] {subs_text}
"""

        def done():
            info.close()
            self.close()

        info = ptg.Window(
            ptg.Label(info_text),
            ptg.Button("Okay", done),
            is_modal = True,
        )
        info.bind(ptg.keys.ESC, info.close)
        info.center()
        self.manager.add(info, assign=False)

    def _confirm_node_kill(self):
        if self.selected_index is None:
            return

        node = self.nodes[self.selected_index]
        
        def kill_node():
            node.shutdown("Terminated by UI")
            self.manager.toast(f"[blue]Node {node.name} killed[/blue]")
            self.close()

        confirm = ptg.Window(
            ptg.Label(f"Are you sure you want to kill this node?\n{node.full_name}"),
            ptg.Splitter(
                ptg.Button("Kill", kill_node),
                ptg.Button("Cancel", lambda: confirm.close()),
            ),
            is_modal = True,
        )
        confirm.bind(ptg.keys.ESC, confirm.close)
        confirm.center()
        self.manager.add(confirm, assign=False)


class BetterUI:
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
        self.launch_thread = None

    def start(self, launch_func: Callable, *args, **kwargs):
        self._ptg_init()

        with ptg.WindowManager(autorun=False) as wm:
            wm.bind(
                ptg.keys.ESC,
                lambda *_: self._close_focused(wm),
                # For closing windows
            )
            wm.bind(
                ptg.keys.CTRL_Q,
                lambda *_: self._quit(wm),
                "Quit",
            )
            wm.bind(
                ptg.keys.F1,
                lambda *_: self._open_nodes_list(wm),
                "Nodes",
            )
            wm.bind(
                ptg.keys.F9,
                lambda *_: self._toggle_mute(wm),
                "Toggle mute",
            )

            wm.layout = self._create_layout()

            wm.add(self._create_header(wm), assign="header", animate=False)
            wm.add(self._create_body(wm), assign="body", animate=False)
            wm.add(self._create_footer(wm), assign="footer", animate=False)

            wm.toast(
                "[bl.title]Welcome to [/bl.title bl.brand_title]"
                + "BetterLaunch[/bl.brand_title bl.title]!",
                offset=ptg.terminal.height // 2 - 3,
                delay=700,
            )

        # Run the launch function in a background thread
        self.launch_thread = threading.Thread(target=launch_func, args=args, kwargs=kwargs)
        self.launch_thread.start()

        # Start the UI loop
        # Make sure we wait until the user has created the singleton
        bl = BetterLaunch.wait_for_instance()
        bl.add_shutdown_callback(wm.stop)
        
        wm.run()

        if not bl.is_shutdown():
            bl.shutdown("UI terminated")

    def _close_focused(self, wm: ptg.WindowManager) -> None:
        if wm.focused is None:
            return

        # Find foremost non-persistent window
        for window in wm:
            if not window.is_persistent:
                window.close()
                return

    def _quit(self, wm: ptg.WindowManager) -> None:
        wm.stop()

    def _open_nodes_list(self, wm: ptg.WindowManager) -> None:
        # This is NOT 'ros2 node kill' as we only manage our own nodes
        # TODO should probably include components, too?
        nodes = BetterLaunch.wait_for_instance().all_nodes()

        menu = NodesList(nodes)
        #menu.center()
        menu.bind(ptg.keys.ESC, menu.close)
        wm.add(menu, assign=False)

    def _toggle_mute(self, wm: ptg.WindowManager) -> None:
        # TODO
        pass

    def _ptg_init(self):
        # Borders and such
        ptg.boxes.Box([" ", " x ", " "]).set_chars_of(ptg.Window)
        ptg.boxes.SINGLE.set_chars_of(ptg.Container)
        ptg.boxes.DOUBLE.set_chars_of(ptg.Window)
        ptg.Splitter.set_char("separator", "")
        ptg.Button.set_char("delimiter", [" ", " "])

        # Style aliases
        ptg.tim.alias("bl.title", "secondary bold")
        ptg.tim.alias("bl.brand_title", "!gradient(210) bold")
        ptg.tim.alias("bl.body", "surface+3")
        ptg.tim.alias("bl.detail", "surface+2")
        ptg.tim.alias("bl.accent", "primary")
        ptg.tim.alias("bl.header", "bold @surface-2 surface+1")
        ptg.tim.alias("bl.footer", "@surface-2 surface+1")

    def _create_layout(self):
        layout = ptg.Layout()

        layout.add_slot("Header", height=1)
        layout.add_break()
        layout.add_slot("Body")
        layout.add_break()
        layout.add_slot("Footer", height=1)

        return layout

    def _create_header(self, wm: ptg.WindowManager):
        # TODO add a time https://ptg.bczsalba.com/tim/usage/#define
        content = ptg.Splitter(
            ptg.Label("BetterLaunch", parent_align=ptg.HorizontalAlignment.LEFT),
            ptg.Label("time-goes-here", parent_align=ptg.HorizontalAlignment.RIGHT),
        )
        content.styles.fill = "bl.header"

        return ptg.Window(content, box="EMPTY", id="bl.header", is_persistent=True)

    def _create_body(self, wm: ptg.WindowManager):
        content = LogView(self.max_log_length)

        log_handler = roslog.launch_config.screen_handler
        if not isinstance(log_handler, LogRecordForwarder):
            raise RuntimeError(
                "Something modified the logging handler, UI cant't start"
            )
        log_handler.add_listener(content.on_log_record)

        return ptg.Window(content, box="EMPTY", id="bl.log", is_persistent=True)

    def _create_footer(self, wm: ptg.WindowManager):
        content = ptg.Splitter().styles(fill="bl.footer")
        for key, (callback, description) in wm.bindings.items():
            if not description or description == f"Binding of {key} to {callback}":
                continue

            keyname = ptg.keys.get_name(str(key))
            if not keyname:
                keyname = ascii(str(key))

            content.lazy_add(
                ptg.Button(
                    f"{keyname} - {description}",
                    onclick=lambda *_, _callback=callback: _callback(wm),
                    parent_align=ptg.HorizontalAlignment.LEFT,
                )
            )

        return ptg.Window(content, box="EMPTY", id="bl.footer", is_persistent=True)
