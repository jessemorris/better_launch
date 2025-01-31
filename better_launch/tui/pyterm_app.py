from typing import Callable, cast
import os
import logging
import threading
from itertools import zip_longest
from collections import deque
import pytermgui as ptg
import pyperclip

from better_launch import BetterLaunch
import ros.logging as roslog
from utils.better_logging import LogRecordForwarder


class LogEntry(ptg.Splitter):
    styles = ptg.StyleManager.merge(
        ptg.Splitter.styles,
        debug="grey",
        info="blue",
        warning="yellow",
        error="red",
        critical="purple bold",
    )

    chars = {
        **ptg.Splitter.chars,
        **{
            "separator": " ",
            "debug": "⚑",  # "›»§🔍",
            "info": "✓",  # "●@#i🏷️",
            "warning": "▲",  # "🚧",
            "error": "⨯",  # "✗!🛑",
            "critical": "🔥",  # "🔥⚡",
        },
    }

    vertical_align = ptg.VerticalAlignment.TOP
    overflow = ptg.Overflow.RESIZE

    def __init__(self, record: logging.LogRecord):
        super().__init__()
        self.record = record

        level = record.levelname.lower()
        style = self.styles[level]
        icon = style(self._get_char(level))

        source_label = ptg.Label(
            record.name + ": ",
            weight=3,
            parent_align=ptg.HorizontalAlignment.RIGHT
        )
        self.lazy_add(source_label)

        icon_label = ptg.Label(icon, weight=1)
        self.lazy_add(icon_label)

        # TODO Seems to be messed up by multiline log messages
        message_label = ptg.Label(
            record.msg, 
            weight=10, 
            parent_align=ptg.HorizontalAlignment.LEFT
        )
        self.lazy_add(message_label)

    def get_lines(self) -> list[str]:
        # An error will be raised if `separator` is not the correct type (str).
        separator = self._get_style("separator")(self._get_char("separator"))  # type: ignore
        separator_length = ptg.regex.real_length(separator)

        self.positioned_line_buffer = []
        vertical_lines = []
        total_offset = 0

        # Splitter usually distributes space evenly, but we want weighted distribution
        weight_sum = sum(getattr(w, "weight", 1) for w in self._widgets)
        available_width = self.width - (len(self._widgets) - 1) * separator_length

        for widget in self._widgets:
            inner = []

            weight = getattr(widget, "weight", 1)
            target_width, error = divmod(available_width * weight, weight_sum)

            if widget.size_policy is ptg.SizePolicy.STATIC:
                target_width += target_width - widget.width
                width = widget.width
            else:
                widget.width = target_width + error
                width = widget.width
                error = 0

            aligned: str | None = None
            for line in widget.get_lines():
                # See `enums.py` for information about this ignore
                padding, aligned = self._align_line(
                    cast(ptg.HorizontalAlignment, widget.parent_align), width, line
                )
                inner.append(aligned)

            new_pos = (
                self.pos[0] + padding + total_offset,
                self.pos[1] + (1 if type(widget).__name__ == "Container" else 0),
            )

            diff_x = new_pos[0] - widget.pos[0]
            diff_y = new_pos[1] - widget.pos[1]

            widget.pos = new_pos

            for pos, line in widget.positioned_line_buffer:
                self.positioned_line_buffer.append(
                    ((pos[0] + diff_x, pos[1] + diff_y), line)
                )

            widget.positioned_line_buffer = []

            if aligned is not None:
                total_offset += ptg.regex.real_length(inner[-1]) + separator_length

            vertical_lines.append(inner)

        lines = []
        for horizontal in zip_longest(*vertical_lines, fillvalue=" " * target_width):
            lines.append((ptg.ansi_interface.reset() + separator).join(horizontal))

        self.height = max(widget.height for widget in self)
        return lines


class LogView(ptg.Window):
    overflow = ptg.Overflow.SCROLL

    def __init__(self, max_lines: int = 1000, **kwargs):
        super().__init__(**kwargs)
        # Sneakily replace our widget list with a double ended queue
        self._widgets = deque(maxlen=max_lines)

        self.width = int(self.terminal.width * 2 / 3)
        self.height = int(self.terminal.height * 2 / 3)
        self.center(store=False)
        
    def on_log_record(self, record: logging.LogRecord):
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

        pubs = shared_node.get_publisher_names_and_types_by_node(
            node.name, node.namespace
        )
        pubs.sort()
        pubs_text = ""
        for topic, types in pubs:
            pubs_text += f"\n  {topic} [{', '.join(types)}]"

        subs = shared_node.get_subscriber_names_and_types_by_node(
            node.name, node.namespace
        )
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
            is_modal=True,
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
            is_modal=True,
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

        def _close_focused(wm: ptg.WindowManager) -> None:
            if wm.focused is None:
                return

            # Find foremost non-persistent window
            for window in wm:
                if not window.is_persistent:
                    window.close()
                    return

        def _quit(wm: ptg.WindowManager) -> None:
            wm.stop()

        def _open_nodes_list(wm: ptg.WindowManager) -> None:
            # This is NOT 'ros2 node kill' as we only manage our own nodes
            # TODO should probably include components, too?
            nodes = BetterLaunch.wait_for_instance().all_nodes()

            menu = NodesList(nodes)
            # menu.center()
            menu.bind(ptg.keys.ESC, menu.close)
            wm.add(menu, assign=False)

        def _toggle_mute(wm: ptg.WindowManager) -> None:
            # TODO
            pass

        with ptg.WindowManager(autorun=False) as wm:
            wm.bind(
                ptg.keys.ESC,
                lambda *_: _close_focused(wm),
                # For closing windows
            )
            wm.bind(
                ptg.keys.CTRL_Q,
                lambda *_: _quit(wm),
                "Quit",
            )
            wm.bind(
                ptg.keys.F1,
                lambda *_: _open_nodes_list(wm),
                "Nodes",
            )
            wm.bind(
                ptg.keys.F9,
                lambda *_: _toggle_mute(wm),
                "Toggle mute",
            )

            wm.layout = self._create_layout()

            wm.add(self._create_header(wm), assign="header", animate=False)
            wm.add(self._create_body(wm), assign="body", animate=False)
            wm.add(self._create_footer(wm), assign="footer", animate=False)

            wm.layout.apply()

            wm.toast(
                "[bl.title]Welcome to [/bl.title bl.brand_title]"
                + "BetterLaunch[/bl.brand_title bl.title]!",
                offset=ptg.terminal.height // 2 - 3,
                delay=700,
            )

        # Run the launch function in a background thread
        self.launch_thread = threading.Thread(
            target=launch_func, args=args, kwargs=kwargs
        )
        self.launch_thread.start()

        # Start the UI loop
        # Make sure we wait until the user has created the singleton
        bl = BetterLaunch.wait_for_instance()
        bl.add_shutdown_callback(wm.stop)

        wm.run()

        if not bl.is_shutdown:
            bl.shutdown("UI terminated")

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

        return ptg.Window(
            content,
            box="EMPTY",
            id="bl.header",
            is_persistent=True,
            vertical_align=ptg.VerticalAlignment.TOP,
        )

    def _create_body(self, wm: ptg.WindowManager):
        content = LogView(
            self.max_log_length,
            box="SINGLE",
            id="bl.log",
            is_persistent=True,
            vertical_align=ptg.VerticalAlignment.TOP,
            parent_align=ptg.HorizontalAlignment.LEFT,
        )

        log_handler = roslog.launch_config.screen_handler
        if not isinstance(log_handler, LogRecordForwarder):
            raise RuntimeError(
                "Something modified the logging handler, UI cant't start"
            )
        log_handler.add_listener(content.on_log_record)

        return content

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

        return ptg.Window(
            content,
            box="EMPTY",
            id="bl.footer",
            is_persistent=True,
            vertical_align=ptg.VerticalAlignment.BOTTOM,
        )
