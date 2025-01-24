from textual.containers import Container
from textual.widgets import Input, Label


# TODO we could do this like the color_commands example, using a screen with custom commands
class NodeSearch(Container):
    def compose(self):
        yield Input(placeholder="nodename")

