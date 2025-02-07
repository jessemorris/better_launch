from textual.binding import Binding
from textual.widgets import ListView, ListItem, Label, Static
from textual.containers import VerticalGroup
from textual.screen import ModalScreen


class ChoiceDialog(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss")]

    DEFAULT_CSS = """
    ChoiceDialog {
        background: rgba(0, 0, 0, 0.3);
        align: center middle;
        width: auto;
        height: auto;

        VerticalGroup {
            margin: 1;
            width: 100%;
            height: auto;
            max-width: 40;
            content-align: center middle;

            #message {
                background: $panel;
                padding: 1;
            }

            #choices {
                width: 100%;
                height: auto;
                padding: 1 2    ;
            }
        }
    }
    """

    class ChoiceItem(ListItem):
        def __init__(self, action: str, **kwargs):
            super().__init__(Static(action), **kwargs)
            self.action = action

    def __init__(
        self,
        choices: list[str],
        message: str = None,
        title: str = None,
        *,
        name=None,
        id=None,
        classes=None
    ):
        super().__init__(name, id, classes)
        self.message = message
        self.choices = choices
        self.title = title

    def compose(self):
        if self.title:
            yield Label(self.title)

        with VerticalGroup():
            if self.message:
                yield Static(self.message, id="message")

            items = [
                ChoiceDialog.ChoiceItem(choice, classes="choice")
                for choice in self.choices
            ]
            yield ListView(*items, id="choices")

    def on_list_view_selected(self, event: ListView.Selected):
        self.dismiss(event.item.action)
