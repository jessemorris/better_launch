from textual.binding import Binding
from textual.widgets import ListView, ListItem, Label, Static
from textual.screen import ModalScreen


class ChoiceDialog(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss")]

    DEFAULT_CSS = """
    ChoiceDialog {
        background: rgba(0, 0, 0, 0.3);
        align: center middle;

        ListView {
            max_width: 30;
            width: auto;
            height: auto;
            padding: 1 2;
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
        if self.message:
            yield Label(self.message)

        items = [ChoiceDialog.ChoiceItem(choice) for choice in self.choices]
        yield ListView(*items)

    def on_list_view_selected(self, event: ListView.Selected):
        self.dismiss(event.item.action)
