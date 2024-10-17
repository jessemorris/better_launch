
class Group:
    def __init__(self, launcher, ns: str = None, remap: dict[str, str] = None):
        self.launcher = launcher
        self.ns = ns or ""
        self.remap = dict(remap or {})

    def add_node(self, pkg: str, exec: str, name: str = None, *, anonymous: bool = False, **kwargs):
        # TODO
        pass
