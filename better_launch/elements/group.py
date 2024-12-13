
class Group:
    def __init__(self, parent, ns: str = None, remap: dict[str, str] = None):
        self.parent = parent
        self.ns = ns or ""
        self.remap = dict(remap or {})
        self.nodes = []

        if ns:
            self.logger = parent.logger.getChild(ns)
        else:
            self.logger = parent.logger

    def add_node(self, node):
        self.nodes.append(node)
        node.start()
