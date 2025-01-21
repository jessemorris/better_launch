from .node import Node


class Group:
    def __init__(self, launcher, parent, ns: str, remap: dict[str, str] = None):
        self.launcher = launcher
        self.parent = parent
        self.children = []
        self.ns = ns
        self.remap = dict(remap or {})
        self.nodes = []

        self._root_chain = self._get_chain_from_root()

    def _get_chain_from_root(self, include_root: bool = False):
        # The launcher doesn't keep the group tree, but we can rebuild at least our own branch
        chain = []
        g = self
        while g is not None:
            chain.append(g)
            g = g.parent

        if include_root and g:
            chain.append(g)

        return list(reversed(chain))

    def assemble_remaps(self):
        remaps = {}
        for g in self._root_chain:
            remaps.update(g.remap)
        return remaps

    def assemble_namespace(self):
        ns = ""

        root = self._root_chain[0].parent
        if root:
            ns = root.ns.strip("/")

        for g in self._root_chain:
            ns += "/" + g.ns.strip("/")

        while "//" in ns:
            ns = ns.replace("//", "/")

        return ns

    def add_group(self, group: "Group"):
        self.children.append(group)

    def add_node(self, node: Node):
        self.nodes.append(node)

    def __repr__(self):
        return self.assemble_namespace()
