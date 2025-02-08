from .node import Node


class Group:
    def __init__(self, parent, namespace: str, remaps: dict[str, str] = None):
        self.parent = parent
        self.children = []
        self.namespace = namespace
        self.remaps = remaps or {}
        self.nodes = []

        self._root_chain = self._get_chain_from_root()

    def _get_chain_from_root(self, include_root: bool = False) -> list["Group"]:
        # The launcher doesn't keep the group tree, but we can rebuild at least our own branch
        chain = []
        g = self
        while g is not None:
            chain.append(g)
            g = g.parent

        if include_root and g:
            chain.append(g)

        return list(reversed(chain))

    def assemble_remaps(self) -> dict[str, str]:
        remaps = {}
        for g in self._root_chain:
            remaps.update(g.remaps)
        return remaps

    def assemble_namespace(self) -> str:
        ns = ""

        root = self._root_chain[0].parent
        if root:
            ns = root.ns.strip("/")

        for g in self._root_chain:
            ns += "/" + g.ns.strip("/")

        while "//" in ns:
            ns = ns.replace("//", "/")

        return ns

    def add_group(self, group: "Group") -> None:
        self.children.append(group)

    def add_node(self, node: Node) -> None:
        self.nodes.append(node)

    def __repr__(self) -> str:
        return self.assemble_namespace()
