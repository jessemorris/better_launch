from .node import Node


class Group:
    def __init__(self, launcher, parent, ns: str = None, remap: dict[str, str] = None):
        self.launcher = launcher
        self.parent = parent
        self.children = []
        self.ns = ns or ""
        self.remap = dict(remap or {})
        self.nodes = []

        self._root_chain = self._get_chain_from_root()

        if ns:
            self.logger = parent.logger.getChild(ns)
        else:
            self.logger = parent.logger

    def _get_chain_from_root(self, include_root: bool = False):
        # The launcher doesn't keep the group tree, but we can rebuild at least our own branch
        chain = []
        g = self
        while g is not None and g is not self.launcher:
            chain.append(g)
            g = g.parent

        if include_root:
            chain.append(g)

        return list(reversed(chain))

    def assemble_remaps(self):
        remaps = {}
        for g in self._root_chain:
            remaps.update(g.remap)
        return remaps

    def assemble_namespace(self):
        ns = "/"

        root = self._root_chain[0].parent
        if root:
            ns = root.ns

        for g in self._root_chain:
            ns += "/" + g.ns.strip("/")

        return ns

    def add_group(self, group: "Group"):
        self.children.append(group)

    def add_node(self, node: Node):
        # Assemble additional node remaps from our group branch
        remaps = self.assemble_remaps()
        remaps.update(node.remap)

        # Why do I hear mad hatter music???
        # launch_ros/actions/node.py:497
        ns = self.assemble_namespace()
        remaps["__ns"] = ns

        node.remap = remaps

        self.nodes.append(node)
        node.start()
