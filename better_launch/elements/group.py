from .node import Node


class Group:
    def __init__(self, parent: "Group", namespace: str, remaps: dict[str, str] = None):
        """Groups are used in better_launch to manage node namespaces and common remaps. Beyond that they don't have any meaning for ROS, and there is usually no reason to interact with them directly.
        
        Parameters
        ----------
        parent : Group
            This group's parent group.
        namespace : str
            The namespace fragment this group represents.
        remaps : dict[str, str], optional
            Any remaps this group provides to its descendent groups and nodes.
        """
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
        """Collect the remaps from the root group up to this group.

        Returns
        -------
        dict[str, str]
            A collection of topic remaps.
        """
        remaps = {}
        for g in self._root_chain:
            remaps.update(g.remaps)
        return remaps

    def assemble_namespace(self) -> str:
        """Return the full namespace string this group represents.

        Returns
        -------
        str
            This group's namespace path from the root group.
        """
        ns = ""

        root = self._root_chain[0].parent
        if root:
            ns = root.namespace.strip("/")

        for g in self._root_chain:
            ns += "/" + g.namespace.strip("/")

        while "//" in ns:
            ns = ns.replace("//", "/")

        return ns

    def add_group(self, group: "Group") -> None:
        """Add a child group to this group.

        Parameters
        ----------
        group : Group
            The group to add.
        """
        self.children.append(group)

    def add_node(self, node: Node) -> None:
        """Add a node to this group. This group will not do any magic to enforce its namespace or remaps onto the node.

        Parameters
        ----------
        node : Node
            The node to add.
        """
        self.nodes.append(node)

    def __repr__(self) -> str:
        return self.assemble_namespace()
