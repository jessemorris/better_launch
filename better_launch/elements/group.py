from .node import Node


class Group:
    def __init__(self, parent: "Group", namespace: str):
        """Groups are used in better_launch to manage node namespaces and common remaps. Beyond that they don't have any meaning for ROS, and there is usually no reason to interact with them directly.
        
        Parameters
        ----------
        parent : Group
            This group's parent group.
        namespace : str
            The namespace fragment this group represents.
        """
        self.parent = parent
        self.children = {}
        self.namespace = namespace
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

    def add_child(self, child: "Group") -> None:
        """Add a child group to this group.

        Parameters
        ----------
        child : Group
            The group to add.
        """
        ns = child.namespace
        if ns in self.children:
            raise ValueError(f"Group {self.namespace} already contains a child group named '{ns}'")

        self.children[ns] = child

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
