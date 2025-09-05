class Node3D(object):
    def __init__(self, current: tuple, parent: tuple = None, g: float = 0, h: float = 0) -> None:
        self.current = current
        self.parent = parent
        self.g = g
        self.h = h
    
    def __add__(self, node):
        assert isinstance(node, Node3D)
        return Node3D((self.x + node.x, self.y + node.y, self.z + node.z), self.parent, self.g + node.g, self.h)

    def __eq__(self, node) -> bool:
        if not isinstance(node, Node3D):
            return False
        return self.current == node.current
    
    def __ne__(self, node) -> bool:
        return not self.__eq__(node)

    def __lt__(self, node) -> bool:
        assert isinstance(node, Node3D)
        return self.g + self.h < node.g + node.h or \
                (self.g + self.h == node.g + node.h and self.h < node.h)

    def __hash__(self) -> int:
        return hash(self.current)

    def __str__(self) -> str:
        return "Node({}, {}, {}, {})".format(self.current, self.parent, self.g, self.h)

    def __repr__(self) -> str:
        return self.__str__()
    
    @property
    def x(self) -> float:
        return self.current[0]
    
    @property
    def y(self) -> float:
        return self.current[1]
    
    @property
    def z(self) -> float:
        return self.current[2]

    @property
    def px(self) -> float:
        if self.parent:
            return self.parent[0]
        else:
            return None

    @property
    def py(self) -> float:
        if self.parent:
            return self.parent[1]
        else:
            return None
        
    @property
    def pz(self) -> float:
        if self.parent:
            return self.parent[2]
        else:
            return None