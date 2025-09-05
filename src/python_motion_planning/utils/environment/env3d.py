from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np

from .node3d import Node3D

class Env3D(ABC):
    def __init__(self, x_range: int, y_range: int, z_range: int, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range
        self.z_range = z_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        return {(i, j, k) for i in range(self.x_range) for j in range(self.y_range) for k in range(self.z_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid3D(Env3D):
    def __init__(self, x_range: int, y_range: int, z_range: int) -> None:
        super().__init__(x_range, y_range, z_range)
        # allowed motions
        self.motions = [Node3D((-1, 0, 0), None, 1, None), Node3D((-1, 1, 0),  None, sqrt(2), None),
                        Node3D((0, 1, 0),  None, 1, None), Node3D((1, 1, 0),   None, sqrt(2), None),
                        Node3D((1, 0, 0),  None, 1, None), Node3D((1, -1, 0),  None, sqrt(2), None),
                        Node3D((0, -1, 0), None, 1, None), Node3D((-1, -1, 0), None, sqrt(2), None),
                        Node3D((0, 0, 1), None, 1, None), Node3D((0, 0, -1), None, 1, None)]
        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()
    
    def init(self) -> None:
        """
        Initialize grid map.
        """
        x, y, z= self.x_range, self.y_range, self.z_range
        obstacles = set()

        # boundary of environment
        for k in range(z):
            for i in range(x):
                obstacles.add((i, 0, k))
                obstacles.add((i, y - 1, k))
            for i in range(y):
                obstacles.add((0, i, k))
                obstacles.add((x - 1, i, k))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))