"""
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.13
"""
import math

from python_motion_planning.utils import Planner3D, Grid3D, Node3D


class GraphSearcher3D(Planner3D):
    """
    Base class for planner based on graph searching.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid3D, heuristic_type: str="euclidean") -> None:
        super().__init__(start, goal, env)
        # heuristic type
        self.heuristic_type = heuristic_type
        # allowed motions
        self.motions = self.env.motions
        # obstacles
        self.obstacles = self.env.obstacles

    def h(self, node: Node3D, goal: Node3D) -> float:
        """
        Calculate heuristic.

        Parameters:
            node (Node3D): current node
            goal (Node3D): goal node

        Returns:
            h (float): heuristic function value of node
        """
        if self.heuristic_type == "manhattan":
            return abs(goal.x - node.x) + abs(goal.y - node.y) + abs(goal.z - node.z)
        elif self.heuristic_type == "euclidean":
            return math.hypot(goal.x - node.x, goal.y - node.y, goal.z - node.z)

    def cost(self, node1: Node3D, node2: Node3D) -> float:
        """
        Calculate cost for this motion.

        Parameters:
            node1 (Node3D): node 1
            node2 (Node3D): node 2

        Returns:
            cost (float): cost of this motion
        """
        if self.isCollision(node1, node2):
            return float("inf")
        return self.dist(node1, node2)

    def isCollision(self, node1: Node3D, node2: Node3D) -> bool:
        """
        Judge collision when moving from node1 to node2.

        Parameters:
            node1 (Node3D): node 1
            node2 (Node3D): node 2

        Returns:
            collision (bool): True if collision exists else False
        """
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1, z1 = node1.x, node1.y, node1.z
        x2, y2, z2 = node2.x, node2.y, node2.z

        if x1 != x2 and y1 != y2 and z1 != z2:
            if x2 - x1 == y1 - y2 and x2 - x1 == z1 - z2:
                s1 = (min(x1, x2), min(y1, y2), min(z1, z2))
                s2 = (max(x1, x2), max(y1, y2), max(z1, z2))
            else:
                s1 = (min(x1, x2), max(y1, y2), max(z1, z2))
                s2 = (max(x1, x2), min(y1, y2), min(z1, z2))
            if s1 in self.obstacles or s2 in self.obstacles:
                return True
        return False
