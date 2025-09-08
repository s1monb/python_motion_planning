"""
@file: jps.py
@breif: Jump Point Search motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star_3d import AStar3D
from python_motion_planning.utils import Node3D, Grid3D

class JPS3D(AStar3D):
    """
    Class for JPS motion planning.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid3D): environment
        heuristic_type (str): heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.JPS3D((5, 5), (45, 25), pmp.Grid3D(51, 31))
        >>> cost, path, expand = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Online Graph Pruning for Pathfinding On Grid Maps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid3D, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "Jump Point Search(JPS)"

    def plan(self) -> tuple:
        """
        JPS motion plan function.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        # OPEN list (priority queue) and CLOSED list (hash table)
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()
        iterations = 0
        while iterations < 20:
            iterations += 1
            node = heapq.heappop(OPEN)

            print(node.current)

            # exists in CLOSED list
            if node.current in CLOSED:
                continue

            # goal found
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            jp_list = []
            for motion in self.motions:
                jp = self.jump(node, motion)
                # exists and not in CLOSED list
                if jp and jp.current not in CLOSED:
                    jp.parent = node.current
                    jp.h = self.h(jp, self.goal)
                    jp_list.append(jp)

            for jp in jp_list:
                # update OPEN list
                heapq.heappush(OPEN, jp)

                # goal found
                if jp == self.goal:
                    break

            CLOSED[node.current] = node
        return [], [], []

    def jump(self, node: Node3D, motion: Node3D):
        """
        Jumping search recursively.

        Parameters:
            node (Node3D): current node
            motion (Node3D): the motion that current node executes

        Returns:
            jump_point (Node): jump point or None if searching fails
        """
          # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.h(new_node, self.goal)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == self.goal:
            return new_node

        # diagonal
        if motion.x and motion.y:
            # if exists jump point at horizontal or vertical
            x_dir = Node3D((motion.x, 0, 0), None, 1, None)
            y_dir = Node3D((0, motion.y, 0), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir):
                return new_node
            
        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            # print("forced neighbor found")
            return new_node
        else:
            return self.jump(new_node, motion)

    def detectForceNeighbor(self, node, motion):
        """
        Detect forced neighbor of node.

        Parameters:
            node (Node): current node
            motion (Node): the motion that current node executes

        Returns:
            flag (bool): True if current node has forced neighbor else False
        """

        x, y, z = node.current
        x_dir, y_dir, z_dir = motion.current

        # horizontal movement (x direction)
        if x_dir and not y_dir and not z_dir:
            # Check forced neighbors in same z-plane
            if (x, y + 1, z) in self.obstacles and \
                (x + x_dir, y + 1, z) not in self.obstacles:
                return True
            if (x, y - 1, z) in self.obstacles and \
                (x + x_dir, y - 1, z) not in self.obstacles:
                return True
            
            # Check forced neighbors at different z-levels
            for dz in [-1, 1]:
                if (x, y + 1, z + dz) in self.obstacles and \
                    (x + x_dir, y + 1, z + dz) not in self.obstacles:
                    return True
                if (x, y - 1, z + dz) in self.obstacles and \
                    (x + x_dir, y - 1, z + dz) not in self.obstacles:
                    return True
        
        # vertical movement (y direction)
        elif not x_dir and y_dir and not z_dir:
            # Check forced neighbors in same z-plane
            if (x + 1, y, z) in self.obstacles and \
                (x + 1, y + y_dir, z) not in self.obstacles:
                return True
            if (x - 1, y, z) in self.obstacles and \
                (x - 1, y + y_dir, z) not in self.obstacles:
                return True
            
            # Check forced neighbors at different z-levels
            for dz in [-1, 1]:
                if (x + 1, y, z + dz) in self.obstacles and \
                    (x + 1, y + y_dir, z + dz) not in self.obstacles:
                    return True
                if (x - 1, y, z + dz) in self.obstacles and \
                    (x - 1, y + y_dir, z + dz) not in self.obstacles:
                    return True
        
        # z-axis movement (vertical)
        elif not x_dir and not y_dir and z_dir:
            # Check forced neighbors in same z-plane
            if (x + 1, y, z) in self.obstacles and \
                (x + 1, y, z + z_dir) not in self.obstacles:
                return True
            if (x - 1, y, z) in self.obstacles and \
                (x - 1, y, z + z_dir) not in self.obstacles:
                return True
            if (x, y + 1, z) in self.obstacles and \
                (x, y + 1, z + z_dir) not in self.obstacles:
                return True
            if (x, y - 1, z) in self.obstacles and \
                (x, y - 1, z + z_dir) not in self.obstacles:
                return True
        
        # diagonal movement (xy plane)
        elif x_dir and y_dir and not z_dir:
            # Check forced neighbors in same z-plane
            if (x - x_dir, y, z) in self.obstacles and \
                (x - x_dir, y + y_dir, z) not in self.obstacles:
                return True
            if (x, y - y_dir, z) in self.obstacles and \
                (x + x_dir, y - y_dir, z) not in self.obstacles:
                return True
            
            # Check forced neighbors at different z-levels
            for dz in [-1, 1]:
                if (x - x_dir, y, z + dz) in self.obstacles and \
                    (x - x_dir, y + y_dir, z + dz) not in self.obstacles:
                    return True
                if (x, y - y_dir, z + dz) in self.obstacles and \
                    (x + x_dir, y - y_dir, z + dz) not in self.obstacles:
                    return True
        
        return False
      

