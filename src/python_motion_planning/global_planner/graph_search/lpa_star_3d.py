"""
@file: lpa_star.py
@breif: Lifelong Planning A* motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .graph_search_3d import GraphSearcher3D
from python_motion_planning.utils import Node3D, Grid3D

class LNode3D(Node3D):
    """
    Class for LPA* nodes.

    Parameters:
        current (tuple): current coordinate
        g (float): minimum cost moving from start(predict)
        rhs (float): minimum cost moving from start(value)
        key (list): priority
    """
    def __init__(self, current: tuple, g: float, rhs: float, key: list) -> None:
        self.current = current
        self.g = g
        self.rhs = rhs
        self.key = key

    def __add__(self, node):
        return LNode3D((self.x + node.x, 
                       self.y + node.y, 
                       self.z + node.z), 
                      self.g, self.rhs, self.key)

    def __lt__(self, node) -> bool:
        return self.key < node.key

    def __str__(self) -> str:
        return "----------\ncurrent:{}\ng:{}\nrhs:{}\nkey:{}\n----------" \
            .format(self.current, self.g, self.rhs, self.key)

class LPAStar3D(GraphSearcher3D):
    """
    Class for LPA* motion planning.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.LPAStar((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, _ = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Lifelong Planning A*
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid3D, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
        # start and goal
        self.start = LNode3D(start, float('inf'), 0.0, None)
        self.goal = LNode3D(goal, float('inf'), float('inf'), None)
        # OPEN set and expand zone
        self.U, self.EXPAND = [], []

        # intialize global information, record history infomation of map grids
        self.map = {s: LNode3D(s, float('inf'), float('inf'), None) for s in self.env.grid_map}
        self.map[self.goal.current] = self.goal
        self.map[self.start.current] = self.start
        # OPEN set with priority
        self.start.key = self.calculateKey(self.start)
        heapq.heappush(self.U, self.start)

    def __str__(self) -> str:
        return "Lifelong Planning A*"

    def plan(self) -> tuple:
        """
        LPA* dynamic motion planning function.

        Returns:
            cost (float): path cost
            path (list): planning path
            _ (None): None
        """
        self.computeShortestPath()
        cost, path = self.extractPath()
        return cost, path, None

    def run(self) -> None:
        """
        Running both plannig and animation.
        """
        # static planning
        cost, path, _ = self.plan()

        print(path)
        print(cost)

    def computeShortestPath(self) -> None:
        """
        Perceived dynamic obstacle information to optimize global path.
        """
        max_iterations = 10000  # Safety limit to prevent infinite loops
        iteration_count = 0
        
        while self.U and iteration_count < max_iterations:
            iteration_count += 1
            node = min(self.U, key=lambda node: node.key)
            if node.key >= self.calculateKey(self.goal) and \
                    self.goal.rhs == self.goal.g:
                break

            self.U.remove(node)
            self.EXPAND.append(node)

            # Locally over-consistent -> Locally consistent
            if node.g > node.rhs:
                node.g = node.rhs
            # Locally under-consistent -> Locally over-consistent
            else:
                node.g = float("inf")
                self.updateVertex(node)

            for node_n in self.getNeighbor(node):
                self.updateVertex(node_n)
        
        if iteration_count >= max_iterations:
            print("Warning: LPA* reached maximum iterations, path may not be optimal")

    def updateVertex(self, node: LNode3D) -> None:
        """
        Update the status and the current cost to node and it's neighbor.

        Parameters:
            node (LNode3D): current node
        """
        # greed correction
        if node != self.start:
            neighbor_costs = []
            for node_n in self.getNeighbor(node):
                if node_n.g != float('inf'):
                    neighbor_costs.append(node_n.g + self.cost(node_n, node))
            
            if neighbor_costs:
                node.rhs = min(neighbor_costs)
            else:
                node.rhs = float('inf')

        if node in self.U:
            self.U.remove(node)

        # Locally unconsistent nodes should be added into OPEN set (set U)
        if node.g != node.rhs:
            node.key = self.calculateKey(node)
            heapq.heappush(self.U, node)

    def calculateKey(self, node: LNode3D) -> list:
        """
        Calculate priority of node.

        Parameters:
            node (LNode3D): current node

        Returns:
            key (list): priority of node
        """
        return [min(node.g, node.rhs) + self.h(node, self.goal),
                min(node.g, node.rhs)]

    def getNeighbor(self, node: LNode3D) -> list:
        """
        Find neighbors of node.

        Parameters:
            node (LNode3D): current node

        Returns:
            neighbors (list): neighbors of node
        """
        neighbors = []
        for motion in self.motions:
            neighbor_pos = (node + motion).current
            # Check if neighbor position is within bounds and not an obstacle
            if (neighbor_pos in self.env.grid_map and 
                neighbor_pos not in self.obstacles and
                neighbor_pos in self.map):
                neighbors.append(self.map[neighbor_pos])
        return neighbors

    def extractPath(self):
        """
        Extract the path based on greedy policy.

        Return:
            cost (float): the cost of planning path
            path (list): the planning path
        """
        node = self.goal
        path = [node.current]
        cost, count = 0, 0
        visited = set()  # Track visited nodes to prevent cycles
        
        while node != self.start:
            if node.current in visited:
                print("Path extraction detected cycle, no valid path found")
                return float('inf'), []
            
            visited.add(node.current)
            neighbors = [node_n for node_n in self.getNeighbor(node) if not self.isCollision(node, node_n)]
            
            if not neighbors:
                print("No valid neighbors found during path extraction")
                return float('inf'), []
            
            next_node = min(neighbors, key=lambda n: n.g)
            path.append(next_node.current)
            cost += self.cost(node, next_node)
            node = next_node
            count += 1
            
            if count >= 1000:
                print("Path extraction exceeded maximum iterations")
                return float('inf'), []
                
        return cost, list(reversed(path))

