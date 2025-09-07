"""
@file: planner.py
@breif: Abstract class for planner
@author: Winter
@update: 2023.1.17
"""
import math
from abc import abstractmethod, ABC

from python_motion_planning.utils.environment.env3d import Env3D
from python_motion_planning.utils.environment.node3d import Node3D
from ..plot.plot import Plot

class Planner3D(ABC):
    def __init__(self, start: tuple, goal: tuple, env: Env3D) -> None:
        # plannig start and goal
        self.start = Node3D(start, start, 0, 0)
        self.goal = Node3D(goal, goal, 0, 0)
        # environment
        self.env = env
        # graph handler
        self.plot = Plot(start, goal, env)

    def dist(self, node1: Node3D, node2: Node3D) -> float:
        return math.hypot(node2.x - node1.x, node2.y - node1.y, node2.z - node1.z)
    
    def angle(self, node1: Node3D, node2: Node3D) -> float:
        return math.atan2(node2.y - node1.y, node2.x - node1.x)

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass

    @abstractmethod
    def run(self):
        '''
        Interface for running both plannig and animation.
        '''
        pass