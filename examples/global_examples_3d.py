"""
@file: global_examples.py
@breif: global planner application examples
@author: Yang Haodong, Wu Maojia
@update: 2024.11.22
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid3D, SearchFactory

if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    search_factory = SearchFactory()

    '''
    graph search
    '''
    # Obst er alle obstacles-ene for de faktiske etasjene
    obst = {(5,5,0), (5,5,2), (3,2,0)}

    # stairs er en dict som for hver mellom etasje,
    # så har du en set med nodene hvor du kan gå opp eller ned.
    # Så for eksempel siden eksemplet her har 2 etasjer. Har vi 3 mulige lag.
    # Laget med indeks 1 vil være mellom etasjen.
    # Så for å komme seg til lag 0 til lag 2, må du være på 4, 4, 0 eller 3, 3, 0
    stairs = {1: {(4,4,1),(3,3,1)}}


    start = (1, 1, 0)
    goal = (7, 7, 2)
    
    env = Grid3D(11,11,3, stairs, obst)

    # creat planner
    # planner = search_factory("a_star_3d", start=start, goal=goal, env=env)
    # planner = search_factory("dijkstra_3d", start=start, goal=goal, env=env)
    planner = search_factory("d_star_3d", start=start, goal=goal, env=env)

    # animation
    planner.run()
