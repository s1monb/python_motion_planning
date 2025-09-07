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
    obst = {(5,5,1), (3,2,1), (5,5,3)}

    # stairs er en dict som for hver mellom etasje,
    # så har du en set med nodene hvor du kan gå opp eller ned.
    # Så for eksempel siden eksemplet her har 2 etasjer. Har vi 3 mulige lag.
    # Laget med indeks 1 vil være mellom etasjen.
    # Så for å komme seg til lag 0 til lag 2, må du være på 4, 4, 0 eller 3, 3, 0
    stairs = {0: {}, 1: {}, 2: {(1,5,2)}, 3: {}, 4: {}}


    start = (1, 1, 1)
    goal = (7, 7, 3)
    
    env = Grid3D(10,10,5, stairs, obst)

    # creat planner
    a_star_planner = search_factory("a_star_3d", start=start, goal=goal, env=env)
    dijkstra_planner = search_factory("dijkstra_3d", start=start, goal=goal, env=env)
    d_star_planner = search_factory("d_star_3d", start=start, goal=goal, env=env)
    gbfs_planner = search_factory("gbfs_3d", start=start, goal=goal, env=env)
    lpa_star_planner = search_factory("lpa_star_3d", start=start, goal=goal, env=env)
    jps_planner = search_factory("jps_3d", start=start, goal=goal, env=env)

    # animation
    print("A* 3D")
    a_star_planner.run()
    print("Dijkstra 3D")
    dijkstra_planner.run()
    print("D* 3D")
    d_star_planner.run()
    print("GBFS 3D")
    gbfs_planner.run()
    print("LPA* 3D")
    lpa_star_planner.run()
    print("JPS 3D")
    jps_planner.run()
