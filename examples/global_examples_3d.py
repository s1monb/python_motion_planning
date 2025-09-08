"""
@file: global_examples.py
@breif: global planner application examples
@author: Yang Haodong, Wu Maojia
@update: 2024.11.22
"""
import sys, os
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid3D, SearchFactory
from time import time

if __name__ == '__main__':

    search_factory = SearchFactory()
    # Obst er alle obstacles-ene for de faktiske etasjene

    # stairs er en dict som for hver mellom etasje,
    # så har du en set med nodene hvor du kan gå opp eller ned.
    # Så for eksempel siden eksemplet her har 2 etasjer. Har vi 3 mulige lag.
    # Laget med indeks 1 vil være mellom etasjen.
    # Så for å komme seg til lag 0 til lag 2, må du være på 4, 4, 0 eller 3, 3, 0
    obst = {(5,5,1), (3,2,1), (5,5,3)}
    stairs = {0: {}, 2: {(32,60,2)}, 4: {}}
    
    envs = [(Grid3D(100,100,5, stairs, obst), (1,1,1), (70,70,3)) for _ in range(10)]

    planners = ["a_star_3d", "dijkstra_3d", "gbfs_3d", "jps_3d"]

    time_data = defaultdict(list)

    for env in envs:
        for planner in planners:
            planner_instance = search_factory(planner, start=env[1], goal=env[2], env=env[0])
            time_start = time()
            planner_instance.run()
            time_end = time()
            print(f"{planner} time: {time_end - time_start} seconds")
            time_data[planner].append(time_end - time_start)

    # Prepare data for box plot
    data_to_plot = []
    labels = []
    
    for planner in planners:
        if planner in time_data and time_data[planner]:
            data_to_plot.append(time_data[planner])
            labels.append(planner.replace('_3d', '').replace('_', ' ').title())
    
    # Create box plot
    plt.figure(figsize=(12, 6))
    box_plot = plt.boxplot(data_to_plot, labels=labels, patch_artist=True)
    
    # Customize colors
    colors = ['lightblue', 'lightgreen', 'lightcoral', 'lightyellow', 'lightpink', 'lightgray']
    for patch, color in zip(box_plot['boxes'], colors[:len(box_plot['boxes'])]):
        patch.set_facecolor(color)
    
    plt.xlabel("Planning Algorithm", fontsize=12)
    plt.ylabel("Execution Time (seconds)", fontsize=12)
    plt.title("Execution Time Comparison of 3D Motion Planning Algorithms", fontsize=14, fontweight='bold')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Print summary statistics
    print("\n" + "=" * 60)
    print("TIMING SUMMARY")
    print("=" * 60)
    for planner in planners:
        if planner in time_data and time_data[planner]:
            times = time_data[planner]
            print(f"{planner:15s}: Mean={np.mean(times):.4f}s, Std={np.std(times):.4f}s, Min={np.min(times):.4f}s, Max={np.max(times):.4f}s")
    
    plt.show()