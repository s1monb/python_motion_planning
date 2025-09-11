
import sys, os
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid3D, SearchFactory
from time import time


def plot_goals(goals, planners, plt, title):
    boxplot_data = defaultdict(list)
    scatter_data = defaultdict(list)  # Store (time, cost) pairs for each planner
    
    for goal in goals:
        for planner in planners:
            planner_instance = search_factory(planner, start=start, goal=goal, env=grid)
            time_start = time()
            planner_instance.run()
            time_end = time()
            
            # Get cost from the planner's plan method
            cost, path, expand = planner_instance.plan()
            
            # Handle different cost return types (some planners return lists, others single values)
            if isinstance(cost, (list, tuple)) and len(cost) > 0:
                # If cost is a list/tuple, take the first element or sum if it's a path cost
                if isinstance(cost[0], (int, float)):
                    cost_value = sum(cost) if len(cost) > 1 else cost[0]
                else:
                    cost_value = cost[0]
            elif isinstance(cost, (int, float)):
                cost_value = cost
            else:
                # If cost is empty or invalid, skip this data point
                continue
            
            execution_time = time_end - time_start
            boxplot_data[planner].append(execution_time)
            scatter_data[planner].append((execution_time, cost_value))
    
    # Remove outliers using IQR method
    def remove_outliers(data, multiplier=1.5):
        """Remove outliers using IQR method"""
        if len(data) < 4:  # Need at least 4 points for IQR
            return data
        
        data_array = np.array(data)
        Q1 = np.percentile(data_array, 25)
        Q3 = np.percentile(data_array, 75)
        IQR = Q3 - Q1
        
        # Define outlier bounds
        lower_bound = Q1 - multiplier * IQR
        upper_bound = Q3 + multiplier * IQR
        
        # Filter out outliers
        filtered_data = [x for x in data if lower_bound <= x <= upper_bound]
        return filtered_data
    
    # Filter outliers from timing data
    filtered_boxplot_data = defaultdict(list)
    filtered_scatter_data = defaultdict(list)
    
    for planner in planners:
        if planner in boxplot_data and boxplot_data[planner]:
            # Get timing data for outlier detection
            times = [item[0] for item in scatter_data[planner]]
            filtered_times = remove_outliers(times)
            
            # Create a set of filtered times for matching
            filtered_times_set = set(filtered_times)
            
            # Filter both boxplot and scatter data
            for i, time_val in enumerate(boxplot_data[planner]):
                if time_val in filtered_times_set:
                    filtered_boxplot_data[planner].append(time_val)
            
            for time_val, cost_val in scatter_data[planner]:
                if time_val in filtered_times_set:
                    filtered_scatter_data[planner].append((time_val, cost_val))
    
    # Use filtered data for plotting
    boxplot_data = filtered_boxplot_data
    scatter_data = filtered_scatter_data

    # Prepare data for box plot
    data_to_plot = []
    labels = []
    
    for planner in planners:
        if planner in boxplot_data and boxplot_data[planner]:
            data_to_plot.append(boxplot_data[planner])
            labels.append(planner.replace('_3d', '').replace('_', ' ').title())
    
    # Create box plot figure
    plt.figure(figsize=(8, 6))
    box_plot = plt.boxplot(data_to_plot, tick_labels=labels, patch_artist=True)
    
    # Customize colors for box plot
    colors = ['lightblue', 'lightgreen', 'lightcoral', 'lightyellow', 'lightpink', 'lightgray']
    for patch, color in zip(box_plot['boxes'], colors[:len(box_plot['boxes'])]):
        patch.set_facecolor(color)
    
    plt.xlabel("Planning Algorithm", fontsize=12)
    plt.ylabel("Execution Time (seconds)", fontsize=12)
    plt.title(f"{title} - Execution Time Comparison", fontsize=14, fontweight='bold')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Create scatter plot figure
    plt.figure(figsize=(8, 6))
    scatter_colors = ['blue', 'green', 'red', 'orange', 'purple', 'brown']
    for i, planner in enumerate(planners):
        if planner in scatter_data and scatter_data[planner]:
            times, costs = zip(*scatter_data[planner])
            # Convert to numpy arrays to ensure proper data types
            times = np.array(times, dtype=float)
            costs = np.array(costs, dtype=float)
            plt.scatter(times, costs, c=scatter_colors[i % len(scatter_colors)], 
                       label=planner.replace('_3d', '').replace('_', ' ').title(), 
                       alpha=0.7, s=50)
    
    plt.xlabel("Execution Time (seconds)", fontsize=12)
    plt.ylabel("Path Cost", fontsize=12)
    plt.title(f"{title} - Time vs Cost Analysis", fontsize=14, fontweight='bold')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Print summary statistics
    print("\n" + "=" * 60)
    print("TIMING SUMMARY (Outliers Removed)")
    print("=" * 60)
    for planner in planners:
        if planner in boxplot_data and boxplot_data[planner]:
            times = boxplot_data[planner]
            print(f"{planner:15s}: Mean={np.mean(times):.4f}s, Std={np.std(times):.4f}s, Min={np.min(times):.4f}s, Max={np.max(times):.4f}s, Count={len(times)}")
    
    print("\n" + "=" * 60)
    print("COST SUMMARY (Outliers Removed)")
    print("=" * 60)
    for planner in planners:
        if planner in scatter_data and scatter_data[planner]:
            times, costs = zip(*scatter_data[planner])
            costs = np.array(costs, dtype=float)
            print(f"{planner:15s}: Mean Cost={np.mean(costs):.4f}, Std Cost={np.std(costs):.4f}, Min Cost={np.min(costs):.4f}, Max Cost={np.max(costs):.4f}, Count={len(costs)}")
    

if __name__ == '__main__':

    search_factory = SearchFactory()
    floors = 5

    floor_1_goals = {(1, 16, 1), (19, 4, 1), (16, 13, 1), (17, 19, 1), (18, 18, 1), (11, 12, 1), (6, 10, 1), (9, 14, 1), (9, 18, 1), (11, 14, 1), (14, 18, 1), (9, 11, 1), (4, 19, 1), (13, 1, 1), (12, 11, 1), (5, 17, 1), (19, 11, 1), (1, 2, 1), (2, 14, 1), (7, 5, 1), (1, 19, 1), (14, 17, 1), (11, 13, 1), (3, 14, 1), (17, 13, 1), (18, 3, 1), (13, 9, 1), (3, 18, 1), (18, 16, 1), (13, 11, 1)}
    floor_2_goals = {(7, 9, 3), (10, 17, 3), (14, 19, 3), (12, 3, 3), (2, 3, 3), (19, 11, 3), (18, 7, 3), (2, 14, 3), (9, 12, 3), (7, 17, 3), (1, 16, 3), (19, 17, 3), (17, 8, 3), (1, 7, 3), (14, 7, 3), (17, 1, 3), (3, 17, 3), (16, 2, 3), (9, 7, 3), (7, 12, 3), (9, 11, 3), (4, 8, 3), (2, 2, 3), (17, 18, 3), (19, 14, 3), (7, 14, 3), (3, 3, 3), (14, 11, 3), (13, 3, 3), (4, 14, 3)}
    floor_3_goals = {(16, 17, 5), (13, 1, 5), (18, 4, 5), (15, 8, 5), (6, 3, 5), (6, 16, 5), (13, 14, 5), (13, 3, 5), (4, 12, 5), (6, 7, 5), (18, 1, 5), (3, 7, 5), (12, 12, 5), (12, 1, 5), (3, 5, 5), (16, 14, 5), (9, 8, 5), (7, 11, 5), (17, 2, 5), (7, 13, 5), (6, 13, 5), (19, 11, 5), (11, 8, 5), (7, 2, 5), (19, 2, 5), (12, 7, 5), (11, 12, 5), (18, 13, 5), (16, 15, 5), (19, 1, 5)}
    floor_4_goals = {(9, 3, 7), (1, 7, 7), (4, 13, 7), (2, 18, 7), (9, 16, 7), (1, 9, 7), (5, 4, 7), (3, 19, 7), (18, 6, 7), (19, 14, 7), (14, 11, 7), (3, 12, 7), (1, 4, 7), (9, 13, 7), (13, 3, 7), (7, 16, 7), (17, 7, 7), (4, 3, 7), (5, 1, 7), (19, 16, 7), (19, 18, 7), (13, 16, 7), (3, 5, 7), (18, 16, 7), (1, 10, 7), (1, 12, 7), (17, 6, 7), (2, 14, 7), (11, 8, 7), (18, 5, 7)}
    floor_5_goals = {(14, 8, 9), (8, 8, 9), (3, 16, 9), (13, 18, 9), (7, 13, 9), (14, 1, 9), (1, 1, 9), (13, 11, 9), (19, 4, 9), (4, 9, 9), (17, 4, 9), (3, 13, 9), (4, 4, 9), (14, 7, 9), (4, 6, 9), (3, 17, 9), (9, 18, 9), (16, 4, 9), (8, 16, 9), (11, 14, 9), (7, 3, 9), (2, 11, 9), (11, 5, 9), (2, 2, 9), (11, 18, 9), (11, 11, 9), (10, 7, 9), (7, 18, 9), (14, 4, 9), (14, 17, 9)}
    
    obst ={(16, 0, 1), (19, 15, 1), (3, 15, 1), (4, 0, 1), (6, 0, 9), (0, 11, 7), (0, 3, 3), (17, 10, 1), (11, 10, 7), (20, 6, 9), (5, 5, 3), (5, 16, 3), (7, 5, 5), (18, 0, 7), (9, 5, 7), (1, 0, 1), (20, 15, 5), (3, 0, 3), (20, 7, 1), (6, 15, 3), (20, 18, 1), (18, 20, 3), (8, 15, 5), (15, 3, 5), (15, 6, 1), (10, 13, 5), (10, 1, 9), (10, 12, 9), (2, 15, 1), (4, 15, 3), (5, 0, 9), (0, 10, 1), (18, 5, 5), (5, 20, 5), (20, 2, 7), (5, 1, 1), (7, 20, 7), (18, 15, 9), (9, 20, 9), (8, 20, 3), (1, 15, 3), (2, 0, 3), (3, 15, 5), (4, 0, 5), (15, 10, 3), (0, 3, 7), (17, 10, 5), (10, 17, 7), (4, 20, 1), (5, 5, 7), (5, 16, 7), (13, 5, 3), (0, 17, 9), (20, 18, 5), (6, 15, 7), (13, 0, 1), (18, 20, 7), (0, 18, 1), (20, 13, 3), (15, 5, 9), (2, 15, 5), (14, 10, 3), (16, 10, 5), (0, 10, 5), (0, 2, 1), (0, 14, 3), (10, 5, 1), (10, 16, 1), (5, 20, 9), (5, 12, 5), (5, 1, 5), (5, 4, 1), (19, 15, 9), (8, 0, 1), (2, 0, 7), (4, 0, 9), (20, 3, 3), (20, 14, 3), (15, 2, 3), (15, 13, 3), (3, 10, 7), (20, 17, 5), (15, 16, 5), (0, 9, 9), (13, 0, 5), (0, 6, 3), (0, 18, 5), (10, 20, 3), (11, 5, 3), (5, 8, 3), (20, 13, 7), (13, 20, 1), (5, 19, 3), (7, 0, 1), (2, 15, 9), (9, 0, 3), (20, 10, 1), (14, 10, 7), (15, 17, 5), (0, 10, 9), (15, 9, 1), (0, 2, 5), (10, 5, 5), (12, 5, 7), (15, 20, 7), (5, 4, 5), (0, 5, 7), (0, 16, 7), (10, 19, 7), (11, 10, 1), (20, 6, 3), (1, 10, 9), (18, 0, 1), (20, 17, 9), (12, 20, 1), (5, 15, 1), (20, 9, 5), (7, 15, 3), (15, 16, 9), (19, 10, 1), (13, 0, 9), (0, 6, 7), (10, 20, 7), (17, 5, 1), (11, 5, 7), (10, 1, 3), (10, 12, 3), (5, 8, 7), (5, 19, 7), (5, 0, 3), (7, 0, 5), (9, 0, 7), (20, 10, 5), (20, 2, 1), (8, 10, 5), (1, 20, 3), (15, 12, 7), (9, 15, 1), (2, 10, 1), (10, 11, 7), (4, 10, 3), (5, 6, 9), (11, 20, 9), (0, 17, 3), (18, 0, 5), (10, 8, 1), (12, 20, 5), (5, 15, 5), (20, 9, 9), (7, 15, 7), (5, 7, 1), (6, 15, 1), (8, 15, 3), (19, 10, 5), (15, 5, 3), (17, 5, 5), (10, 1, 7), (10, 4, 3), (4, 15, 1), (5, 0, 7), (5, 11, 7), (0, 1, 9), (0, 12, 9), (10, 15, 9), (7, 0, 9), (5, 14, 9), (20, 2, 5), (6, 10, 7), (18, 15, 7), (8, 10, 9), (0, 13, 1), (1, 20, 7), (20, 5, 7), (20, 16, 7), (15, 0, 9), (15, 4, 7), (9, 15, 5), (2, 10, 5), (4, 10, 7), (3, 10, 1), (15, 1, 1), (16, 5, 5), (0, 17, 7), (10, 8, 5), (0, 9, 3), (10, 0, 1), (5, 15, 9), (12, 20, 9), (12, 0, 3), (5, 7, 5), (19, 10, 9), (20, 13, 1), (15, 5, 7), (17, 5, 9), (1, 5, 5), (20, 20, 9), (20, 1, 5), (20, 12, 5), (15, 8, 9), (15, 19, 9), (15, 11, 5), (17, 0, 7), (10, 18, 9), (14, 20, 5), (16, 20, 7), (15, 20, 1), (0, 13, 5), (17, 20, 3), (0, 16, 1), (0, 5, 1), (10, 19, 1), (11, 0, 3), (5, 3, 3), (13, 15, 1), (5, 18, 1), (18, 10, 9), (9, 15, 9), (2, 10, 9), (20, 17, 3), (14, 5, 7), (15, 1, 5), (16, 5, 9), (15, 16, 3), (3, 20, 9), (10, 8, 9), (10, 0, 5), (12, 0, 7), (15, 15, 7), (0, 0, 7), (10, 14, 7), (11, 5, 1), (13, 10, 7), (5, 13, 7), (6, 20, 5), (0, 20, 3), (20, 12, 9), (20, 1, 9), (12, 15, 1), (20, 4, 5), (15, 11, 9), (19, 5, 1), (14, 20, 9), (15, 20, 5), (17, 20, 7), (0, 5, 5), (0, 16, 5), (11, 0, 7), (10, 19, 5), (10, 11, 1), (5, 3, 7), (5, 18, 5), (0, 4, 9), (11, 20, 3), (20, 9, 3), (6, 5, 3), (8, 5, 5), (20, 8, 7), (20, 19, 7), (15, 7, 7), (15, 18, 7), (9, 10, 1), (10, 6, 7), (6, 20, 9), (0, 20, 7), (0, 1, 3), (0, 12, 3), (10, 3, 1), (10, 15, 3), (12, 15, 5), (20, 4, 9), (5, 2, 1), (5, 14, 3), (8, 10, 3), (19, 5, 5), (1, 20, 1), (20, 5, 1), (20, 16, 1), (15, 12, 5), (15, 0, 3), (15, 4, 1), (5, 6, 7), (0, 7, 9), (10, 10, 9), (5, 9, 9), (8, 5, 9), (0, 8, 1), (0, 19, 1), (20, 0, 7), (20, 11, 7), (5, 10, 1), (7, 10, 3), (9, 10, 5), (20, 20, 3), (3, 5, 1), (14, 0, 3), (15, 8, 3), (15, 19, 3), (0, 12, 7), (16, 0, 5), (10, 15, 7), (10, 3, 5), (17, 0, 1), (12, 15, 9), (10, 18, 3), (16, 20, 1), (19, 5, 9), (15, 0, 7), (1, 0, 5), (20, 15, 9), (3, 0, 7), (20, 7, 5), (15, 3, 9), (15, 14, 9), (15, 6, 5), (19, 20, 3), (10, 2, 9), (10, 13, 9), (3, 20, 3), (4, 5, 3), (15, 15, 1), (0, 8, 5), (17, 15, 3), (0, 19, 5), (0, 0, 1), (5, 10, 5), (13, 10, 1), (5, 13, 1), (18, 5, 9), (9, 10, 9), (20, 1, 3), (14, 0, 7), (20, 12, 3), (8, 20, 7), (1, 15, 7), (16, 0, 9), (15, 11, 3), (3, 15, 9), (10, 3, 9), (17, 0, 5), (14, 20, 3), (15, 10, 7), (16, 20, 5), (2, 20, 3), (4, 20, 5), (11, 0, 1), (13, 5, 7), (0, 4, 3), (0, 15, 3), (1, 0, 9), (20, 7, 9), (20, 18, 9), (12, 10, 1), (5, 17, 3), (15, 6, 9), (19, 20, 7), (2, 5, 5), (19, 0, 1), (4, 5, 7), (20, 8, 1), (20, 19, 1), (15, 15, 5), (15, 7, 1), (0, 0, 5), (10, 14, 5), (10, 6, 1), (5, 13, 5), (0, 14, 7), (11, 15, 3), (5, 1, 9), (5, 12, 9), (20, 4, 3), (6, 0, 3), (0, 11, 1), (8, 0, 5), (20, 3, 7), (20, 14, 7), (15, 2, 7), (2, 20, 7), (9, 5, 1), (4, 20, 9), (14, 15, 5), (16, 15, 7), (0, 4, 7), (0, 15, 7), (0, 7, 3), (10, 10, 3), (5, 17, 7), (0, 18, 9), (5, 9, 3), (6, 5, 1), (8, 5, 3), (2, 5, 9), (19, 0, 5), (13, 20, 5), (20, 8, 5), (20, 19, 5), (20, 0, 1), (20, 11, 1), (15, 7, 5), (15, 17, 9), (7, 20, 1), (9, 20, 3), (0, 2, 9), (10, 5, 9), (10, 16, 9), (6, 0, 7), (0, 11, 5), (8, 0, 9), (0, 3, 1), (10, 17, 1), (20, 6, 7), (5, 5, 1), (20, 15, 3), (14, 15, 9), (3, 0, 1), (18, 20, 1), (15, 14, 3), (0, 7, 7), (10, 10, 7), (12, 10, 9), (10, 2, 3), (10, 13, 3), (19, 0, 9), (13, 20, 9), (20, 10, 9), (5, 20, 3), (7, 20, 5), (9, 20, 7), (8, 20, 1), (3, 15, 3), (2, 0, 1), (4, 0, 3), (0, 11, 9), (15, 10, 1), (0, 3, 5), (10, 17, 5), (5, 5, 5), (5, 16, 5), (13, 5, 1), (7, 5, 7), (18, 0, 9), (20, 7, 3), (20, 18, 3), (6, 15, 5), (18, 20, 5), (8, 15, 7), (2, 15, 3), (10, 4, 7), (4, 15, 5), (14, 10, 1), (16, 10, 3), (0, 10, 3), (0, 14, 1), (5, 20, 7), (20, 2, 9), (5, 12, 3), (5, 1, 3), (7, 20, 9), (19, 15, 7), (2, 0, 5), (4, 0, 7), (20, 3, 1), (20, 14, 1), (15, 10, 5), (17, 10, 7), (1, 10, 3), (15, 2, 1), (15, 13, 1), (3, 10, 5), (10, 9, 5), (16, 15, 1), (0, 9, 7), (5, 7, 9), (6, 15, 9), (13, 0, 3), (0, 18, 3), (0, 6, 1), (10, 20, 1), (5, 8, 1), (20, 13, 5), (5, 19, 1), (2, 15, 7), (9, 0, 1), (14, 10, 5), (15, 17, 3), (16, 10, 7), (0, 10, 7), (0, 2, 3), (0, 14, 5), (10, 5, 3), (5, 12, 7), (5, 4, 3), (0, 13, 9), (6, 0, 1), (8, 0, 3), (2, 0, 9), (13, 15, 5), (20, 3, 5), (20, 14, 5), (20, 6, 1), (15, 2, 5), (15, 13, 5), (3, 10, 9), (20, 17, 7), (15, 1, 9), (7, 15, 1), (15, 16, 7), (10, 0, 9), (13, 0, 7), (0, 6, 5), (0, 18, 7), (10, 20, 5), (11, 5, 5), (10, 1, 1), (10, 12, 1), (5, 8, 5), (5, 19, 5), (13, 20, 3), (20, 13, 9), (5, 0, 1), (5, 11, 1), (7, 0, 3), (9, 0, 5), (20, 10, 3), (15, 17, 7), (15, 9, 3), (9, 20, 1), (0, 2, 7), (10, 5, 7), (10, 16, 7), (12, 5, 9), (15, 20, 9), (0, 5, 9), (0, 16, 9), (10, 19, 9), (10, 11, 5), (5, 18, 9), (11, 20, 7), (0, 17, 1), (18, 0, 3), (12, 20, 3), (5, 15, 3), (20, 9, 7), (8, 15, 1), (0, 6, 9), (15, 5, 1), (10, 20, 9), (17, 5, 3), (10, 12, 5), (10, 1, 5), (5, 8, 9), (5, 19, 9), (5, 0, 5), (0, 1, 7), (7, 0, 7), (9, 0, 9), (5, 14, 7), (20, 2, 3), (6, 10, 5), (18, 15, 5), (8, 10, 7), (1, 20, 5), (20, 16, 5), (20, 5, 5), (15, 4, 5), (2, 10, 3), (10, 11, 9), (4, 10, 5), (14, 5, 1), (16, 5, 3), (0, 17, 5), (10, 8, 3), (0, 9, 1), (12, 20, 7), (5, 15, 7), (12, 0, 1), (5, 7, 3), (7, 15, 9), (19, 10, 7), (15, 5, 5), (17, 5, 7), (1, 5, 3), (20, 20, 7), (10, 4, 5), (15, 19, 7), (16, 10, 1), (10, 7, 7), (10, 18, 7), (5, 2, 9), (6, 10, 9), (0, 13, 3), (17, 20, 1), (1, 20, 9), (20, 5, 9), (20, 16, 9), (5, 3, 1), (15, 4, 9), (9, 15, 7), (1, 10, 1), (4, 10, 9), (3, 10, 3), (20, 17, 1), (14, 5, 5), (15, 1, 3), (16, 5, 7), (15, 16, 1), (3, 20, 7), (0, 9, 5), (10, 0, 3), (12, 0, 5), (5, 7, 7), (0, 8, 9), (0, 19, 9), (5, 10, 9), (13, 10, 5), (6, 20, 3), (0, 20, 1), (1, 5, 7), (3, 5, 9), (20, 1, 7), (20, 12, 7), (17, 0, 9), (14, 20, 7), (16, 20, 9), (15, 20, 3), (0, 13, 7), (17, 20, 5), (0, 5, 3), (0, 16, 3), (10, 19, 3), (11, 0, 5), (13, 15, 3), (11, 20, 1), (14, 5, 9), (20, 9, 1), (15, 1, 7), (10, 0, 7), (12, 0, 9), (15, 15, 9), (0, 0, 9), (13, 10, 9), (5, 13, 9), (6, 20, 7), (0, 20, 5), (11, 15, 7), (0, 1, 1), (0, 12, 1), (10, 15, 1), (12, 15, 3), (20, 4, 7), (5, 14, 1), (19, 5, 3), (17, 20, 9), (15, 0, 1), (11, 0, 9), (10, 11, 3), (5, 6, 5), (11, 20, 5), (5, 9, 7), (6, 5, 5), (20, 8, 9), (20, 19, 9), (20, 0, 5), (20, 11, 5), (15, 18, 9), (9, 10, 3), (10, 6, 9), (20, 20, 1), (14, 0, 1), (0, 20, 9), (16, 0, 3), (15, 8, 1), (0, 1, 5), (0, 12, 5), (10, 3, 3), (10, 15, 5), (10, 18, 1), (5, 2, 3), (5, 14, 5), (19, 5, 7), (20, 5, 3), (20, 16, 3), (15, 0, 5), (18, 10, 1), (1, 0, 3), (15, 4, 3), (20, 15, 7), (3, 0, 5), (15, 3, 7), (16, 5, 1), (19, 20, 1), (10, 13, 7), (3, 20, 1), (6, 5, 9), (0, 8, 3), (0, 19, 3), (20, 0, 9), (20, 11, 9), (5, 10, 3), (7, 10, 5), (9, 10, 7), (1, 5, 1), (20, 20, 5), (3, 5, 3), (20, 1, 1), (20, 12, 1), (14, 0, 5), (8, 20, 5), (15, 19, 5), (16, 0, 7), (15, 11, 1), (3, 15, 7), (10, 3, 7), (17, 0, 3), (10, 18, 5), (14, 20, 1), (16, 20, 3), (0, 3, 9), (2, 20, 1), (4, 20, 3), (5, 5, 9), (5, 16, 9), (13, 5, 5), (0, 4, 1), (0, 15, 1), (1, 0, 7), (3, 0, 9), (20, 7, 7), (20, 18, 7), (18, 20, 9), (15, 6, 7), (19, 20, 5), (2, 5, 3), (3, 20, 5), (15, 15, 3), (0, 8, 7), (17, 15, 5), (0, 19, 7), (0, 0, 3), (10, 14, 3), (5, 10, 7), (13, 10, 3), (7, 10, 9), (6, 20, 1), (11, 15, 1), (14, 0, 9), (20, 4, 1), (8, 20, 9), (1, 15, 9), (15, 10, 9), (2, 20, 5), (10, 9, 9), (4, 20, 7), (14, 15, 3), (13, 5, 9), (0, 15, 5), (0, 4, 5), (0, 7, 1), (10, 10, 1), (12, 10, 3), (5, 9, 1), (19, 20, 9), (8, 5, 1), (2, 5, 7), (19, 0, 3), (4, 5, 9), (20, 8, 3), (20, 19, 3), (17, 15, 9), (15, 7, 3), (10, 6, 3), (0, 14, 9), (11, 15, 5), (6, 0, 5), (0, 11, 3), (8, 0, 7), (11, 10, 3), (20, 14, 9), (20, 3, 9), (20, 6, 5), (15, 13, 9), (2, 20, 9), (9, 5, 3), (20, 15, 1), (14, 15, 7), (16, 15, 9), (0, 15, 9), (15, 3, 1), (15, 14, 1), (0, 7, 5), (10, 10, 5), (10, 2, 1), (10, 13, 1), (12, 10, 7), (5, 9, 5), (19, 0, 7), (13, 20, 7), (20, 0, 3), (20, 11, 3), (18, 5, 1), (20, 10, 7), (5, 20, 1), (7, 20, 3), (15, 9, 7), (9, 20, 5)}
    stairs = {0: {}, 2: {(18,19,2), (3,18,2), (6,17,2), (6,9,2)}, 4: {(18,19,4), (3,18,4), (6,17,4), (6,9,4)}, 6: {(18,19,6), (3,18,6), (6,17,6), (6,9,6)}, 8: {(18,19,8), (3,18,8), (6,17,8), (6,9,8)}, 10: {}}

    start = (7, 2, 1)
    grid = Grid3D(21,21,floors*2+1, stairs, obst)

    planners = ["a_star_3d", "dijkstra_3d", "gbfs_3d", "jps_3d"]
    # med dijkstra
    plot_goals(floor_1_goals, planners, plt, "Start Floor 1 → Goal Floor 1")
    # plot_goals(floor_2_goals, planners, plt, "Start Floor 1 → Goal Floor 2")
    # plot_goals(floor_3_goals, planners, plt, "Start Floor 1 → Goal Floor 3")
    plot_goals(floor_4_goals, planners, plt, "Start Floor 1 → Goal Floor 4")
    # plot_goals(floor_5_goals, planners, plt, "Start Floor 1 → Goal Floor 5")
    # uten dijkstra
    planners = ["a_star_3d", "gbfs_3d", "jps_3d"]
    plot_goals(floor_1_goals, planners, plt, "Start Floor 1 → Goal Floor 1")
    # plot_goals(floor_2_goals, planners, plt, "Start Floor 1 → Goal Floor 2")
    # plot_goals(floor_3_goals, planners, plt, "Start Floor 1 → Goal Floor 3")
    plot_goals(floor_4_goals, planners, plt, "Start Floor 1 → Goal Floor 4")
    # plot_goals(floor_5_goals, planners, plt, "Start Floor 1 → Goal Floor 5")
    plt.show()
