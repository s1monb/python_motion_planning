from environment.env3d import Grid3D
from collections import defaultdict

def generate_layer_views(grid):
    """Organize obstacles by z-layer into 2D sets of (x, y) positions."""
    layers = defaultdict(set)
    for x, y, z in grid.obstacles:
        layers[z].add((x, y))
    return layers

def print_layer(z, grid):
    obstacle_set = generate_layer_views(grid)
    print(f"Layer {z}:")
    for y in range(grid.y_range):
        row = ''
        for x in range(grid.x_range):
            row += 'X ' if (x, y) in obstacle_set[z] else '. '
        print(row)
    print()
    

def print_all_layers(grid):
    """Prints all layers from top (z=0) to bottom (z=depth-1)."""
    for z in range(grid.z_range):
        print_layer(z, grid)


grid = Grid3D(10,8,3)
obstacles = grid.obstacles
obstacles.add((5,5,0))
obstacles.add((5,5,1))
obstacles.add((5,5,2))
print_all_layers(grid)


