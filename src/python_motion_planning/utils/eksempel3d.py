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

# Obst er alle obstacles-ene for de faktiske etasjene
obst = {(5,5,0), (5,5,2), (3,2,0)}

# stairs er en dict som for hver mellom etasje,
# så har du en set med nodene hvor du kan gå opp eller ned.
# Så for eksempel siden eksemplet her har 2 etasjer. Har vi 3 mulige lag.
# Laget med indeks 1 vil være mellom etasjen.
# Så for å komme seg til lag 0 til lag 2, må du være på 4, 4, 0 eller 3, 3, 0
stairs = {1: {(4,4,1),(3,3,1)}}

obst = {(5,5,1), (5,5,3), (3,2,1)}

# stairs er en dict som for hver mellom etasje,
# så har du en set med nodene hvor du kan gå opp eller ned.
# Så for eksempel siden eksemplet her har 2 etasjer. Har vi 3 mulige lag.
# Laget med indeks 1 vil være mellom etasjen.
# Så for å komme seg til lag 0 til lag 2, må du være på 4, 4, 0 eller 3, 3, 0
stairs = {0: {}, 2: {(4,4,2),(3,3,2)}, 4: {}}

grid = Grid3D(11,11,5, stairs, obst)

print_all_layers(grid)


