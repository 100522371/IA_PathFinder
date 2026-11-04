# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

def h1(current_node, objective_node) -> np.float32:

    n_steps = (
        abs(float(current_node[0]) - float(objective_node[0]))
        + abs(float(current_node[1]) - float(objective_node[1]))
    )

    return np.float32(EPSILON * n_steps)


def h2(current_node, objective_node) -> np.float32:
    distance = (
        (float(current_node[0]) - float(objective_node[0])) ** 2
        + (float(current_node[1]) - float(objective_node[1])) ** 2
    ) ** 0.5

    return np.float32(EPSILON * distance)

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """ Builds an adjacency graph (not an adjacency matrix) from the detection map """
    # The only possible connections from a point in space (now a node in the graph) are:
    #   -> Go up
    #   -> Go down
    #   -> Go left
    #   -> Go right
    # Not every point has always 4 possible neighbors
    # ...
    # WE WILL ONLY ADD THEM IF the deterction probab of going there is < threshold AND ONLY ADD EDGES IF THE DETECTION PROBAB < thresh
    height, width = detection_map.shape
    G = nx.DiGraph()  # Directed graph
    provisional = []
    # the nodes must be array too
    for i in range(0, height):
        for j in range(0, width):
            current_node = (np.float32(i), np.float32(j)) # # our graph uses (x-coord, y-coord) ie (lat-converted, lon-converted) like  detection map
            current_cost = detection_map[i, j] # inverted

            # Add the node only if detection is below the threshold
            if current_cost < tolerance:
                G.add_node(current_node) 
                provisional.append(current_node)
            else:
                # debugging
                print(f"current_node {current_node} has current_cost {current_cost}\n")

    for i in range(0, height):
        for j in range(0, width):
            # Check each of the 4 neighbors
            current_node = (np.float32(i), np.float32(j)) # # we save it inverted
            if current_node in list(G.nodes):
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni = i + dx     # i + 1 is the right movement, i + 0, i - 1
                    nj = j + dy
                    # Make sure neighbor is within bounds
                    if 0 <= ni < height and 0 <= nj < width:
                        neighbor_cost = detection_map[ni, nj] # inverted
                        if neighbor_cost < tolerance:
                            neighbor_node = (np.float32(ni), np.float32(nj))

                            # You can set edge weight to the destination cost (or average, or 1, depending on your needs)
                            G.add_edge(current_node, neighbor_node, weight=neighbor_cost)
    print("these are the nodes of G generated: ")
    print(G.nodes)
    print("\n")
    return G

def discretize_coords(high_level_plan: np.ndarray,
                      boundaries: Boundaries,
                      map_width: int,
                      map_height: int) -> list[tuple[int, int]]:
    """Convierte coordenadas geográficas en índices de matriz."""

    lat_range = boundaries.max_lat - boundaries.min_lat
    lon_range = boundaries.max_lon - boundaries.min_lon

    if lat_range <= 0 or lon_range <= 0:
        raise ValueError("Los límites geográficos no son válidos.")

    coordinates = []

    for latitude, longitude in high_level_plan:
        row = round(
            (boundaries.max_lat - latitude)
            / lat_range
            * (map_height - 1)
        )
        column = round(
            (longitude - boundaries.min_lon)
            / lon_range
            * (map_width - 1)
        )

        row = max(0, min(map_height - 1, row))
        column = max(0, min(map_width - 1, column))

        coordinates.append((row, column))

    return coordinates

def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array,
                 initial_location_index: np.int32,
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """ Implementation of the main searching / path finding algorithm """
    ...
    solution_plan = []
    # 1) decide a "high level plan"
    high_level_plan = decide_HLP(locations, initial_location_index)
    HLP_coords = discretize_coords(high_level_plan, boundaries, map_width, map_height) # to know which specific cells we must go to
    print("HLP_coords")
    print(HLP_coords)
    print("\n")
    
    # 2) A* algo in each of the pairs    Python networkx library, called 
    num_pairs = len(HLP_coords) - 1
    for i in range(0, num_pairs):
        print("this is HLP's POI " + str(i) + ": " + str(HLP_coords[i]) ) # debug
        
        # check if the first node HLP_coords[0] is actually not included bc it had probability of detection > tolerance
        if i == 0 and HLP_coords[i] not in list(G.nodes):
            print(f"HLP_coords[{i}] = {HLP_coords[i]} WHILE G.nodes = {G.nodes}")
            print("starting point is not part of the graph bc of detection probab too high")
        

        try:
            astar_path = nx.astar_path(G, HLP_coords[i], HLP_coords[i+1], heuristic=heuristic_function, weight='weight')
            solution_plan.append(astar_path)
        except nx.NetworkXNoPath as e:
            #path_as_strings = [] 
            print(f"NetworkXNoPath ERROR:!!!!!!!!!!!!! {e}") # i am not sure we can do this
            solution_plan = []
            break
        except nx.NodeNotFound as e:
            #path_as_strings = [] 
            print(f"NodeNotFound ERROR:!!!!!!!!!!!!! {e}")
            solution_plan = []
            break

    return solution_plan, NODES_EXPANDED

def decide_HLP(locations: np.ndarray,
               initial_location_index: int) -> list:
    """Genera una ruta escogiendo siempre el POI no visitado más cercano."""

    if not 0 <= initial_location_index < len(locations):
        raise IndexError("El índice inicial no es válido.")

    route = [locations[initial_location_index]]
    unvisited = [
        index for index in range(len(locations))
        if index != initial_location_index
    ]

    current_index = initial_location_index

    while unvisited:
        next_index = min(
            unvisited,
            key=lambda index: manhattan_distance(
                locations[current_index],
                locations[index]
            )
        )

        route.append(locations[next_index])
        unvisited.remove(next_index)
        current_index = next_index

    return route

def manhattan_distance(c1, c2):
    """auxiliary"""
    return abs(c1[0]-c2[0]) + abs(c1[1]-c2[1])

def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """ Computes the total cost of the whole planning solution """
    # ...
    print("[@compute_path_cost] This is solution_plan " + str(solution_plan))
    cost = 0
    
    for i in range(0, len(solution_plan)):
        num_pairs = len(solution_plan[i]) - 1
        for j in range(num_pairs):
            current_node = solution_plan[i][j]
            next_node = solution_plan[i][j + 1]
            cost += G[current_node][next_node]["weight"]
    return cost
