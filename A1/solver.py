import math
import random
from collections import deque, defaultdict
import heapq
import numpy as np

random.seed(42)

###############################################################################
#                                Node Class                                   #
###############################################################################

class Node:
    """
    Represents a graph node with an undirected adjacency list.
    'value' can store (row, col), or any unique identifier.
    'neighbors' is a list of connected Node objects (undirected).
    """
    def __init__(self, value):
        self.value = value
        self.neighbors = []

    def add_neighbor(self, node):
        """
        Adds an undirected edge between self and node:
         - self includes node in self.neighbors
         - node includes self in node.neighbors (undirected)
        """
        if node not in self.neighbors:
            self.neighbors.append(node)
        if self not in node.neighbors:
            node.neighbors.append(self)
        pass

    def __repr__(self):
        return f"Node({self.value})"
    
    def __lt__(self, other):
        return self.value < other.value


###############################################################################
#                   Maze -> Graph Conversion (Undirected)                     #
###############################################################################

def parse_maze_to_graph(maze):
    """
    Converts a 2D maze (numpy array) into an undirected graph of Node objects.
    maze[r][c] == 0 means open cell; 1 means wall/blocked.

    Returns:
        nodes_dict: dict[(r, c): Node] mapping each open cell to its Node
        start_node : Node corresponding to (0, 0), or None if blocked
        goal_node  : Node corresponding to (rows-1, cols-1), or None if blocked
    """
    rows, cols = maze.shape
    nodes_dict = {}

    # 1) Create a Node for each open cell
    # 2) Link each node with valid neighbors in four directions (undirected)
    # 3) Identify start_node (if (0,0) is open) and goal_node (if (rows-1, cols-1) is open)

    # TODO: Implement the logic to build nodes and link neighbors
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == 0:
                nodes_dict[(r, c)] = Node((r,c))
    directions = [(-1,0), (1,0), (0,-1), (0,1)]
    
    for (r, c), node in nodes_dict.items():
        for dr, dc in directions:
            neighbor_pos = (r + dr, c + dc)
            #Check if open
            if neighbor_pos in nodes_dict:
                node.add_neighbor(nodes_dict[neighbor_pos])

    # TODO: Assign start_node and goal_node if they exist in nodes_dict
    start_node = nodes_dict.get((0, 0), None)
    goal_node = nodes_dict.get((rows - 1, cols - 1), None) #return none if not found

    return nodes_dict, start_node, goal_node


###############################################################################
#                         BFS (Graph-based)                                    #
###############################################################################

def bfs(start_node, goal_node):
    """
    Breadth-first search on an undirected graph of Node objects.
    Returns a list of (row, col) from start to goal, or None if no path.
    
    Steps (suggested):
      1. Use a queue (collections.deque) to hold nodes to explore.
      2. Track visited nodes so you don’t revisit.
      3. Also track parent_map to reconstruct the path once goal_node is reached.
    """
    # TODO: Implement BFS
    
    if not start_node or not goal_node:
        return None
    queue = deque([start_node])
    visited = {start_node}
    parent_map = {start_node: None}

    #since Node isnt an array cant traverse through it via for loop so use queue, pop left to take out/eval oldest
    while queue:
        current = queue.popleft()
        if current == goal_node:
            break
        for neighbor in current.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                parent_map[neighbor] = current  
                queue.append(neighbor)
    if goal_node not in parent_map:
        return None
    #reconstruct path
    return reconstruct_path(goal_node, parent_map)

###############################################################################
#                          DFS (Graph-based)                                   #
###############################################################################

def dfs(start_node, goal_node):
    """
    Depth-first search on an undirected graph of Node objects.
    Returns a list of (row, col) from start to goal, or None if no path.

    Steps (suggested):
      1. Use a stack (Python list) to hold nodes to explore.
      2. Keep track of visited nodes to avoid cycles.
      3. Reconstruct path via parent_map if goal_node is found.
    """
    # TODO: Implement DFS
    if not start_node or not goal_node:
        return None
        
    stack = [start_node] # use stack instead because we are using LIFO , aka normal python list using .pop()
    visited = {start_node}
    parent_map = {start_node: None}

    #since Node isnt an array cant traverse through it via for loop so use queue, pop left to take out/eval oldest
    while stack:
        current = stack.pop()
        if current == goal_node:
            break
        for neighbor in current.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                parent_map[neighbor] = current  
                stack.append(neighbor)
                
    if goal_node not in parent_map:
        return None
    #reconstruct path
    return reconstruct_path(goal_node, parent_map)


###############################################################################
#                    A* (Graph-based with Manhattan)                           #
###############################################################################

def astar(start_node, goal_node):
    """
    A* search on an undirected graph of Node objects.
    Uses manhattan_distance as the heuristic, assuming node.value = (row, col).
    Returns a path (list of (row, col)) or None if not found.

    Steps (suggested):
      1. Maintain a min-heap/priority queue (heapq) where each entry is (f_score, node).
      2. f_score[node] = g_score[node] + heuristic(node, goal_node).
      3. g_score[node] is the cost from start_node to node.
      4. Expand the node with the smallest f_score, update neighbors if a better path is found.
    """
    # TODO: Implement A*
    if not start_node or not goal_node:
        return None

    count = 0;
    g_score = {start_node: 0}
    f_score = {start_node: manhattan_distance(start_node, goal_node)}

    open_set = []
    heapq.heappush(open_set, (f_score[start_node], id(start_node), start_node)) #id to make sure heapq doesnt find two same f scores and try compare

    visited = set() # O(1) instead of O(n)
    parent_map = {start_node: None}

    while open_set:
        current_f, _, current = heapq.heappop(open_set)

        if current == goal_node:
            break

        if current in visited:
            continue
        visited.add(current)

        for neighbor in current.neighbors:
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                parent_map[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal_node)
                heapq.heappush(open_set, (f_score[neighbor], id(neighbor), neighbor))
    if goal_node not in parent_map:
        return None
    return reconstruct_path(goal_node, parent_map)

def manhattan_distance(node_a, node_b):
    """
    Helper: Manhattan distance between node_a.value and node_b.value 
    if they are (row, col) pairs.
    """
    # TODO: Return |r1 - r2| + |c1 - c2|
    r1,c1 = node_a.value
    r2,c2 = node_b.value
    return abs(r1-r2) + abs(c1-c2)


###############################################################################
#                 Bidirectional Search (Graph-based)                          #
###############################################################################

def bidirectional_search(start_node, goal_node):
    """
    Bidirectional search on an undirected graph of Node objects.
    Returns list of (row, col) from start to goal, or None if not found.

    Steps (suggested):
      1. Maintain two frontiers (queues), one from start_node, one from goal_node.
      2. Alternate expansions between these two queues.
      3. If the frontiers intersect, reconstruct the path by combining partial paths.
    """
    # TODO: Implement bidirectional search
    if not start_node or not goal_node:
        return None
    
    if start_node == goal_node:
        return [start_node.value]

    forward_queue = deque([start_node])
    backward_queue = deque([goal_node])

    forward_parent = {start_node: None}
    backward_parent = {goal_node: None}
    meeting_node = None

    #forwards
    while forward_queue and backward_queue:
        # --- Expand Forward ---
        curr_f = forward_queue.popleft()
        for neighbor in curr_f.neighbors:
            if neighbor in backward_parent:
                forward_parent[neighbor] = curr_f
                meeting_node = neighbor
                break
            if neighbor not in forward_parent:
                forward_parent[neighbor] = curr_f
                forward_queue.append(neighbor)
        
        if meeting_node:
            break
        #backwards
        curr_b = backward_queue.popleft()
        for neighbor in curr_b.neighbors:
            if neighbor in forward_parent:
                backward_parent[neighbor] = curr_b
                meeting_node = neighbor
                break
            if neighbor not in backward_parent:
                backward_parent[neighbor] = curr_b
                backward_queue.append(neighbor)
        
        if meeting_node:
            break
    if not meeting_node:
        return None
        
    path_forward = reconstruct_path(meeting_node, forward_parent)
    # meeting -> goal
    path_backward = reconstruct_path(meeting_node, backward_parent)
    path_backward.reverse()
    return path_forward[:-1] + path_backward #skip duplicate meeting


###############################################################################
#             Simulated Annealing (Graph-based)                               #
###############################################################################

def simulated_annealing(start_node, goal_node, temperature=1.0, cooling_rate=0.99, min_temperature=0.01):
    """
    A basic simulated annealing approach on an undirected graph of Node objects.
    - The 'cost' is the manhattan_distance to the goal.
    - We randomly choose a neighbor and possibly move there.
    Returns a list of (row, col) from start to goal (the path traveled), or None if not reached.

    Steps (suggested):
      1. Start with 'current' = start_node, compute cost = manhattan_distance(current, goal_node).
      2. Pick a random neighbor. Compute next_cost.
      3. If next_cost < current_cost, move. Otherwise, move with probability e^(-cost_diff / temperature).
      4. Decrease temperature each step by cooling_rate until below min_temperature or we reach goal_node.
    """
    # TODO: Implement simulated annealing
    if not start_node or not goal_node:
        return None

    current = start_node
    current_cost = manhattan_distance(current, goal_node)
    path_nodes = [current]

    temp = temperature
    while temp > min_temperature and current != goal_node:
        if not current.neighbors:
            break
        #cost of random number
        next_node = random.choice(list(current.neighbors))
        next_cost = manhattan_distance(next_node, goal_node)
        cost_diff = next_cost - current_cost

        #move is next cost < current always and move with probability V otherwise
        if cost_diff <0 or random.random() < math.exp(-cost_diff/temp):
            current = next_node
            current_cost = next_cost
            path_nodes.append(current)
            
        temp *= cooling_rate
        
    if current == goal_node:
        return [node.value for node in path_nodes]
        
    return None


###############################################################################
#                           Helper: Reconstruct Path                           #
###############################################################################

def reconstruct_path(end_node, parent_map):
    """
    Reconstructs a path by tracing parent_map up to None.
    Returns a list of node.value from the start to 'end_node'.

    'parent_map' is typically dict[Node, Node], where parent_map[node] = parent.

    Steps (suggested):
      1. Start with end_node, follow parent_map[node] until None.
      2. Collect node.value, reverse the list, return it.
    """
    # TODO: Implement path reconstruction
    path = []
    curr = end_node
    while curr is not None:
        path.append(curr.value)
        curr = parent_map[curr]

    path.reverse()
    return path


###############################################################################
#                              Demo / Testing                                 #
###############################################################################
if __name__ == "__main__":
    # A small demonstration that the code runs (with placeholders).
    # This won't do much yet, as everything is unimplemented.
    random.seed(42)
    np.random.seed(42)

    # Example small maze: 0 => open, 1 => wall
    maze_data = np.array([
        [0, 0, 1],
        [0, 0, 0],
        [1, 0, 0]
    ])

    # Parse into an undirected graph
    nodes_dict, start_node, goal_node = parse_maze_to_graph(maze_data)
    print("Created graph with", len(nodes_dict), "nodes.")
    print("Start Node:", start_node)
    print("Goal Node :", goal_node)

    # Test BFS
    # Similarly test DFS, A*, etc.
    path_dfs = dfs(start_node, goal_node)
    print("DFS Path:", path_dfs)

    path_astar = astar(start_node, goal_node)
    print("A* Path:", path_astar)

    path_bi = bidirectional_search(start_node, goal_node)
    print("Bidirectional Path:", path_bi)

    path_sa = simulated_annealing(start_node, goal_node)
    print("Simulated Annealing Path:", path_sa)
    
