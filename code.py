import heapq

class Node():
    def __init__(self, position, cost, heuristic):
        self.position = position
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None  # Add a parent attribute to track the path

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def astar_search(graph, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(start, 0, heuristic(start, goal))
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            # Path found, reconstruct and return it
            path = []
            while current_node:
                path.insert(0, current_node.position)
                current_node = current_node.parent
            return path

        closed_set.add(current_node.position)

        for neighbor in graph[current_node.position]:
            if neighbor not in closed_set:
                cost = current_node.cost + graph[current_node.position][neighbor]
                heuristic_val = heuristic(neighbor, goal)
                new_node = Node(neighbor, cost, heuristic_val)
                new_node.parent = current_node

                # Check if the neighbor is already in the open set with a lower cost
                existing_node = next((node for node in open_set if node.position == neighbor), None)
                if existing_node and existing_node.cost <= cost:
                    continue

                heapq.heappush(open_set, new_node)

    return None  # No path found

def heuristic(node, goal):
    # Simple Manhattan distance as the heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

graph = \
    {(0, 0): {(0, 1): 2, (1, 0): 4},
    (0, 1): {(0, 0): 2, (1, 1): 5, (0, 2): 3},
    (1, 0): {(0, 0): 4, (2, 1): 2},
    (1, 1): {(0, 1): 5, (2, 1): 8, (2, 2): 1},
    (0, 2): {(0, 1): 3, (1, 2): 6},
    (1, 2): {(0, 2): 6, (2, 2): 8},
    (2, 1): {(1, 0): 2, (1, 1): 8},
    (2, 2): {(1, 1): 1, (1, 2): 8}
}

start_node = (0, 0)
goal_node = (2, 2)

path = astar_search(graph, start_node, goal_node)

if path:
    print("Path found:", path)
else:
    print("No path found")
