import numpy as np
import time
import cv2
import math
from queue import PriorityQueue

scale = 50
R = 0.033 * scale  # Radius of the wheel in m
r = 0.105 * scale # Radius of the robot in m
L = 0.160 * scale # Distance between the wheels in m
map_width, map_height = 600, 250    # Map dimensions
threshold = 5 # Threshold for the goal node

def obstacles(clearance):
    # Define the Obstacle Equations and Map Parameters
    eqns = {
        "Rectangle1": lambda x, y: np.logical_and(0 <= y, y <= 100) & np.logical_and(100 <= x, x <= 150),
        "Rectangle2": lambda x, y: np.logical_and(150 <= y, y <= 250) & np.logical_and(100 <= x, x <= 150),
        "Hexagon": lambda x, y: np.logical_and((75/2) * np.abs(x-300)/75 + 50 <= y, y <= 250 - (75/2) * np.abs(x-300)/75 - 50) & np.logical_and(235 <= x, x <= 365),
        "Triangle": lambda x, y: np.logical_and((200/100) * (x-460) + 25 <= y, y <= (-200/100) * (x-460) + 225) & np.logical_and(460 <= x, x <= 510)
    }
    
    clearance = clearance + r
    y, x = np.meshgrid(np.arange(map_height), np.arange(map_width), indexing='ij')  
    is_obstacle = np.zeros_like(x, dtype=bool)  
    for eqn in eqns.values():   # Check if the point is an obstacle
        is_obstacle |= eqn(x, y)

    is_clearance = np.zeros_like(x, dtype=bool) # Check if the point is within the clearance
    for eqn in eqns.values():
        for c_x in np.arange(-clearance, clearance+0.5, 0.5):
            for c_y in np.arange(-clearance, clearance+0.5, 0.5):
                if (c_x**2 + c_y**2) <= clearance**2:
                    is_clearance |= eqn(x+c_x, y+c_y)

    pixels = np.full((map_height, map_width, 3), 255, dtype=np.uint8)
    pixels[is_obstacle] = [0, 0, 0]  # obstacle
    pixels[np.logical_not(is_obstacle) & np.logical_or.reduce((y < clearance, y >= map_height - clearance, x < clearance, x >= map_width - clearance), axis=0)] = [192, 192, 192]  # boundary
    pixels[np.logical_not(is_obstacle) & np.logical_not(np.logical_or.reduce((y < clearance, y >= map_height - clearance, x < clearance, x >= map_width - clearance), axis=0)) & is_clearance] = [192, 192, 192]  # clearance
    pixels[np.logical_not(is_obstacle) & np.logical_not(np.logical_or.reduce((y < clearance, y >= map_height - clearance, x < clearance, x >= map_width - clearance), axis=0)) & np.logical_not(is_clearance)] = [255, 255, 255]  # free space

    return pixels

# Define a function to check if current node is in range
def is_in_range(node):
    if node is None:
        return False
    if len(node) == 3:
        x, y, _ = node
    else:
        x, y = node
    y = map_height - y - 1
    return 0 <= x < map_width and 0 <= y < map_height and (pixels[int(y), int(x)] == [255, 255, 255]).all()

def is_valid_node(node, visited):
    if node is None:
        return False
    if not is_in_range(node):
        return False  # out of range
    x, y, _ = node
    y = map_height - y - 1
    if not (pixels[int(y), int(x)] == [255, 255, 255]).all():
        return False  # in obstacle space
    threshold_theta = math.radians(30)
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                neighbor_node = (x + i * threshold, y + j * threshold, k * threshold_theta)
                if neighbor_node in visited:
                    return False  # Too close to a visited node
    return True

# Define a function to check if current node is the goal node
def is_goal(current_node, goal_node):
    return np.sqrt((goal_node[0]-current_node[0])**2 + (goal_node[1]-current_node[1])**2) <= 5

# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, current_node = [goal_node], goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(start_node)
    return path[::-1]

# Define a function to calculate the euclidean distance
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2 = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def move_func(input_node, UL, UR, plot=False):
    t, dt = 0, 0.1 #Time step
    Xi, Yi, Thetai = input_node #Input point's coordinates
    Thetan = 3.14 * Thetai / 180 #Convert end point angle to radian
    Xn, Yn = Xi, Yi #End point coordinates
    Cost=0
    valid = True # Flag to indicate if all points in the curve are valid nodes
    while t<1:
        t += dt
        X_prev, Y_prev = Xn, Yn
        Dx = 0.5*R * (UL + UR) * math.cos(Thetan) * dt
        Dy = 0.5*R * (UL + UR) * math.sin(Thetan) * dt
        Xn += Dx
        Yn += Dy
        Thetan += (R / L) * (UR - UL) * dt
        if Thetan < 0:
            Thetan += 2*math.pi
        Cost += math.sqrt(math.pow(Dx,2)+math.pow(Dy,2))
        node = (Xn, Yn, Thetan)
        if (pixels[int(map_height - Yn - 1), int(Xn)] == [0, 0, 0]).all() or (pixels[int(map_height - Yn - 1), int(Xn)] == [192, 192, 192]).all():
            valid = False # Mark as invalid
            break
        if plot: cv2.arrowedLine(pixels, (int(X_prev), map_height - 1 - int(Y_prev)), (int(Xn), map_height - 1 - int(Yn)), (255, 0, 0), thickness=1)
    Thetan = (180 * (Thetan) / 3.14) % 360 #Convert back to degrees
    if valid:
        return (Xn, Yn, Thetan), Cost
    else:
        return None, float('inf')
    
# Define the A* algorithm
def a_star(start_node, goal_node, display_animation=True):
    rows = int(map_height / threshold)  # number of rows
    cols = int(map_width / threshold)   # number of columns
    angles = int(360 / 30)           # number of angles
    V = [[[False for _ in range(angles)] for _ in range(cols)] for _ in range(rows)]    # visited nodes matrix 

    open_list = PriorityQueue()
    closed_list = set()
    cost_to_come = {start_node: 0}
    cost_to_go = {start_node: euclidean_distance(start_node, goal_node)}
    cost = {start_node: cost_to_come[start_node] + cost_to_go[start_node]}
    parent = {start_node: None}
    open_list.put((cost[start_node], start_node))
    visited = set([start_node])
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 15.0, (map_width, map_height))

    while not open_list.empty():
        _, current_node = open_list.get()
        closed_list.add(current_node)
        x, y, theta = current_node
        V[int(y / threshold)][int(x / threshold)][int(theta / 30)] = True   # Mark current node as visited
        out.write(pixels)
        if display_animation:
            cv2.imshow('Path', pixels)
            cv2.waitKey(1)
        # Check if current node is the goal node
        if is_goal(current_node, goal_node):
            approx_goal_node = current_node # Approximate goal node (within threshold distance) 
            cost[goal_node] = cost [current_node]   # Cost of goal node
            path = backtrack_path(parent, start_node, approx_goal_node) # Backtrack the path
            if display_animation:
                for i in range(len(path) - 1):
                                    x1, y1, _ = path[i]
                                    x2, y2, _ = path[i+1]
                                    cv2.line(pixels, (int(x1), map_height - 1 - int(y1)), (int(x2), map_height - 1 - int(y2)), (0, 0, 255), thickness=2)
                cv2.imshow('Path', pixels)
                cv2.waitKey(0)
                out.write(pixels)

            print("Final Cost: ", cost[goal_node])
            out.release()
            cv2.destroyAllWindows()
            return path

        for action in actions:    # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, action[0], action[1])
            if is_valid_node(new_node, visited):    # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(new_node[2] / 30) # Get the index of the node in the 3D array
                if not V[i][j][k]: # Check if the node is in closed list
                    new_cost_to_come = cost_to_come[current_node] + move_cost
                    new_cost_to_go = euclidean_distance(new_node, goal_node)
                    new_cost = new_cost_to_come + new_cost_to_go    # Update cost
                    if new_node not in cost_to_come or new_cost_to_come < cost_to_come[new_node]:
                        cost_to_come[new_node] = new_cost_to_come   # Update cost to come
                        cost_to_go[new_node] = new_cost_to_go    # Update cost to go
                        cost[new_node] = new_cost   # Update cost
                        parent[new_node] = current_node  # Update parent
                        V[i][j][k] = True
                        open_list.put((new_cost, new_node)) # Add to open list
                        visited.add(new_node)   # Add to visited list
                        _, _ = move_func(current_node, action[0], action[1], plot=display_animation) # Plot the path

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break

    out.release()
    cv2.destroyAllWindows()
    return None

# Get valid start and goal nodes from user input
while True:
    clearance = int(input("\nEnter the clearance: "))
    pixels = obstacles(clearance)
    start_node = tuple(map(int, input("Enter the start node (in the format 'x y o'): ").split()))
    if not is_in_range(start_node):
        print("Error: Start node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    goal_node = tuple(map(int, input("Enter the goal node (in the format 'x y'): ").split()))
    if not is_in_range(goal_node):
        print("Error: Goal node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    RPM1 = int(input("Enter RPM1: "))
    RPM2 = int(input("Enter RPM2: "))
    break

actions=[[RPM1,RPM1], [0,RPM1], [RPM1,0], [RPM2,RPM2], [0,RPM2], [RPM2,0], [RPM1,RPM2], [RPM2,RPM1]]  # List of actions

# Run A* algorithm
start_time = time.time()
path = a_star(start_node, goal_node)
if path is None:
    print("\nError: No path found.")
else:
    print("\nGoal Node Reached!\nShortest Path: ", path, "\n")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n")