#!/usr/bin/env python3

import numpy as np
import time
import cv2
import math
from queue import PriorityQueue
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
import rospy

scale = 100 # Scale of the map to convert from m to pixels
R = 0.033 * scale  # Radius of the wheel in m
r = 0.105 * scale  # Radius of the robot in m
L = 0.160 * scale # Distance between the wheels in m
map_width, map_height = (6*scale), (2*scale)    # Map dimensions
threshold = 0.05 * scale # Threshold for the goal node

def obstacles(clearance):
    # Define the Obstacle Equations and Map Parameters
    eqns = {
        "Rectangle1": lambda x, y: np.logical_and(0.75*scale <= map_height - y - 1, map_height - y - 1 <= 2*scale) & np.logical_and(1.5*scale <= x, x <= 1.65*scale),
        "Rectangle2": lambda x, y: np.logical_and(0*scale <= map_height - y - 1, map_height - y - 1 <= 1.25*scale) & np.logical_and(2.50*scale <= x, x <= 2.65*scale),
        "Circle": lambda x, y: (x - (4*scale))**2 + (map_height - y - 1 - (1.1*scale))**2 <= (0.5*scale)**2
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
    return np.sqrt((goal_node[0]-current_node[0])**2 + (goal_node[1]-current_node[1])**2) <= threshold

# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, actions, current_node = [], [], goal_node
    action = parents[goal_node][1]
    while current_node != start_node:
        path.append(current_node)
        actions.append(action)
        current_node, action = parents[current_node]
    path.append(start_node)
    actions.append(parents[start_node][1])
    return path[::-1], actions[::-1]

# Define a function to calculate the euclidean distance
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2 = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def move_func(input_node, UL, UR, plot=False):
    t, dt = 0, 0.1 #Time step
    Xi, Yi, Thetai = input_node #Input point's coordinates
    Thetan = math.pi * Thetai / 180 #Convert end point angle to radian
    Xn, Yn = Xi, Yi #End point coordinates
    Cost=0
    valid = True # Flag to indicate if all points in the curve are valid nodes
    while t<1:
        t += dt
        X_prev, Y_prev = Xn, Yn
        linearx = 0.5*R * (UL + UR) * math.cos(Thetan) * dt
        lineary = 0.5*R * (UL + UR) * math.sin(Thetan) * dt
        lineartheta = (R / L) * (UR - UL) * dt
        Xn += linearx
        Yn += lineary
        Thetan += lineartheta
        if Thetan < 0:
            Thetan += 2*math.pi
        Cost += math.sqrt(math.pow(linearx,2)+math.pow(lineary,2))
        if (pixels[int(map_height - Yn - 1), int(Xn)] == [0, 0, 0]).all() or (pixels[int(map_height - Yn - 1), int(Xn)] == [192, 192, 192]).all():
            valid = False # Mark as invalid
            break
        if plot: cv2.arrowedLine(pixels, (int(X_prev), map_height - 1 - int(Y_prev)), (int(Xn), map_height - 1 - int(Yn)), (255, 0, 0), thickness=1)
    Thetan = (180 * (Thetan) / math.pi) % 360 #Convert back to degrees
    if valid:
        return (Xn,Yn, Thetan), Cost
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
    parent = {start_node: (None,None)}
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
            path, final_actions = backtrack_path(parent, start_node, approx_goal_node) # Backtrack the path
            if display_animation:
                for i in range(len(path) - 1):
                                    x1, y1, _ = path[i]
                                    x2, y2, _ = path[i+1]
                                    cv2.line(pixels, (int(x1), map_height - 1 - int(y1)), (int(x2), map_height - 1 - int(y2)), (0, 0, 255), thickness=2)
                cv2.imshow('Path', pixels)
                cv2.waitKey(2000)
                out.write(pixels)
            print("Final Cost: ", cost[goal_node])
            out.release()
            return path, final_actions

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
                        parent[new_node] = (current_node, action)  # Update parent
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

time.sleep(5)
# Get valid start and goal nodes from user input
while True:
    clearance = float(input("\nEnter the clearance (in m): "))
    print("Generating 2D map...")
    pixels = obstacles(clearance*scale)
    print("Map generated.")
    start_node = tuple(map(float, input("Enter the start node (in the format 'x y o')(in m): ").split()))
    gazebostart_node = start_node
    start_node = ((start_node[0]+0.5)*scale, (start_node[1]+1)*scale, start_node[2]%360)
    if not is_in_range(start_node):
        print("Error: Start node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    goal_node = tuple(map(float, input("Enter the goal node (in the format 'x y')(in m): ").split()))
    goal_node = ((goal_node[0]+0.5)*scale, (goal_node[1]+1)*scale)
    if not is_in_range(goal_node):
        print("Error: Goal node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    RPM1, RPM2 = map(int, input("Enter RPM1, RPM2 (in the format 'R1 R2'): ").split())
    break

actions=[[RPM1,RPM1], [0,RPM1], [RPM1,0], [RPM2,RPM2], [0,RPM2], [RPM2,0], [RPM1,RPM2], [RPM2,RPM1]]  # List of actions

# Run A* algorithm
start_time = time.time()
path, act = a_star(start_node, goal_node)
for i in range(len(path)):
    x, y, theta = path[i]
    path[i] = (x/scale - 0.5, y/scale - 1, theta)
if path is None:
    print("\nError: No path found.")
else:
    print("\nGoal Node Reached!\n\nShortest Path: ", path, "\n\nActions: ", act, "\n")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n\nStarting ROS simulation...")

#ROS Simulation
def get_orientation(msg, current):
    current['x'], current['y'] = msg.pose.pose.position.x, msg.pose.pose.position.y
    _, _, current['yaw'] = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w))

rospy.init_node('algorithm')
rospy.wait_for_service('/gazebo/set_model_state')   # Wait for the service to be available
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
state = ModelState(model_name='burger', reference_frame='world')
state.pose.position.x, state.pose.position.y, state.pose.position.z = gazebostart_node[0], gazebostart_node[1], 0   # Set the initial position of the robot
q = quaternion_from_euler(0, 0, gazebostart_node[2] * math.pi / 180)    # Set the initial orientation of the robot
state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = q  
set_model_state(state)
rospy.sleep(2)

current = {'x': 0, 'y': 0, 'yaw': 0}    # Dictionary to store the current position of the robot
odom_sub = rospy.Subscriber('/odom', Odometry, get_orientation, callback_args=current)  # Subscribe to the odom topic to get the current position of the robot

for point in path:
    goal_x, goal_y, _ = point
    pub, rate = rospy.Publisher('/cmd_vel', Twist, queue_size=10), rospy.Rate(10)   # Create a publisher and rate object
    reached_goal, twist = False, Twist()

    while not reached_goal and not rospy.is_shutdown():
        Deltax, Deltay = goal_x - current['x'], goal_y - current['y']   # Calculate delta x and delta y
        goal_angle, distance_to_goal = math.atan2(Deltay, Deltax), math.sqrt(Deltax**2 + Deltay**2) # Calculate goal angle and distance to goal

        if abs(goal_angle - current['yaw']) > 0.1:  # If the robot is not facing the goal, rotate
            twist.linear.x, twist.angular.z = 0.0, goal_angle - current['yaw']  # Rotate towards the goal
        else:
            twist.linear.x, twist.angular.z = min(0.2, distance_to_goal), 0.0   # Move towards the goal
            reached_goal = distance_to_goal < 0.05  # Check if the robot has reached the goal

        pub.publish(twist)
        rate.sleep()

print("Simulation complete. Reached Goal Node.\n\n")