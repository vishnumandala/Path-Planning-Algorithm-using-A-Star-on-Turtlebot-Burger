# PROJECT 3 - Phase 2

## ENPM661 - Planning for Autonomous Robots
### Hamza Shah Khan: 119483152 | hamzask@umd.edu
### Vishnu Mandala: 119452608 | vishnum@umd.edu

## Part 1: 2D Implementation

Part 1 of this project implements a path planning algorithm using A* algorithm on a differential drive TurtleBot3 robot, to find the optimal path from a start node to a goal node in a 2D environment with obstacles. The obstacles are defined using half-plane equations, and the map parameters include width, height, radius and clearance.

The program first creates a 2D map of the environment with the obstacles marked as black and the free space marked as white. The boundaries and clearances are marked as gray. The start and goal nodes are also defined on this map.

The program then uses A* algorithm to find the optimal path from the start node to the goal node considering the differential drive constraints of the Turtlebot3 robot while also avoiding the obstacles. The algorithm is visualized using animation, with each visited node marked in blue and the optimal path marked in red. The final cost of the path is also displayed.

### **Dependencies**

1. python 3.11 (any version above 3 should work)
2. Python running IDE (I used VS Code)

### **Libraries**
* NumPy
* Queue
* Time
* OpenCV
* Math

### **Contents**

* Part 1
  * proj3p2_hamza_vishnu.py
* Part 2
  * proj3p2part2
    * src	
      * proj3p2part2.py		
    * launch	
      * astar_turtlebot.launch			
    * world	
      * map.world			
* proj3p2_hamza_vishnu.pdf
* README.md

### **Instructions**
1. Download the zip file and extract it
2. Install python and the required dependencies: 
	`pip install numpy opencv-python`
3. Run the code:
	`$python3 Part01/proj3p2_hamza_vishnu.py`
4. Type in the Clearance Value, Start Node(x y 0) and Goal Node(x y) and the Wheel RPM1 and RPM2
5. The optimal path will be displayed on the screen and saved to a video file named animation.mp4 in the same directory as the program. The final cost of the path will be displayed in the console.

### **Example Output**

Enter the clearance: 3
Generating Map...
Enter the start node (in the format 'x y o'): 20 20 30
Enter the goal node (in the format 'x y'): 580 30
Enter RPM1: 5
Enter RPM2: 10
Final Cost:  664.3138513011415

Goal Node Reached!

Shortest Path:  [(20, 20, 30), (35.72076941345914, 29.070827362972288, 30.0), (42.26259834755264, 40.18903795081577, 95.02786624203819), (47.625138385156, 51.92161212802336, 30.0), (54.1669673192495, 63.03982271586683, 95.02786624203819), (59.52950735685286, 74.77239689307446, 30.0), (66.07133629094633, 85.89060748091796, 95.02786624203819), (71.43387632854966, 97.62318165812557, 30.0), (79.2942610352793, 102.15859533961168, 30.0), (95.01503044873844, 111.22942270258405, 30.0), (99.31490441136336, 111.26256822086647, 325.1547309848169), (103.59534663003146, 110.85289868027837, 30.182597226855364), (107.89509320054447, 110.89974042665413, 325.3373282116723), (120.74026947475298, 109.71164154058074, 30.36519445371073), (125.03984502718045, 109.77217903978534, 325.51992543852765), (137.88874059020918, 108.62500194405989, 30.547791680566093), (150.78682332051963, 108.84769985748848, 325.702522665383), (158.26912261001243, 103.71244695792697, 325.702522665383), (171.12160709474105, 102.60620329200711, 30.730388907421457), (184.01891503189748, 102.86998436171882, 325.8851198922383), (199.0161523547165, 92.64719742844855, 325.8851198922383), (214.0133896775355, 82.42441049517828, 325.8851198922383), (229.01062700035453, 72.20162356190801, 325.8851198922383), (244.00786432317355, 61.97883662863774, 325.8851198922383), (259.0051016459926, 51.756049695367466, 325.8851198922383), (271.86104464888683, 50.69075068339304, 30.912986134276764), (276.15984541078075, 50.792371199935175, 326.0677171190937), (291.18956931286533, 40.61740685468509, 326.0677171190937), (304.04884039530003, 39.59306330536447, 31.095583361132128), (316.9442061803752, 39.93900224172742, 326.2503143459491), (329.80667486995833, 38.955624548227696, 31.278180587987492), (342.70087331581135, 39.34263736137087, 326.4329115728044), (355.56640910770693, 38.40023550121251, 31.460777814842857), (368.4593093875668, 38.82831826443681, 326.6155087996598), (381.32778174582006, 37.92690179939236, 31.64337504169822), (394.2192530460864, 38.396050169295954, 326.7981060265152), (407.0905314049469, 37.535628245293026, 31.825972268553585), (419.9804429265188, 38.045837461815246, 326.9807032533705), (427.575374350322, 33.078681165932196, 326.9807032533705), (442.7652371979279, 23.144368574166137, 326.9807032533705), (457.95510004553375, 13.210055982400082, 326.9807032533705), (470.82905381078155, 12.390637329428579, 32.00856949540889), (478.5257329317975, 17.198626046981484, 32.00856949540889), (491.41395389139956, 17.749890933451205, 327.1633004802258), (504.2904524416695, 16.97148386547474, 32.191166722264256), (517.1768520731787, 17.56379882866294, 327.3458977070812), (530.0557647612852, 16.826411243534505, 32.37376394911962), (537.7216580642385, 21.683334638826572, 32.37376394911962), (550.6061056200117, 22.316693669003023, 327.5284949339366), (563.4873017742758, 21.620333048384794, 32.556361175974985), (578.7880690671005, 31.38296692715053, 32.55636117597499), (578.7880690671005, 31.38296692715053, 32.55636117597499)]     

Runtime: 187.375009059906 seconds

### *Output*
![A* on Differential Drive Robot](https://github.com/vishnumandala/Path-Planning-Algorithm-using-A-Star-on-Turtlebot-Burger/blob/main/Part%201/part1_output.png)

### **Animation**

https://drive.google.com/file/d/1d204RiwlYIIe02Jf-8912CeFpOY6TO14/view?usp=share_link

## Part 2: Gazebo Visualization

This part of the project is to simulate the path planning implementation on Gazebo with the TurtleBot3 Burger robot.

The output video shows the TurtleBot motion in Gazebo environment. The motion of 
the TurtleBot is a result of the solution generated by the A* algorithm, implemented in Part01, and the 
corresponding velocity commands published to the appropriate ROS topic. 

### **Dependencies**

* Ubuntu 20.04
* ROS Noetic
* Gazebo
* catkin
* Turtlebot3 packages

### **Libraries**

* math
* numpy
* opencv
* time
* matplotlib
* queue
* rospy
* geometry_msgs
* tf.transformations
* gazebo_msgs.msg
* gazebo_msgs.srv
* nav_msgs.msg

### **Instructions**
1. Download the zip file and extract it
2. Install python and the required dependencies:
	`$pip install numpy opencv-python`
3. Navigate to Part2/proj3p2part2, open terminal in this folder and run the command
	  `$catkin_make`
	  to initialize it as a ROS workspace. Then source this ROS workspace using "source".
	  `$source catkin_ws/devel/setup.bash`
	  Once this is done, Start the ROS Master by running
	  `$roscore`
	  run the launch file in another terminal using the command 
	  `$roslaunch proj3p2part2 astar_turtlebot.launch`
	  and run the ROS executable file in another terminal using the command
	  `$rosrun proj3p2part2 algorithm.py`
4. Type in the Clearance Value, Start Node(x y 0) and Goal Node(x y) and the Wheel RPM1 and RPM2 when asked by the program in the terminal
5. The optimal path calculated using A* is displayed. The final cost of the path will be displayed in the console. Close the path display window to start the Gazebo Simulation.

### **Example Output**

~$ rosrun proj3p2part2 algorithm.py 

Enter the clearance (in m): 0.01
Generating 2D map...
Map generated.
Enter the start node (in the format 'x y o')(in m): 0 0 30
Enter the goal node (in the format 'x y')(in m): 5 0
Enter RPM1, RPM2 (in the format 'R1 R2'): 5 10
Final Cost:  602.5932457259098

Goal Node Reached!

Shortest Path:  [(0.0, 0.0, 30.0), (0.2579919007751933, 0.0020572131231002544, 325.00510011484715), (0.5553626250804788, -0.2061245640000413, 325.00510011484715), (0.8527333493857643, -0.414306341123183, 325.00510011484715), (1.109566398074471, -0.4388183400766602, 30.000000000000114), (1.2667500088613468, -0.34806834007666, 30.00000000000011), (1.5811172304350989, -0.16656834007665933, 30.00000000000011), (1.7118947796482993, 0.05583059340374197, 94.99489988515283), (1.8190832902014042, 0.2905105375763066, 30.000000000000103), (2.133450511775156, 0.47201053757630707, 30.000000000000103), (2.219447812033555, 0.4726962752840078, 325.0051001148472), (2.476280860722262, 0.4481842763305308, 30.00000000000017), (2.790648082296014, 0.6296842763305315, 30.00000000000017), (3.1050153038697657, 0.8111842763305321, 30.00000000000017), (3.3630072046449597, 0.8132414894536333, 325.00510011484727), (3.619840253333667, 0.7887294905001565, 30.00000000000017), (3.7058375535920653, 0.789415228207857, 325.00510011484727), (3.8545229157447087, 0.6853243396462878, 325.0051001148472), (4.151893640049994, 0.477142562523146, 325.0051001148472), (4.44926436435528, 0.26896078540000445, 325.0051001148472), (4.7466350886605655, 0.06077900827686289, 325.0051001148472), (5.003468137349273, 0.03626700932338611, 30.00000000000017)]

Actions:  [None, [10, 10], [10, 10], [5, 10], [5, 5], [10, 10], [5, 10], [10, 5], [10, 10], [5, 0], [5, 10], [10, 10], [10, 10], [10, 5], [5, 10], [5, 0], [5, 5], [10, 10], [10, 10], [10, 10], [5, 10], [5, 10]]

Runtime: 21.750512838363647 seconds

Starting ROS simulation...
Simulation complete. Reached Goal Node.

### *Output*
![A* on Turtlebot3 Burger](https://github.com/vishnumandala/Path-Planning-Algorithm-using-A-Star-on-Turtlebot-Burger/blob/main/Part%202/proj3p2part2/part2_output.png)

### **Animation**

https://drive.google.com/file/d/1fZ7YNa50fs31JxDoAsgtxc9kvh9STm9Q/view?usp=share_link

### **Gazebo Simulation**

https://drive.google.com/file/d/1GjYFWi2oQGLZSQk5o0g2MLNK1YDFE3it/view?usp=share_link


