# Guide

## Code breakdown

### A Star Planner
The part of the code between the comment blocks "A star planner" and "A star Navigartor" is the A-Star Planner code. 

#### Functions in A Start Planner:
1. path_planner() : This function builds the final path by retracing the steps from the goal node back to the parent node.
2. astar() : This function is the core of the 'A Star planner'. Operation like creating child nodes, calculating their running costs, heuristic cost, and creating the path to goal node.
3. findIndex() : This function will return the index of the given element in a pathList.
4. heuristic() : This function will return the heuristic cost of the node. This is simply the Euclidean distance from the node to the end point.
5. isPointInOpenList() : This function is similar to findIndex() but returns a binary output if the element is found in the openPoints list.
6. removeFromOpenList() : This function will perform the pop operation to the list being built. It removes the given element and shifts the index of the subsequnt elements.
7. getIndexOfLowestFCost() : This function will return the index of the element with the lowest total cost in the list.
8. isPointInClosedList() :  This function is similar to findIndex() but returns a binary output if the element is found in the closedPoints list.
9. addToPathList() : This function will add element to pathList.
10. addToOpenList() : This function will add element to openList.

Apart from the functions, another important variable that is used is the "grid" variable. This is essentially the map definition of the explorable area. The grid is represented with 0s, for accessible areas, and 1s for non-accessible areas. 

### A Star Navigator
The part of the code after the "A Star Navigator" comment block is the A Star Navigator code.

This part of the code can be broken down into further parts like,
1. Perception : All the functions related to sensing the environment using the IR sensors and the Ultra Sonic sensors will fall into this category.  
2. Navigation : All the motion decision and command functions will fall into this category.  
3. User Interface : All functions related to Display on LCD, taking in user inputs using buttons etc. will fall into this category
   
#### Perception Functions
1. get_distance_side() : The function will obtain the distance of obstacles from the ultrsonic sensor
2. RCTime() : Used to sense the track using the IR Sensors
3. callibration() : Used to calibrate the color that IR Sensor will return 1 for.
4. ir_read() : This function is used to interpret the values obtained from the IR sensor. We compare the incoming duration with the calibrated value and convert the analog signal to digital one
5. mazeUpdate() : Used to update non-accessible areas in the maze based on the obstacles detected by ultrasonic sensor.

#### Navigation Functions

Motion command Functions : left_turn() and right_turn() generates motor command that would help the robot move in the desired direction.
Motion decision Function : basic_line_follower() uses multiple if else conditions to deduce the direction in which the robot should move in order to reach the required node. This is done by comparing the x,y coordinates of current node with that of the next one.

#### User Interface Functions:
1. display() : Used to display specific data and messages on the LCD Display hooked up.
2. acquire_target() : Used to take in the drop point for the widget from user.

