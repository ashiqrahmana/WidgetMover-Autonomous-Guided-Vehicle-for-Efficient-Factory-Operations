# WidgetMover-Autonomous-Guided-Vehicle-for-Efficient-Factory-Operations

## **Problem Statement**
Given a layout with 3 lanes S, A, and B and specific traversal rules, the problem involves discovering a widget in lane A and moving it to a specific location at lane B. 

![Arena](https://github.com/ashiqrahmana/WidgetMover-Autonomous-Guided-Vehicle-for-Efficient-Factory-Operations/blob/main/Arena.jpg)

### **Scenario**
The robot starts at the home location S. 
There are a total of 15 intersections labelled i1, …, i5 (central intersections); A1, …, A5 (upper intersections); and B1, …, B5 (lower intersections). 
The green arrows represent the directions the robot is allowed to move on a given path. 
The lanes S, A, and B are unidirectional (direction allowed to move is shown in figure). 
The ‘A’ lane is called the pickup lane, where the robot must pick up the widget from a machine location at A1, …, A5. 
The ‘B’ lane is called the drop-off lane, where the robot must drop the partially processed widget at a machine location B1, …, B5. 
One or more obstacles may be introduced dynamically while the robot is traversing through the arena. As an example, an obstacle may be introduced while the robot is moving in the center lane, in response the robot must change its course accordingly. In Figure 1, the red box between i3 and i4 depicts an obstacle. 
The minimum height of the objects used (representing some artifact on the shop floor) will be 20 centimetres. 
The desired pickup and drop-off locations will be randomly assigned. In the figure above, the blue box at A4 represents a pickup location and the blue box at B3 represents a drop-off location. 

## **Hardware**

**Controller** : Parallax Propeller

**Sensors** : 
| Sensor | Model | Use |
|--------|-----|----|
| IR Sensor | QTI | For Line following |
| Ultra sonic Sensor | HC-SR04 | For Obstacle detection |

**Actuator**
Parallax Continious Servo

**Circuit Diagram**
![Circuit](https://github.com/ashiqrahmana/WidgetMover-Autonomous-Guided-Vehicle-for-Efficient-Factory-Operations/blob/main/Circuit.png)

## **Approach**
Since the problem involves navigating from one node to another, the A* algorithm is employed. This approach helps avoid any hardcoded navigation as the algorithm will automatically generate the next set of nodes that has to be traversed to reach the final node. 

In order to define the area of operation, it is important to set a predetermined map. This is done using the grid variable defined as 2D matrix of 0s and 1s where 0s indicate availability of node and 1s indicate the node is not accessible. 

Each element in the 2D array indicate an intersection in the map. Since the robot will know if there is obstacle present in the next segment because of the ultrasonic sensor, we force the next intersection to 1 blocking access to that intersection, i.e.. if obstacle found intersection i, (i+1)th intersection is updated as 1, thereby forcing the A-Star to generate a path through an alternate route. This blocked intersection is released by the robot immediately after the alternate route is taken. The rational behind this being the path between intersection i and (i+1) is what is not accessible and not the (i+1)th intersection as such. 

Another aspect of the A* algorithm that was crucial was generating the child nodes. Since there were lane constraints, this will be the deciding factor to determine the child nodes in certain lanes. Eg. in lane S only forward movements and turns are possible, so if current node is (x,y) then the possible child nodes would be, (x+1,y), (x,y+1), (x,y-1). Also, in order to honor the lane speed parameter, we also employ a higher cost on Lane A,B so that the robot will always prefer the S lane as that will optimize time as well alongside the distance which is the primary optimization objective.

Once the nodes are generated, we then back propogate from the final node to reach the initial node and invert it to provide the path. This generated path is then converted to drive command by comparing the current and the next nodes and updating the current node once the drive command is executed. If the command is a turn, we update both current node and orientation of the robot. This way, we can keep track of vertical lanes and accomodate the lane conditions.


## **Results**
[Video](https://youtu.be/_2NTl6i4DrA)
