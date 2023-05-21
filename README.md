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

At every node, A* path planner is invoked and the map is updated based on the ultrasonic sensor reading to block off unaccessable nodes. This forces the algorithm to generate alternative path that will get it to the destination. The child nodes are generated based on the accessible nodes from the given node to make sure that the lane constraints are met. 

Once the nodes are generated, we then generate the drive command by comparing the current and the next nodes and update the current node. If the command is a turn, we update both current node and orientation of the robot. This way, we can keep track of vertical lanes and accomodate the lane conditions.

## **Results**
![Video](https://drive.google.com/file/d/1150Cy5POfgab_w2hkNcj7LKvisDDdG4a/view?usp=share_link)
