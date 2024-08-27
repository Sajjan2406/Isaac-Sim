# Mobile_Manipiulator
## Scene Setup:
**Mobile Manipulator**: Idealworks's  STR
**Robot arm**: Universla Robot's UR10
![alt text](<images/capture.2024-08-26 13.29.50.png>)
This Project leverages visual scripting through 'Action Graphs' to simulate the robot assembly inside a warehouse scene.

The robots are imported into the warehouse environment and are rigged  as per the documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_rigging_robot.html

After rigging of the robot, they are assembled by following this guide : https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_assembling_robots.html

## Procedure:
### Building the graph: 
1. Add a new action graph from Windows tab or by right clicking and selectin Visual Scripting
2. Add 'Articulation Controller(AC)' and 'Differential Controller(DC)' nodes.
3. Specify the STR robot in the 'targetPrim' attribute of the 'AC' to tell it which robot to control.
4. In the 'Differential Controller', set 'wheelDistance' to `0.57926` and 'wheelRadius' to `0.14`.
5. 'AC' applies the driver commands(in the form of force, position, or velocity) to the specified joints  and 'DC' computes the drive commands 
6. Pass the joints to be controlled (`left_wheel_joint` and `right_wheel_joint`) to the 'AC' in the form of tokens.

This is the basic setup.  

Two different approach are implemented.

### Keyboard Control:
1. Set up four 'on Keyboard Input' nodes and assign the keys 'W', 'A', 'S', and 'D' to each node respectively.
2. Configure the output from the nodes such that:
    - Pressing 'W' moves the robot forward.
    - Pressing 'S' moves the robot in the reverse direction.
    - Pressing 'A' turns the robot counterclockwise.
    - Pressing 'D' turns the robot clockwise.
3. Click Play to see the simulation


![Action Graph Keyboard Control](<images/capture.2024-08-26 14.33.13.png>)  
Watch the simualtion here:  https://youtu.be/21WPmkZWuHA  
For a 1st person POV, check this one out: https://youtu.be/zSoryHLmI6Q

### Co-ordinate Controlled:  

1. A prim 'TargetXform' is added. This will be used as the target for the 'STR'
2. The action graph is setup like the image below:
![Action Graph Coordinate Control](<images/Screenshot from 2024-08-27 10-46-11.png>)
3. The coordinates of the 'TargetXform' is manually passed through 'constant Vector3f' node.
4. The `Quintic Path Planner` node utilizes this value and sends the signal to the `Stanley Control PID` node to steer to the target.t
5. 'Check Goal 2D' node checks if the goal is reahched and sends the message to PID controller
6. Odometry values are also passed to the PID control
7. All this information is processed and sent to the `Differential Controller`, which then passes the commands to the `Articulation Controller` to move the STR to the target.
8. 
The simulation can be seen here:  https://youtu.be/kyLfHsvM13Q

