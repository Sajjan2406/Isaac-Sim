# Mobile_Manipiulator
Scene Setup:
Mobile Manipulator: Idealworks's  STR
Robot arm: Universla Robot's UR10
![alt text](<images/capture.2024-08-26 13.29.50.png>)
This Project leverages visual scripting through 'Action Graphs' to simulate the robot assembly inside a warehouse scene.

The robots are imported into the warehouse environment and are rigged  as per the documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_rigging_robot.html

After rigging of the robot, they are assembled referring : https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_assembling_robots.html

Procedure:
Building the graph: 
1. Add a new action graph from Windows tab or by right clicking and selectin Visual Scripting
2. Added 'Articulation Controller(AC)' and 'Differential Controller(DC)'
3. To tell the 'AC' what robot to control, STR robot is specified in the 'targetPrim' attribute
4. 'wheeldistance' and 'wheelRadius' are specified as '0.57926' and '0.14' respectively in the 'Differential Controller' action graph node.
5. 'AC' applies the driver commands(in the form of force, position, or velocity) to the specified joints  and 'DC' computes the drive commands 
6. The joints to be controlled(left_wheel_joint and right_wheel_joint)are passed to the 'AC' in the form of tokens

This is the basic setup.
Two different approach are implemented.

Keyboard Control:
1. Four 'on Keyboard Input' nodes are setup and parameters are set as 'W,A,S and D' to each node respectively
2. The output from the nodes are configured such that:
    a. Pressing 'W' moves the robot forward
    b. 'S' moves the robot in the reverse direction
    c. 'A' turns the robot in th Counter clockwise direction
    d. 'D' turns the robot in Clockwise Direction
3. Click Play to see the simulation

The Action Graph set up can be seen below: 
![alt text](<images/capture.2024-08-26 14.33.13.png>)  
The simulation can be seen here:  https://youtu.be/21WPmkZWuHA  
To have a 1st person POV, check this one out: https://youtu.be/zSoryHLmI6Q

Co-ordinate Controlled:  
Building the Graph:
1. A prim 'TargetXform' is added. This will be used as the target for the 'STR'
2. The action graph is setup like the image below:
![alt text](<images/Screenshot from 2024-08-27 10-46-11.png>)
3. The coordinates of the 'TargetXform' is manually passed through 'constant Vector3f' node.
4. This value is utilized by 'Quintic Path Planner' node and sends the signal to 'Stanley Control PID' node to steer to the target
5. 'Check Goal 2D' node checks if the goal is reahched and sends the message to PID controller
6. Odometry values are also passed to the PID control
7. All these info are processesd and sent to 'Diifernetial Controller' and later 'Articulation Controller' moves the STR to the target

The simulation can be seen here:  https://youtu.be/kyLfHsvM13Q

