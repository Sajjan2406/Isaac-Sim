# Franka_Robot

This is a part of my Master thesis. 

## Aim: 
1. To analyse the performance of Nvidia Cumotion motion planning algorithms 
2. To integrate NVIDIA Issac Sim(physics based smulation platform) and ROS2(a robotics middleware framework)


Cumotion or Curobo is a CUDA accelerated library, developed by NVIDIA, containing a suite of robotics algorithms leveraging parallel compute.

This project is to uses Isaacsim simulation platform to manipulate 'Franka Panda' robot through ROS2 middleware.

## Integration of IssacSim with ROS2
One of the recommended ways to execute the trajectories calulated by moveit on to the simulation software is to use ros2_control framework. This framework connects ‘Control Alogirthms’ with ‘Hardware Drivers’. To make ros2_control communicate with ‘IsaacSim’, ‘topic_based_ros2_control’ plugin is introduced.This ‘topic_based_ros2_control’ plugin provides a hardware interface, of ’system’ type component,that subscribes and publishes on configured topics to recieve the control commands and publishe the current state of the robot. It suppports command and state interfaces through ROS2 topic communication layer.  ros2_control publishes the ‘joint_commands’, recieved from moveit2 froamework,on to
‘isaacs_joint_commands’ topic and tracks the ‘joint_states’ from ‘isaacs_joint_states’ topic.

IsaacSim environmemnt is loaded with enabling the ROS2 bridge paackge through the ’omni.isaac.ros2_bridge’ extension. Inside the IsaacSim environment a OmniGraph(as explained in the
section 5.1) is configured to communicate with ROS2. The Figure 9 explins the Action Graph contruction.

## Integration of Cumotion pipeline with Moveit2 framework
The cumotion pipeline is appended to the to ‘planning pipelines’, along with other pipelines, of Moveit2 during its launch. To access the capabilites of cuMotion planner, a ‘CumotionAc-
tionServer’ is launched along side the move_group node. This server acts a an action server for moveit listenig to the planning requests coming from the Moveit Clients
## Scene setup:
1. Isaac Sim GUI with Franka Panda robot.
2. RViz GUI with Franka Panda robot.
![alt text](<images/Screenshot from 2024-08-26 16-07-06.png>)
The scene is setup by referencing this tutorial : https://nvidia-isaac-ros.github.io/concepts/manipulation/cumotion_moveit/tutorial_isaac_sim.html

## Procedure:
1. NVIDIA Isaac Ros repository is cloned into the workspace
2. Docker container is built and dependencies are installed
3. A python script is used to spwan the Franka Panda Robot along with the necessary ROS2 action graphs, to communicate with ROS2 topics. 
4. Inside the docker container, differnet nodes like move_group, rviz, tf_pubisher and so on are launched specifying the planning pipelines and required config files (urdf, .yaml etc.).
5. In another terminal, inside the container,  'cumotion_planner node' is launched
6. A custom code is run, to communicate with 'move_group' and 'MoveitInterface', to Manipulate the robot.
7. The robot motion is visualized in Isaac Sim and Rviz GUI

**Planning with RViz GUI**:
1. Select the planning pipeline and the planner id from Context Tab
2. Set the goal pose by dragging the robot arm and click plan.
3. The generated plan can be seen in the GUI
4. Click execute to execute the generated tarjectory in both RViz GUI and IsaacSim GUI. 


This is how the communication takes place between Moveit!, RViz and IsaacSim:![alt text](<images/Screenshot from 2024-08-26 16-12-39.png>)

**Custom Node Development**:

I developed a custom node that communicates with 'move_group' node through 'Moveitinteface':
1. Include 'move_group_interface' so the the code directly communicates with the required topics that are being subscribed by Moveit!.
2. Set the 'planning pipeline id' and 'planner id' to 'move_group_interface'.
3. Define a 'target pose' and set it 'move_group_interface'.
4. Tell moveit tp plan the trajectory to the 'target pose'.
5. Execute the tajectory if the planning is successful.
6. Build and run the code as a ROS2 node by specifying the 'planning pipeline id' and 'planner id' in the CLI.  
7. Visualise the planning in RViz GUI. The execution is visualised in both RViz and Isaac sim GUI.

The communication between the nodes can be visualised as: 
![alt text](images/rosgraph_mv.png)
## Simulation
The simulation looks like this: https://youtu.be/_O5hBJp4P28

## Next steps:
1. To set up different parameters for comparision
2. Set up dynamic object collision avoidance
