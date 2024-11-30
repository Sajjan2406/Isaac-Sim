# Franka_Robot

This is a part of my Master thesis. 

## Aim: 
1. To analyse the performance of Nvidia Cumotion motion planning algorithms 
2. To integrate NVIDIA Issac Sim(physics based smulation platform) and ROS2(a robotics middleware framework)


Cumotion or Curobo is a CUDA accelerated library, developed by NVIDIA, containing a suite of robotics algorithms leveraging parallel compute.

This project is to uses Isaacsim simulation platform to manipulate 'Franka Panda' robot through ROS2 middleware.

IsaacSim environmemnt is loaded with enabling the ROS2 bridge paackge through the ’omni.isaac.ros2_bridge’ extension. Inside the IsaacSim environment a OmniGraph(as explained in the
section 5.1) is configured to communicate with ROS2. The Figure 9 explins the Action Graph contruction.

The robots used are Franka Panda for manipulaton and Idealworks iw.hub robot for mobile platform
The project uses 2 benchmarking scenes and a factory scene(refer the figures) to analyse the performance of the cuMotion pipeline with OMPL pipeline.the queries are such that the robot starts from its non-collission intial state. From this state it moves to the first target and then to the final target. 
1. Narrow Tunnel
  a. The targets are on both sides of the tunnel walls. The robot can move move through the the tunnel or avoid it to reach the final target pose.
3. Library Scene: Contains a Bookshelf and a table and the robot is placed in between them.
  a. the robot needs to move from the intial Position to the Top shelf(First Target) and then under the table(Final Target)
5. Fctory Scene: Containes a rack and two Containers
  a. The robot has to avoid obstacles and move from intial non collisios state to middle section of the rack(First Target) and then to the container(Final Target)

##Evaluation of trajectory generation performance
The parameters analysed are:
1. C Space Path Length:  The length of the path that a robot takes in the configuration space
2. Mean Velocity:  The average speed at which the robot’s end-effector or a particular point of interest on the robot moves along the path
3. Max Accelaration:  The highest rate of change of velocity that the robot experiences along the planned path.
4. Max jerk:  The maximum rate of change of acceleration
5. Full Pipeline Time: the total time from the start of the motion planning process to the completion of the execution of the generated path.

The experiment is conducted for 5 iterations for each planner and planning scene. The planned trajectory for the robot is it is visualised in the RViz and the robot motion in IsaacSim

##Results and Discussion

## Simulation
The simulation looks like this: https://youtu.be/_O5hBJp4P28

## Next steps:
1. To set up different parameters for comparision
2. Set up dynamic object collision avoidance
