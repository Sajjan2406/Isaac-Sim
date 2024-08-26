#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  
  // specify planning pipeline
 std::string planning_pipeline;
 std::string planner_id;
 
 node-> get_parameter("planning_pipeline", planning_pipeline);
 node-> get_parameter("planner_id", planner_id);

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
 using moveit::planning_interface::MoveGroupInterface;
 auto move_group_interface = MoveGroupInterface(node, "panda_arm");


// Set the planning pipeline and the planner id
 
 //move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
 //move_group_interface.setPlannerId("PTP");
 //rclcpp::get_logger("planner set success");
 
 //move_group_interface.setPlanningPipelineId("ompl");
 //move_group_interface.setPlannerId("RRTConnectkConfigDefault");
 
 //move_group_interface.setPlanningPipelineId("isaac_ros_cumotion");
 //move_group_interface.setPlannerId("cuMotion");
 move_group_interface.setPlanningPipelineId(planning_pipeline);
 move_group_interface.setPlannerId(planner_id);
 
// Set a target Pose
 auto const target_pose = []{
   geometry_msgs::msg::Pose msg;
   msg.orientation.w = 1.0;
   msg.position.x = 0.28;
   msg.position.y = -0.2;
   msg.position.z = 0.5;
   return msg;
}();
 move_group_interface.setPoseTarget(target_pose, "panda_hand"); //added "panda_hand"

// Create a plan to that target pose
 auto const [success, plan] = [&move_group_interface]{
   moveit::planning_interface::MoveGroupInterface::Plan msg;
   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
   return std::make_pair(ok, msg);
}();

// Execute the plan
 if(success) {
   move_group_interface.execute(plan);
   rclcpp::get_logger("plan success");
}  else {
   RCLCPP_ERROR(logger, "Planing failed!");
   }
   rclcpp::sleep_for(std::chrono::seconds(1));

 auto const target_pose1 = []{
   geometry_msgs::msg::Pose msg;
   msg.orientation.w = 1.0;
   msg.position.x = -0.28;
   msg.position.y = 0.2;
   msg.position.z = 0.5;
   return msg;
}();
 move_group_interface.setPoseTarget(target_pose1, "panda_hand"); //added "panda_hand"

// Create a plan to that target pose
 auto const [success1, plan1] = [&move_group_interface]{
   moveit::planning_interface::MoveGroupInterface::Plan msg;
   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
   return std::make_pair(ok, msg);
}();

// Execute the plan
 if(success1) {
   move_group_interface.execute(plan1);
   rclcpp::get_logger("plan1 success");
}  else {
   RCLCPP_ERROR(logger, "Planing failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

