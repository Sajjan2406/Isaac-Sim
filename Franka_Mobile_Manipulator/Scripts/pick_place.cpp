#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> //added on 09/10

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
  
  node->get_parameter("planning_pipeline", planning_pipeline); 
  node->get_parameter("planner_id", planner_id); 

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set the planning pipeline and the planner id
  move_group_interface.setPlanningPipelineId(planning_pipeline); 
  move_group_interface.setPlannerId(planner_id); 

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.306;
    msg.position.y = 0.380;
    msg.position.z = 0.523;
    msg.orientation.x = 0.506;
    msg.orientation.y = 0.535;
    msg.orientation.z = 0.492;
    msg.orientation.w = -0.464;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose, "panda_hand");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    auto const logger = rclcpp::get_logger("Plan success");
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Plan success"); 
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Set another target Pose 
  auto const target_pose1 = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = -0.431;
    msg.position.y = 0.106;
    msg.position.z = 0.802;
    msg.orientation.x = 0.085;
    msg.orientation.y = 0.989;
    msg.orientation.z = -0.013;
    msg.orientation.w = -0.121;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose1, "panda_hand");

  // Create a plan to that target pose
  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    auto const logger = rclcpp::get_logger("Plan2 success");
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success1) {
    move_group_interface.execute(plan1);
    RCLCPP_INFO(logger, "Plan 1 success");
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  rclcpp::sleep_for(std::chrono::seconds(5));

  rclcpp::spin_some(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;

}

