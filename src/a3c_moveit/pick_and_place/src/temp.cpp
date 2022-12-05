/* Author: Vedant Ranade */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <ros/ros.h>
/**
 * Execute joint space motion
*/
bool executeJointSpaceMotion(
const std::vector<double>& targetAngles,
      moveit::planning_interface::MoveGroupInterface& moveGroup,
const moveit::core::JointModelGroup* jointModelGroup,
      moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  moveit::core::RobotStatePtr current_state = moveGroup.getCurrentState(); 
  moveGroup.setJointValueTarget(targetAngles);
  //Scale velocity wrt max velocity defined joints_limits
  moveGroup.setMaxVelocityScalingFactor(1.0);
  moveGroup.setMaxAccelerationScalingFactor(1.0);
  // If plan succeeds,execute, otherwise return
  auto success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    moveGroup.execute(plan);
  }else{
    ROS_ERROR_NAMED("tutorial", "JointSpaceMotionFailed");
    moveGroup.execute(plan);
  }
  return success;
}
/**
 * Execute cartesian Motion
*/
bool executeCartesianMotion(
  std::vector<geometry_msgs::Pose>& waypoints,
  moveit::planning_interface::MoveGroupInterface& moveGroup
  )
{
  moveGroup.setPlanningTime(1);
  moveit_msgs::RobotTrajectory trajectory;
  static const double jump_threshold = 0.0;
  static const double eef_step = 0.001;
  double fraction = moveGroup.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "(Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  moveGroup.execute(trajectory);
  return false;
}
/**
 * Add blocks
*/
void addObject(
moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
const moveit::planning_interface::MoveGroupInterface& moveGroup
)
{
  std::vector<moveit_msgs::CollisionObject> box;
  box.resize(1);

  box.at(0).header.frame_id = moveGroup.getPlanningFrame();
  box.at(0).id = "box";
  box.at(0).primitives.resize(1);
  box.at(0).primitives.at(0).type = box.at(0).primitives[0].BOX;
  box.at(0).primitives.at(0).dimensions.resize(3);
  box.at(0).primitives.at(0).dimensions = {0.05,0.08,0.2};
  box.at(0).primitive_poses.resize(1);

  box.at(0).primitive_poses[0].position.x = -0.503912+-0.175;
  box.at(0).primitive_poses[0].position.y = 0.0225;
  box.at(0).primitive_poses[0].position.z = 0.10001;
  box.at(0).primitive_poses[0].orientation.w = 1.0;

  box.at(0).operation = box.at(0).ADD;

  planning_scene_interface.applyCollisionObjects(box);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // Set plan group
  static const std::string PLANNING_GROUP = "a3c_plan_group";
  static const std::string PLANNING_GROUP_2 = "robotiq_gripper_solver";
  //Create move group interface
  moveit::planning_interface::MoveGroupInterface moveGroupInterface(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface moveGroupInterface2(PLANNING_GROUP_2);
  //Create Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  //Create a JointModelGroup ptr
  const moveit::core::JointModelGroup* jointModelGroup =
    (moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
  const moveit::core::JointModelGroup* gripperModelGroup =
    (moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_2));
  //Create a Plan object
  moveit::planning_interface::MoveGroupInterface::Plan robotJointsPlan;
  moveit::planning_interface::MoveGroupInterface::Plan gripperPlan;
  /**
   * Set all the initial waypoints
  */
   // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", moveGroupInterface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", moveGroupInterface.getEndEffectorLink().c_str());

  //Add object
  addObject(planningSceneInterface,moveGroupInterface);

  //Set home pose, this information can be extracted from the plan_group as well
  std::vector<double> homePose = {
    0,0,0,0,0,0
  };
  geometry_msgs::Pose target_pose1 = moveGroupInterface.getCurrentPose().pose;
  target_pose1.position.x = 0.4414;
  target_pose1.orientation.w=0;
  target_pose1.orientation.z=0;
  target_pose1.orientation.x=0.7071068;
  target_pose1.orientation.y=0.7071068;
  moveGroupInterface.setPoseTarget(target_pose1);
  
  bool success = (moveGroupInterface.plan(robotJointsPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  moveGroupInterface.execute(robotJointsPlan);
  ros::Duration(5).sleep();
  for(const auto&i: moveGroupInterface.getCurrentJointValues()){
    std::cout<<i<<std::endl;
  }

  ros::shutdown();
  return 0;
}
