/* Author: Vedant Ranade */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>

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
  }
  return success;
}
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Set plan group
  static const std::string PLANNING_GROUP = "a3c_plan_group";
  //Create move group interface
  moveit::planning_interface::MoveGroupInterface moveGroupInterface(PLANNING_GROUP);
  //Create Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  //Create a JointModelGroup ptr
  const moveit::core::JointModelGroup* jointModelGroup =
    (moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
  //Create a Plan object
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  /**
   * Set all the waypoints
  */
  //Set home pose, this information can be extracted from the plan_group as well
  std::vector<double> homePose = {
    0,0,0,0,0,0
  };
  //Set first waypoint joint angle
  std::vector<double> firstWaypointJointAngles = {
    -0.2374,0.2906,-1.7445,1.4493,-0.4103,2.5357
  };
  /**
   * Move to initial approach position of picking
  */
  executeJointSpaceMotion(firstWaypointJointAngles,moveGroupInterface,jointModelGroup,plan);
  ros::Duration(1.0).sleep();
  /**
   * Task space motion in -Z direction
  */
  //Set next task space pose
  geometry_msgs::Pose taskSpacePose1;
  taskSpacePose1.position.x = -0.3436;
  taskSpacePose1.position.y = 0.04350;
  taskSpacePose1.position.z = 0.150407;
  taskSpacePose1.orientation.w = 0.692813;
  taskSpacePose1.orientation.x = 0.306239;
  taskSpacePose1.orientation.y = -0.615454;
  taskSpacePose1.orientation.z = -0.217817;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();
  /**
   * Task space motion ,Reach In
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose2 = taskSpacePose1;
  taskSpacePose2.position.x -=  0.15;
  waypoints.push_back(taskSpacePose2);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();
  // TODO : Add pick logic  

  /**
   * Task space motion ,Reach Out
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose3 = taskSpacePose2;
  taskSpacePose3.position.x +=  0.15;
  waypoints.push_back(taskSpacePose3);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();
  /**
   * Task space motion in +Z direction
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose4 = taskSpacePose3;
  taskSpacePose4.position.z =  0.289671;
  waypoints.push_back(taskSpacePose4);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();
  /**
   * Rotate J1 to get to approach of place position
  */
  std::vector<double> secondWaypointJointAngles = firstWaypointJointAngles;
  secondWaypointJointAngles.at(0) = 1.0;
  executeJointSpaceMotion(secondWaypointJointAngles,moveGroupInterface,jointModelGroup,plan);
  ros::Duration(1.0).sleep();
  /**
   * Task space motion in -Z direction
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose5;
  taskSpacePose5.position.x = -0.140888;
  taskSpacePose5.position.y = -0.305582;
  taskSpacePose5.position.z = 0.150407;
  taskSpacePose5.orientation.w = 0.690448;
  taskSpacePose5.orientation.x = 0.607006;
  taskSpacePose5.orientation.y = -0.319058;
  taskSpacePose5.orientation.z = 0.230277;
  waypoints.push_back(taskSpacePose5);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();

  // TODO : Place the part 

  /**
   * Reach Out to clear space for joint motion
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose6 = taskSpacePose5;
  taskSpacePose6.position.y += 0.1582;
  waypoints.push_back(taskSpacePose6);
  executeCartesianMotion(waypoints,moveGroupInterface);
  ros::Duration(1.0).sleep();

  /**
   * Return to home pose
  */
  executeJointSpaceMotion(homePose,moveGroupInterface,jointModelGroup,plan);
  ros::Duration(1.0).sleep();
  ros::shutdown();
  return 0;
}
