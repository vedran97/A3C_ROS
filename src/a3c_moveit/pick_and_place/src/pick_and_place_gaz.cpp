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

#include <std_srvs/Empty.h>
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
  moveGroup.setStartStateToCurrentState();
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
  //Create move group interface
  moveit::planning_interface::MoveGroupInterface moveGroupInterface(PLANNING_GROUP);
  //Create Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  //Create a JointModelGroup ptr
  const moveit::core::JointModelGroup* jointModelGroup =
    (moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
  //Create a Plan object
  moveit::planning_interface::MoveGroupInterface::Plan robotJointsPlan;

  ros::ServiceClient onClient = node_handle.serviceClient<std_srvs::Empty>("/A3C/on");
  ros::ServiceClient offClient = node_handle.serviceClient<std_srvs::Empty>("/A3C/off");
  /**
   * Set all the initial waypoints
  */
   // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", moveGroupInterface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", moveGroupInterface.getEndEffectorLink().c_str());

  //Set home pose, this information can be extracted from the plan_group as well
  std::vector<double> homePose = {
    0,0,0,0,0,0
  };
  //Set first waypoint joint angle
  std::vector<double> firstWaypointJointAngles = {
    0.216918,-0.917107,-1.033627,0.005887,-1.185505,0.166678
  };
  /**
   * Move to initial approach position of picking
  */
  executeJointSpaceMotion(firstWaypointJointAngles,moveGroupInterface,jointModelGroup,robotJointsPlan);

  /**
   * Task space motion in -Z direction
  */
  //Set next task space pose
  geometry_msgs::Pose taskSpacePose1 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose1.position.z = 0.044204;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);

  // TODO : Add pick logic  
  ROS_INFO_STREAM("Pick part position: " << moveGroupInterface.getCurrentPose().pose.position);
  ROS_INFO_STREAM("Pick part orientation: " << moveGroupInterface.getCurrentPose().pose.orientation);
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response resp;
  bool success = onClient.call(req,resp);
  ROS_INFO_STREAM("Service Status: Gripper triggered? : " << success);
  ros::Duration(1).sleep();


  /**
   * Task space motion in +Z direction
  */
  //Set next task space pose
  waypoints.clear();
  taskSpacePose1 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose1.position.z += 0.1;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);
  
  /**
   * Rotate J1 to get to approach of place position
  */
  std::vector<double> secondWaypointJointAngles = moveGroupInterface.getCurrentJointValues();
  secondWaypointJointAngles.at(0) = 2.06687;
  executeJointSpaceMotion(secondWaypointJointAngles,moveGroupInterface,jointModelGroup,robotJointsPlan);



  /**
   * Task space motion in -Z direction
  */
  waypoints.clear();
  taskSpacePose1=moveGroupInterface.getCurrentPose().pose;
  taskSpacePose1.position.z -= 0.1;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);


  // TODO : Place the part 
  ROS_INFO_STREAM("Place part position: " << moveGroupInterface.getCurrentPose().pose.position);
  ROS_INFO_STREAM("Place part orientation: " << moveGroupInterface.getCurrentPose().pose.orientation);

  success = offClient.call(req,resp);
  ROS_INFO_STREAM("Service Status: Gripper triggered? : " << success);
  ros::Duration(1).sleep();

  /**
   * Task space motion in +Z direction
  */
  waypoints.clear();
  taskSpacePose1=moveGroupInterface.getCurrentPose().pose;
  taskSpacePose1.position.z += 0.1;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);


  /**
   * Return to home pose
  */
  executeJointSpaceMotion(homePose,moveGroupInterface,jointModelGroup,robotJointsPlan);
  ros::shutdown();
  return 0;
}
