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
  //Set first waypoint joint angle
  std::vector<double> firstWaypointJointAngles = {
    0.450102,-0.0184897,-1.94337,0.977727,0.594344,0.686399
  };
  /**
   * Move to initial approach position of picking
  */
  executeJointSpaceMotion({0.0},moveGroupInterface2,gripperModelGroup,gripperPlan);
  executeJointSpaceMotion(firstWaypointJointAngles,moveGroupInterface,jointModelGroup,robotJointsPlan);

  /**
   * Task space motion in -Z direction
  */
  //Set next task space pose
  geometry_msgs::Pose taskSpacePose1 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose1.position.z = 0.1;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(taskSpacePose1);
  executeCartesianMotion(waypoints,moveGroupInterface);


  /**
   * Task space motion ,Reach In
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose2 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose2.position.x -=  0.15;
  waypoints.push_back(taskSpacePose2);
  executeCartesianMotion(waypoints,moveGroupInterface);


  // TODO : Add pick logic  
  ROS_INFO_STREAM("Pick part position: " << moveGroupInterface.getCurrentPose().pose.position);
  ROS_INFO_STREAM("Pick part orientation: " << moveGroupInterface.getCurrentPose().pose.orientation);

  executeJointSpaceMotion({0.1610},moveGroupInterface2,gripperModelGroup,gripperPlan);
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "box";
  att_coll_object.link_name = "tool0";
  att_coll_object.object.operation = att_coll_object.object.ADD;
  ROS_INFO_STREAM("Attaching cylinder to robot.");
  planningSceneInterface.applyAttachedCollisionObject(att_coll_object);


  /**
   * Task space motion ,Reach Out
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose3 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose3.position.x +=  0.15;
  waypoints.push_back(taskSpacePose3);
  executeCartesianMotion(waypoints,moveGroupInterface);

  /**
   * Task space motion in +Z direction
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose4 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose4.position.z =  0.2;
  waypoints.push_back(taskSpacePose4);
  executeCartesianMotion(waypoints,moveGroupInterface);

  /**
   * Rotate J1 to get to approach of place position
  */
  std::vector<double> secondWaypointJointAngles = moveGroupInterface.getCurrentJointValues();
  secondWaypointJointAngles.at(0) = 2.06687;
  executeJointSpaceMotion(secondWaypointJointAngles,moveGroupInterface,jointModelGroup,robotJointsPlan);


  /**
   * Reach in to place the part
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose6=moveGroupInterface.getCurrentPose().pose;
  taskSpacePose6.position.y -= 0.1;
  waypoints.push_back(taskSpacePose6);
  executeCartesianMotion(waypoints,moveGroupInterface);

  /**
   * Task space motion in -Z direction
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose5=moveGroupInterface.getCurrentPose().pose;
  taskSpacePose5.position.z = 0.1;
  waypoints.push_back(taskSpacePose5);
  executeCartesianMotion(waypoints,moveGroupInterface);


  // TODO : Place the part 
  ROS_INFO_STREAM("Place part position: " << moveGroupInterface.getCurrentPose().pose.position);
  ROS_INFO_STREAM("Place part orientation: " << moveGroupInterface.getCurrentPose().pose.orientation);
  att_coll_object.object.id = "box";
  att_coll_object.link_name = "tool0";
  att_coll_object.object.operation = att_coll_object.object.REMOVE;
  planningSceneInterface.applyAttachedCollisionObject(att_coll_object);
  executeJointSpaceMotion({0.0},moveGroupInterface2,gripperModelGroup,gripperPlan);

  /**
   * z+ y+ to clear the part
  */
  waypoints.clear();
  geometry_msgs::Pose taskSpacePose7 = moveGroupInterface.getCurrentPose().pose;
  taskSpacePose7.position.y += 0.1;
  taskSpacePose7.position.z += 0.1;
  waypoints.push_back(taskSpacePose7);
  executeCartesianMotion(waypoints,moveGroupInterface);


  /**
   * Return to home pose
  */
  executeJointSpaceMotion(homePose,moveGroupInterface,jointModelGroup,robotJointsPlan);
  ros::shutdown();
  return 0;
}
