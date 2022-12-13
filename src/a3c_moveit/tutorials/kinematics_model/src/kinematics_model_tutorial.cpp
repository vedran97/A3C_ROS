#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a3c_arm_kinematic_model");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  static const std::string PLANNING_GROUP = "a3c_plan_group";
  
  // Instantiate Robot Model by loading it
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Base frame of Model,which is used to compute tranforms: %s", kinematic_model->getModelFrame().c_str());
  // Construct RobotState from RobotModel
  auto kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  // kinematic_state->setToDefaultValues();
  const auto joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
  //Get joints in the group
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  ROS_INFO("Get names of All joints:");  
  for(const auto& jointName:joint_names){
    ROS_INFO("%s",jointName.c_str());
  }
  //Get Active joints in the group
  const std::vector<std::string> &active_joint_names = joint_model_group->getActiveJointModelNames();
  ROS_INFO("Get names of All active joints:");  
  for(const auto& jointName:active_joint_names){
    ROS_INFO("%s",jointName.c_str());
  }
  for(const auto&jointName:active_joint_names)
  {
    ROS_INFO("Joint %s: %f", jointName.c_str(), *kinematic_state->getJointPositions(jointName));
  }

  //Forward Kinematics:

  //Set Joint values
  const std::vector<double> joint_values = 
  {-0.2374,0.290699,-1.739199,1.4499,-0.4118,2.534199};
  kinematic_state->setJointGroupActivePositions(joint_model_group,joint_values);
  //Get EE position via FK
  const auto &end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
  /* Print end-effector pose. Remember that this is wrt base frame */
  //Prints the EE position wrt base frame
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  //Prints the quaternion orientation from this rotation matrix
  ROS_INFO_STREAM("Quarternion vector: " << Eigen::Quaterniond(end_effector_state.rotation()).vec() << "\r\n Quarternion w: " << Eigen::Quaterniond(end_effector_state.rotation()).w());
  
  //Inverse Kinematics:

  bool found_ik = kinematic_state->setFromIK(joint_model_group,end_effector_state,0.1);
  if (found_ik)
  {
    for(const auto&jointName:active_joint_names)
    {
      ROS_INFO("Joint %s: %f", jointName.c_str(), *kinematic_state->getJointPositions(jointName));
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  ros::shutdown();
  return 0;
}
