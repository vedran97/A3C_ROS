cmake_minimum_required(VERSION 3.1.3)
project(a3c_moveit)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    std_msgs
    std_srvs 
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(DEPENDENCIES std_msgs std_srvs )

catkin_package(CATKIN_DEPENDS std_msgs std_srvs)
# set(THIS_PACKAGE_INCLUDE_DIRS
#   doc/interactivity/include
# )

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# add_subdirectory(tutorials/controller_configuration)
# add_subdirectory(doc/interactivity)
add_subdirectory(tutorials/kinematics_model)
add_subdirectory(tutorials/move_group_interface)
add_subdirectory(pick_and_place)
add_subdirectory(a3c_ik_validation)
# add_subdirectory(tutorials/motion_planning_pipeline)

# add_subdirectory(doc/pick_place)
# add_subdirectory(doc/planning)
# add_subdirectory(doc/planning_scene)
# add_subdirectory(doc/planning_scene_ros_api)
# add_subdirectory(doc/robot_model_and_robot_state)
# add_subdirectory(doc/state_display)
# add_subdirectory(doc/subframes)
# add_subdirectory(doc/trajopt_planner)
# add_subdirectory(doc/creating_moveit_plugins/lerp_motion_planner)
# add_subdirectory(doc/moveit_cpp)
# add_subdirectory(doc/collision_environments)
# add_subdirectory(doc/visualizing_collisions)
# add_subdirectory(doc/bullet_collision_checker)