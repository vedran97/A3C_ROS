# Running Instructions

1. For tutorials packages: 
    1. There are 2 packages right now
        1. moveit_group_interface:
            1. "roslaunch a3c_moveit move_group_interface.launch" ;  Launch robot with robotiq gripper in rviz with moveit motion planner plugin and fake controllers
            2. Once the robot spawns with all controllers and planners, ie the terminal stops printing stuff, please run "roslaunch a3c_moveit move_group_interface_tutorial_node.launch" in another terminal
        2. kinematics_model:
            1. "roslaunch a3c_moveit kinematics_model.launch" ;  Launch robot with robotiq gripper in rviz with moveit motion planner plugin and fake controllers
            2. Once the terminal stops printing stuff, run "rosrun a3c_moveit a3c_kinematics_model" in another terminal
2. a3c_ik_validation: 
    1. "roslaunch a3c_moveit ik_validation.launch" ;  Launch robot with robotiq gripper in rviz with moveit motion planner plugin and fake controllers
    2. Once the terminal stops printing stuff, run "rosrun a3c_moveit rvizIKValidation.py" in another terminal
3. pick_and_place:
    1. "roslaunch a3c_moveit pick_and_place_gazebo.launch" ;  Launch robot with robotiq gripper in rviz with moveit motion planner plugin, gazebo instance and effort controllers
    2. Once the terminal stops printing stuff, run "roslaunch a3c_moveit pick_and_place_gaz_node.launch" in another terminal
    3. "roslaunch a3c_moveit pick_and_place_rviz.launch" ;  Launch robot with robotiq gripper in rviz with moveit motion planner plugin and fake controllers
    4. Once the terminal stops printing stuff, run "roslaunch a3c_moveit pick_and_place_rviz_node.launch" in another terminal