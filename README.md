# Initial Setup (if using docker(highly recommended)):

1. Install vs-code editor on your ubuntu 20.04 host OS : https://linuxize.com/post/how-to-install-visual-studio-code-on-ubuntu-20-04/
2. Install docker on your ubuntu 20.04 host OS : https://docs.docker.com/engine/install/ubuntu/
3. Please remember to follow all the steps in docker installation, and verifying it's successful installation @ https://docs.docker.com/engine/install/ubuntu/#next-steps > https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user (Step 1 2 3 4)
4. Install following extensions on your VSCode: 
    1. Docker : https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker
    2. Remote-Containers :  https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
5. Clone this repository using:
    > ``` git clone https://github.com/Abhoir1/ENPM_662_Project2.git```
6. From the current working directory, cd to ENPM_662_Project2 repo which u just cloned
7. Open a terminal in /workspaces/ENPM_662_Project2 and type 
    > ``` code . ```
8. A VSCode popup should arise which says "Open folder in Container" where you choose Yes OR press F1,search for "Open Folder in Container" and execute the command
9. Now wait for the build process to finish, once it's completed, you have a fully functional ROS1 workspace with example packages
11. When the container is built for the first time, a error will popup saying "Failed to enable ROS Extension" , just choose the reload window option
10. Set up ROS dependencies using the following section

# Setting up ROS-Dependencies (docker does this for you once after a fresh build):

1. Once the repo is opened in a container, you have to install ROS1-dependencies. To do this, follow these steps:
    0. Inside the container, use a bash terminal will open at the following command's path if not, run it
    1. cd /workspaces/ENPM_662_Project2
    2. rosdep update
    3. sudo apt-get update
    4. rosdep install --from-paths src --ignore-src -r -y

# Building Packages:

1. cd /workspaces/ENPM_662_Project2
2. catkin_make

# This repository contains the following packages as of this commit:

1. a3c_description:
    1. This package contains urdf of Systemantics India's Asystr 3C robot, and robotiq 2F 140mm stroke gripper
    2. There are launch files for spawning just the robot in gazebo and moveit, and for spawning robot with the gripper
2. a3c_kinematics: 
    1. This package contains a custom written c++ library for forward kinematics of A3C
    2. The profiler node runs FKs for a set amount of time, and gives a per iteration result
    3. Further work can be done on writing a velocity and position IK implementation
3. a3c_moveit:
    1. Contains moveit based tutorial, and demo packages developed around Asystr3C 
    2. The package demonstates basic rviz visualization of joint space and cartesian space traj gen using moveit
    3. This package also demonstates basic pick and place operation using gazebo and rviz
4. a3c_moveit_config:
    1. This package contains entire moveit config of A3C, it has custom tuned position and effort controllers for each joint and the gripper
    2. Self collision avoidance is kept pretty tight
    3. It also has ROS based IK validation package, which draws a circle
5. kinematics_validation:
    1. This package contains 2 files, 1 used for generating symbolic FK equation, and the other generates a circular trajectory for A3C using inverse of jacobian
6.  roboticsgroup_gazebo_plugin:
    1. It is a mimic joint plugin cloned from https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins , All credits go to the author @Konstantinos Chatzilygeroudis
    2. This mimic plugin is used to enable gazebo's use of mimic'ed joints used by the robotiq 2F gripper
7. workspace visualization:
    1. This packages iterates J1 and J2 positions in steps to plot a 3D plot of maximum reachable workspace






