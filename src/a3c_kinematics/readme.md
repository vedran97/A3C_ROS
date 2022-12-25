# Running Instructions and Description:

1. This package has a library which generates the End Effector Pose of a A3C,and has 2 nodes.
2. One of the nodes calculates EEF(End effector) position for like 6 sets of joint angle and prints them out. The other node calculates FK for 6 sets of angles for a fixed no of iterations, and then computes time for each iteration.
3. Run `rosrun a3c_kinematics a3c_fk_node` in a terminal, to get results of EEF position for the chosen set of joint angles. This node also demonstrates how you can use this library in your codebase.
4. Run `rosrun a3c_kinematics a3c_fk_profile_node` in a terminal, to get the profiling results of FK operation.