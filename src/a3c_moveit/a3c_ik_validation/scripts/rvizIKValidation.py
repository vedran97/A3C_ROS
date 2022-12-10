import math
from sympy import *
import plotly.graph_objects as go
import numpy as np
import matplotlib.pyplot as plt
from tqdm import *

import time
import sys
import rospy
import moveit_commander
import moveit_msgs.msg

t = symbols ('t')
theta1 = symbols('theta1')
theta2 = symbols('theta2')
theta3 = symbols('theta3')
theta4 = symbols('theta4')
theta5 = symbols('theta5')
theta6 = symbols('theta6')
d1=0.1915
d2=0.1410
d3=0.1415
d4=0.230
d5=0.1635
d6=0.1660
pen = 0.1
a2=0.230

#Making the DH matrix
dh_matrix = [[0,theta1,0,d1],# done
             [0,theta2-math.pi/2,-math.pi/2,d2],
             [a2,theta3+math.pi/2,0,-d3],
             [0,theta4,math.pi/2,d4],
             [0,theta5,-math.pi/2,d5],
             [0,theta6+math.pi/2,math.pi/2,d6]]
#cleaning A from very small values
def cleanA(A):
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if (isinstance(A[i,j], Float) and abs(A[i,j]) < 1e-14 and A):
                A[i,j]=0.0
            if (isinstance(A[i,j], Mul))and(i<3)and(j<3):
                A[i,j]=nsimplify(A[i,j],tolerance=1e-10,rational=False)
    return A
#Forming A matrix
def getAMatrix(input):
    a = input[0]
    theta = input[1]
    alpha = input[2]
    d = input[3]
    A = Matrix(
        [[cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]]
    )
    return cleanA((A))
A = []
j=1
for i in dh_matrix:
    mat = getAMatrix(i)
    # print('mat no ',j)
    # pprint(mat)
    A.append(mat)
    j+=1
    
#Transformation from EE to base

T = ((A[0]@A[1]@A[2]@A[3]@A[4]@A[5]))
T_6 = T
T_5 = A[0]@A[1]@A[2]@A[3]@A[4]
T_4 = A[0]@A[1]@A[2]@A[3]
T_3 = A[0]@A[1]@A[2]
T_2 = A[0]@A[1]
T_1 = A[0]

# Extracting the last column i.e. translation of end effector wrt base frame
X_P = T[:3, 3]

# Taking partial derivative of the traslation wrt theta1 to theta7
X_P_diff1 = X_P.diff(theta1)
X_P_diff2 = X_P.diff(theta2)
X_P_diff3= X_P.diff(theta3)
X_P_diff4 = X_P.diff(theta4)
X_P_diff5 = X_P.diff(theta5)
X_P_diff6 = X_P.diff(theta6)

# Extracting the z column of all the transformation matrices wrt to base frame
Z_axis_1 = T_1[:3, 2]
Z_axis_2 = T_2[:3, 2]
Z_axis_3 = T_3[:3, 2]
Z_axis_4 = T_4[:3, 2]
Z_axis_5 = T_5[:3, 2]
Z_axis_6 = T_6[:3, 2]

# Defining the Jacobian Matrix
J = Matrix([[X_P_diff1, X_P_diff2, X_P_diff3, X_P_diff4, X_P_diff5, X_P_diff6],
    [Z_axis_1, Z_axis_2,Z_axis_3, Z_axis_4, Z_axis_5, Z_axis_6]])

# Prepping the traj gen loop

trajectory_time = 5
steps = 500
time_steps = np.linspace(0,trajectory_time,steps)
delta_t = trajectory_time/steps
radius = 0.05
omega = 2*np.pi/trajectory_time

x_dot = +1 * omega * radius * np.sin(omega*time_steps-(0.785398163+3*math.pi/2))
y_dot = -1 * omega * radius * np.cos(omega*time_steps-(0.785398163+3*math.pi/2))
z_dot = 0

X_LIST=[]
Y_LIST=[]
xOld=0.3
xInit=xOld
yOld=0.18
yInit=yOld

for i in tqdm(range(steps)):
  xNew = xOld+delta_t*x_dot[i]
  yNew = yOld+delta_t*y_dot[i]
  X_LIST.append(xNew)
  Y_LIST.append(yNew)
  xOld = xNew
  yOld = yNew

fig, axs = plt.subplots()
axs.set_xlabel("x")
axs.set_ylabel("y")
axs.plot(X_LIST,Y_LIST)
axs.set_aspect('equal')
axs.set_title("Expected Trajectory")
fig.tight_layout()
plt.show()
fig.savefig("./Expected_Trajectory.png")
# in this rotation i assume dx/dt = 0 , and rate of change of roll pitch yaw to be zero 
X_dot = np.zeros((6,steps))
X_dot[0,:] = x_dot
X_dot[1,:] = y_dot
X_dot[2,:] = z_dot

# # in this rotation i assume dx/dt = 0 , and rate of roll pitch yaw to be zero
q_init = np.array([-2.11696,-0.370079,-1.3761,-0.000627978,-1.3936,-2.11535])
q_old = q_init
q_list = np.zeros((6,steps))
Xee = np.zeros((6,steps))

Xee_init = T.subs([(theta1, q_old[0]), (theta2, q_old[1]), (theta3, q_old[2]), (theta4, q_old[3]), (theta5, q_old[4]), (theta6, q_old[5])])

for i in tqdm(range(steps)):
  jacobianMatrix_step = J.subs([(theta1, q_old[0]), (theta2, q_old[1]), (theta3, q_old[2]), (theta4, q_old[3]), (theta5, q_old[4]), (theta6, q_old[5])])
  jacobianMatrix_step = np.array(jacobianMatrix_step).astype(np.float64)
  joint_vel = np.linalg.pinv(jacobianMatrix_step) @ X_dot[:,i]

  q_old = q_old + delta_t*joint_vel

  q_list[:,i] = (q_old)

  xee = T.subs([(theta1, q_old[0]), (theta2, q_old[1]), (theta3, q_old[2]), (theta4, q_old[3]), (theta5, q_old[4]), (theta6, q_old[5])])

  xee = simplify(np.array(xee).astype(np.float64))

  Xee[0:3,i]=(xee[:-1,3])


X = np.array(Xee)

x=  X[0,:]
y = X[1,:]
z = X[2,:]

#plotting generated trajectory calculated using Velocity IK
fig, axs = plt.subplots()
axs.set_xlabel("X")
axs.set_ylabel("Y")
axs.scatter(x,y)
axs.set_aspect('equal')
axs.set_title("Generated_Trajectory")
fig.tight_layout()
plt.show()
fig.savefig("./Generated_Trajectory.png")

#plotted joint angles generated for drawing a circle
fig, axs = plt.subplots(3,2)
k=0
for i in range(3):
    for j in range(2):
        k+=1
        axs[i][j].plot(time_steps,q_list[i+j,:])
        axs[i][j].set(xlabel='time', ylabel='J'+str(k)+" Angle")
plt.show()
fig.savefig("./Joint_Angles.png")

# Init ROS now
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()
## This interface can be used to plan and execute motions:
group_name = "a3c_plan_group"
move_group = moveit_commander.MoveGroupCommander(group_name)
##
## Getting Basic Information
## ^^^^^^^^^^^^^^^^^^^^^^^^^
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Get robot to initial pose
move_group.go(q_init)
# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
print("Start recording video now,Enable traces")
time.sleep(5)

#   Instead of repeatedly sending a single point to joint state publisher, I am creating 
#   Create a RobotTrajectory object , and a joint trajectory object
#   Add all computed joint trajectories in it in the form of a JointTrajectoryPoint()
#   Add the correct frame id, time stamps etc
#   Add this joint trajectories in a RobotTrajectory Object
#   send circle trajectory using moveit's movegroup.execute() command
jointTrajectories = moveit_msgs.msg.trajectory_msgs.msg.JointTrajectory()
jointTrajectories.header.frame_id = "world"
jointTrajectories.header.stamp = jointTrajectories.header.stamp.from_sec(0)
jointTrajectories.header.seq = 0
jointTrajectories.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
jointTrajectories.points = []

for i in tqdm(range(steps)):
    newPoint = moveit_msgs.msg.trajectory_msgs.msg.JointTrajectoryPoint()
    newPoint.velocities = [0,0,0,0,0,0]
    newPoint.accelerations = [0,0,0,0,0,0]
    newPoint.positions = q_list[:,i]
    newPoint.time_from_start  = newPoint.time_from_start.from_sec(time_steps[i])
    jointTrajectories.points.append(newPoint)

traj = moveit_msgs.msg.RobotTrajectory()
traj.joint_trajectory = jointTrajectories
move_group.execute(traj)
move_group.stop()

