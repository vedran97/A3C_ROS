import math
from sympy import *
import plotly.graph_objects as go
import numpy as np
import matplotlib.pyplot as plt
from tqdm import *
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


# # in this rotation i assume dx/dt = 0 , and rate of
q_init = np.array([-math.pi,math.pi/2,0,0,0,0])

X_LIST,Y_LIST,Z_LIST = [],[],[]


while (q_init[0]<=math.pi):
    q_init[1]=math.pi/2
    while (q_init[1]>=-math.pi/2):
        xee = T.subs([(theta1, q_init[0]), (theta2, q_init[1]), (theta3, q_init[2]), (theta4, q_init[3]), (theta5, q_init[4]), (theta6, q_init[5])])
        xee = (np.array(xee).astype(np.float64))
        X_LIST.append((xee[:-1,3])[0])
        Y_LIST.append((xee[:-1,3])[1])
        Z_LIST.append((xee[:-1,3])[2])
        q_init[1]-=0.1
    q_init[0]+=1
    print("inside outer loop")
    

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Plot x, y and z data points
ax.scatter(X_LIST, Y_LIST, Z_LIST, color='red')
ax.scatter(0,0,0,color = 'green',s=100)

ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ')
plt.show()
fig.savefig("./workspaceViz.png")