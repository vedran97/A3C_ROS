import math
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

t = sp.symbols ('t')
theta1 = sp.symbols('theta1')
theta2 = sp.symbols('theta2')
theta3 = sp.symbols('theta3')
theta4 = sp.symbols('theta4')
theta5 = sp.symbols('theta5')
theta6 = sp.symbols('theta6')

cos = sp.cos
sin = sp.sin

#Forming A matrix
def getAMatrix(input):
    a = input[0]
    theta = input[1]
    alpha = input[2]
    d = input[3]
    A = sp.Matrix(
        [[cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]]
    )
    return ((A))

def getDHMatrix():
    #define DH parameters:
    d1=0.1915
    d2=0.1410
    d3=0.1415
    d4=0.230
    d5=0.1635
    d6=0.1660
    a2=0.230

    #Making the DH matrix
    dh_matrix = [[0,theta1,0,d1],
                [0,theta2-math.pi/2,-math.pi/2,d2],
                [a2,theta3+math.pi/2,0,-d3],
                [0,theta4,math.pi/2,d4],
                [0,theta5,-math.pi/2,d5],
                [0,theta6+math.pi/2,math.pi/2,d6]]
    return dh_matrix

def getEEFTransformationMatrix():
    A = []
    dh_matrix = getDHMatrix()
    for i in dh_matrix:
        mat = getAMatrix(i)
        A.append(mat)
    #Transformation from EE to base
    T = ((A[0]@A[1]@A[2]@A[3]@A[4]@A[5]))
    return T

if __name__ == "__main__":

    X_LIST,Y_LIST,Z_LIST = [],[],[]

    q_init = np.array([-math.pi,math.pi/2,0,0,0,0])

    T = getEEFTransformationMatrix()

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