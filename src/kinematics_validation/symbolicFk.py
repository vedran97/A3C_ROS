from contextlib import redirect_stdout
import math
from sympy import *

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
# d1=symbols('d1')
# d2=symbols('d2')
# d3=symbols('d3')
# d4=symbols('d4')
# d5=symbols('d5')
# d6=symbols('d6')
# a2=symbols('a2')
# pi= symbols('pi')
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
    return simplify((A))
A = []
j=1
for i in dh_matrix:
    mat = getAMatrix(i)
    print('mat no ',j)
    pprint(mat)
    A.append(mat)
    j+=1
    
#Transformation from EE to base

T = simplify((A[0]@A[1]@A[2]@A[3]@A[4]@A[5]))
pprint(cleanA(T.subs([(theta1, -2.11703), (theta2, -0.370231), (theta3, -1.37619), (theta4, -0.000243698), (theta5, -1.39371), (theta6, -2.11568)])))
