import sympy as sp

symbols = sp.symbols

t = symbols ('t')
theta1 = symbols('theta1')
theta2 = symbols('theta2')
theta3 = symbols('theta3')
theta4 = symbols('theta4')
theta5 = symbols('theta5')
theta6 = symbols('theta6')

cos = sp.cos
sin = sp.sin

#cleaning A from very small values
def cleanA(A):
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if (isinstance(A[i,j], sp.Float) and abs(A[i,j]) < 1e-14 and A):
                A[i,j]=0.0
            if (isinstance(A[i,j], sp.Mul))and(i<3)and(j<3):
                A[i,j]=sp.nsimplify(A[i,j],tolerance=1e-10,rational=False)
    return A

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
                [0,theta2-sp.pi/2,-sp.pi/2,d2],
                [a2,theta3+sp.pi/2,0,-d3],
                [0,theta4,sp.pi/2,d4],
                [0,theta5,-sp.pi/2,d5],
                [0,theta6+sp.pi/2,sp.pi/2,d6]]
    return dh_matrix
    
#Transformation from EE to base
def getEEFTransformationMatrix():
    A = []
    dh_matrix = getDHMatrix()
    for i in dh_matrix:
        mat = getAMatrix(i)
        A.append(mat)
    #Transformation from EE to base
    T = sp.simplify((A[0]@A[1]@A[2]@A[3]@A[4]@A[5]))
    return T


if __name__ == "__main__":
    sp.pprint("EE Position:")
    sp.pprint(getEEFTransformationMatrix())