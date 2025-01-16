import numpy as np 
from sympy import * 

# use modified DH paramters 
alpha0, alpha1, alpha2 = symbols('alpha0 alpha1 alpha2')
a0, a1, a2 = symbols('a0 a1 a2')
d1, d2, d3 = symbols('d1 d2 d3')
theta1, theta2, theta3 = symbols('theta1 theta2 theta3')

# update DH parameters based on your manipulator structure 
dh_params  = {alpha0: 0, a0: 0, d1: 0, theta1: theta1,
              alpha1: 0, a1: 10.5, d2: 0, theta2: theta2,
              alpha2: 0, a2: 10, d3: 0, theta3: 0}

# define translation and rotation of {0} with respect to base frame
L0 = 9
alphab = pi/2

# define modified DH transformation matrix 
def dh_tf_matrix(alpha, a, d, theta):
    TF = Matrix([[cos(theta), -sin(theta), 0, a], 
                 [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                 [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [0, 0, 0, 1]])
    return TF

# calculate transformation matrix from frame {3} to base frame {b}
def compute_fk_t(theta0, theta1, theta2, theta3): 
    T_01 = dh_tf_matrix(alpha0, a0, d1, theta1).evalf(subs = dh_params)
    T_12 = dh_tf_matrix(alpha1, a1, d2, theta2).evalf(subs = dh_params)
    T_23 = dh_tf_matrix(alpha2, a2, d3, theta3).evalf(subs = dh_params)
    
    # compute transformation matrix from frame {3} to {0}
    T_03 = T_01 * T_12 * T_23 

    # given theta1, 2, and 3, calculate T_03 
    T_03 = np.matrix(T_03.evalf(subs = {theta1: theta1, theta2: theta2, theta3: theta3}))

    # translation matrix from frame {0} to {b}
    T_b0 = Matrix([[1, 0, 0, 0], 
                   [0, 1, 0, 0],
                   [0, 0, 1, L0],
                   [0, 0, 0, 1]])

    # rotation matrix from frame {0} to {b}
    R_b0 = Matrix([[cos(theta0), -sin(theta0)*cos(alphab), sin(theta0)*sin(alphab), 0], 
                   [sin(theta0), cos(theta0)*cos(alphab), -cos(theta0)*sin(alphab), 0],
                   [0, sin(alphab), cos(alphab), 0],
                   [0, 0, 0, 1]])

    # compute transformation matrix from frame {0} to {b}
    T_b0 = T_b0 * R_b0
    T_b0 = np.matrix(T_b0.evalf(subs = {theta0: theta0}))

    # compute transformation matrix from frame {3} to {b}
    T_b3 = T_b0 * T_03
    return T_b3
