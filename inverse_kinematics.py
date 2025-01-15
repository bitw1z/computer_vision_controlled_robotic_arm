import forward_kinematics
from sympy import * 

dh_params = forward_kinematics.dh_params
L0 = forward_kinematics.L0

# given desired position x, y, z and psi, compute theta0, 1, 2, and 3
def compute_joint_angles(x, y, z):
    theta0 = atan2(y, x).evalf()

    # ignore base frame and only consider frame 0, 1, 2, and 3 
    # plane geometry associated with a three-link planar manipulator 
    w = sqrt(x**2 + y**2).evalf()
    h = z - L0 

    # compute theta2 
    a1, a2 = symbols('a1 a2')
    cos2 = (w**2 + h**2 - a1**2 - a2**2)/(2*a1*a2)
    cos2 = cos2.evalf(subs = dh_params)

    # check existence of solutions (whether desired position is reachable)
    if ((cos2 < -1) or (cos2 > 1)):
        print("Solution does not exist")
        return -1
    
    # sign of sin2 determines elbow-up and elbow-down 
    sin2_plus = sqrt(1 - cos2**2).evalf()
    sin2_minus = -sqrt(1 - cos2**2).evalf()

    theta2 = []
    theta2_p = atan2(sin2_plus, cos2).evalf()
    theta2_m = atan2(sin2_minus, cos2).evalf()
    theta2.extend([theta2_p, theta2_m])

    # compute theta1 
    k1 = a1 + a2*cos2
    k2_plus = a2 * sin2_plus
    k2_minus = a2 * sin2_minus

    theta1 = []
    theta1_p = atan2(h, w) - atan2(k2_plus, k1)
    theta1_p = theta1_p.evalf(subs = dh_params)
    theta1_m = atan2(h, w) - atan2(k2_minus, k1)
    theta1_m = theta1_m.evalf(subs = dh_params)
    theta1.extend([theta1_p, theta1_m])

    # using forward kinematics to check calculation is correct 
    fk_t = forward_kinematics.compute_fk_t(theta0, theta1[0], theta2[0], 0)
    x_err = abs(fk_t[0, 3] - x)
    y_err = abs(fk_t[1, 3] - y)
    z_err = abs(fk_t[2, 3] - z)

    # radians to degrees 
    theta0_degree = (180 * theta0 / pi).evalf()
    theta1_degree = (180 * theta1[1] / pi).evalf()
    theta2_degree = (180 * theta2[1] / pi).evalf()
    
    # calibrtion on servo motors
    if (theta2_degree > 0):
        theta2_degree = 90 - theta2_degree
    elif (theta2_degree == 0): 
        theta2_degree = 90
    elif (theta2_degree < 0): 
        theta2_degree = theta2_degree * (-1) + 90
    
    return theta0_degree, theta1_degree, theta2_degree