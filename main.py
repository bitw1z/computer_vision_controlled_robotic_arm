from inverse_kinematics import *
from serial_communication import *
from object_detection import *
from coord_conversion import *
import time 

# call back to handle 2D to 3D conversion
def handle_2d_to_3d(cx, cy):
    p = convert_2D_to_3D(cx, cy)

    # convert world coordinates to robotic arm base coordinates
    r = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    t = np.array([[11, 8.5, 0]])
    p_transformed = r*p + t

    # extract x, y, and z coordinates of desired point
    x = p_transformed[0][0]
    y = p_transformed[1][0]
    z = p_transformed[2][0]

    # compute joint angles and send angles
    angle1, angle2, angle3 = compute_joint_angles(x, y, z)
    send_angles(angle1, angle2, angle3)

def main():
    wait_for_ready()
    time.sleep(3)
    detect_objects(handle_2d_to_3d)

if __name__=="__main__":
    main()