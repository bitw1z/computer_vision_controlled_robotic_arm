import cv2 
import numpy as np 

loaddir = "camera_data/" 
cameraMatrix = np.load(loaddir+'cam_mtx.npy')

# focal length and center pixels from intrisic parameters 
fx = cameraMatrix[0][0]
fy = cameraMatrix[1][1]
cx = cameraMatrix[0][2]
cy = cameraMatrix[1][2]

# rotation and translation matrix from world to camera coordinate system
R_cw = np.load(loaddir+'cam_rotation.npy')
t_cw = np.load(loaddir+'cam_translation.npy')

# given pixel values, convert it to real world coordinate system 
def convert_2D_to_3D(u, v):
    x = (u - cx) / fx
    y = (v - cy) / fy
    P_c = np.array([[x, y, 1]]).T
    O_c = np.array([[0, 0, 0]]).T

    R_wc = R_cw.T
    P_w = np.dot(R_wc, (P_c - t_cw))
    O_w = np.dot(R_wc, (O_c - t_cw))

    line = P_w - O_w
    line_z = line[2][0]
    O_w_z = O_w[2][0]

    k = (-1)*(O_w_z/line_z) # 0 = O_w_z + k*line_z
    P = O_w + k*(P_w - O_w)
    return P
