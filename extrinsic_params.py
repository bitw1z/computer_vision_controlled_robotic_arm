import cv2
import numpy as np

# 2D pixel coordinates
points_2D = np.array([
                        (30, 195),  # Left Bottom
                        (219,195),  # Right Bottom
                        (30, 64),  # Left Top
                        (219, 64),  # Right Top
                      ], dtype="double")
                      
# 3D world coordiantes 
points_3D = np.array([
                      (4.0, 12.0, 0.0),     
                      (17.0, 12.0, 0.0),       
                      (4.0, 3.0, 0.0),       
                      (17.0, 3.0, 0.0)         
                     ], dtype="double")

# camera intrinsic parameter 
loaddir = "camera_data/" 
cameraMatrix = np.load(loaddir+'cam_mtx.npy')

# pass null values 
dist_coeffs = np.zeros((4, 1))

# solvePnp 
print("Starting calculation")
retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cameraMatrix, 
                                  dist_coeffs, rvec=None, tvec=None, 
                                  useExtrinsicGuess=None, flags=None)

if retval: 
  rvec, _ = cv2.Rodrigues(rvec) # rotation matrix only
  np.save(loaddir+'cam_rotation.npy', rvec)
  np.save(loaddir+'cam_translation.npy', tvec)

print("Calculation ended")