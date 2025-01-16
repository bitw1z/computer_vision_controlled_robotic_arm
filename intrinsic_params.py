# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
import numpy as np
import cv2
import glob

savedir = "camera_data/" 
 
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
 
# arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane
 
images = glob.glob('*.jpg')
 
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
    # find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (6,8), None)
 
    # if found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # draw and display the corners
        cv2.drawChessboardCorners(img, (6,8), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
 
if cv2.waitKey(1) & 0xFF == ord('q'):
    cv2.destroyAllWindows()

print("Starting calibration")
ret, cam_mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    np.save(savedir+'cam_mtx.npy', cam_mtx)
    np.save(savedir+'dist_coeff.npy', dist)

print("Calibration ended")
