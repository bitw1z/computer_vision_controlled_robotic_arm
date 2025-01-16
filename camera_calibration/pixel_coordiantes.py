import cv2 
import numpy as np 

def mouse_callback(event, x, y, flags, param):
    print(f"mouse event, x: {x}, y: {y}")
    
url = 'http://192.168.45.94:81/stream'  # ESP32 camera URL
cap = cv2.VideoCapture(url)

# set resolution explicitly
ret, frame = cap.read()

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_callback)

while True:
    cv2.imshow('frame', frame)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    
cv2.destroyAllWindows()
