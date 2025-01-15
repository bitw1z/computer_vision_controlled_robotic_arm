import cv2

print("Started")
url = 'http://192.168.45.94:81/stream'  # ESP32 camera URL
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: Unable to open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    cv2.imshow('ESP32 OV2460', frame)

    # break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release resources
cap.release()
cv2.destroyAllWindows()