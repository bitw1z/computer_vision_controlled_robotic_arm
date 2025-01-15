import cv2
import numpy as np

# load YOLO
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# load COCO names (class labels)
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# set up the camera stream
url = 'http://192.168.45.94:81/stream'  # ESP32 camera URL
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: Unable to open video stream.")
    exit()

def detect_objects(callback)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break

        # prepare the image for YOLO
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), 
                                     True, crop=False)
        net.setInput(blob)
        outputs = net.forward(output_layers)

        # process detections
        class_ids = []
        confidences = []
        boxes = []
        height, width, channels = frame.shape

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:  # filter by confidence threshold
                    # get bounding box coordinates
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # apply non-maxima suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # draw bounding boxes
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                
                # pass the 2D centers to the callback
                cx = x + 0.5*w
                cy = y + 0.5*h
                callback(x, y)

                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # show the resulting frame
        cv2.imshow('ESP32 Object Detection', frame)

        # break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # release resources
    cap.release()
    cv2.destroyAllWindows()



