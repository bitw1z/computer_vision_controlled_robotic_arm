# computer_vision_controlled_robotic_arm
## Introduction
This project enables a 3-DoF robotic arm built by upcycled materials to autonomously manipulate objects using ESP32 camera for real-time object detection and Arduino Uno for servo motor controls. 

## Features
- **Joint Angles Calculation**: Computes joint angles using forward and inverse kinematics. [Explore more](https://bitwiz03.medium.com/start-of-my-journey-into-computer-vision-controlled-3-dof-robotic-arm-3353c88c40bf).
- **Iterative Design**: Refines the robotic arm prototype to ensure stabiltiy and balance of structure. [Explore more](https://bitwiz03.medium.com/evaluating-and-enhancing-my-3-dof-robotic-arm-hardware-software-0fe39215a9fd).
- **Serial Communication and Servo Motor Control**: Enables efficient data exchange between Python and Arduino. Controls servo motors to move the robotic arm smoothly. [Explore more](https://bitwiz03.medium.com/fine-tuning-control-systems-optimizing-motor-algorithms-and-communication-protocols-for-robotic-a84301adf23b).
- **Real-Time Object detection**: Leverages pretrained YOLOv3 weights for object detection. [Explore more](https://bitwiz03.medium.com/adding-vision-to-robotic-arm-setting-up-the-esp32-camera-for-object-detection-d72eb6692d51).
- **ESP32 Camera Calibration**:  Calibrates the ESP32 camera to obtain intrisic and extrinsic parameters and therefore converts 2D pixel coordinates to 3D world coordinates. [Explore more](https://bitwiz03.medium.com/bridging-dimensions-camera-calibration-for-2d-to-3d-mapping-3d2b0a060a6f).

## Hardware Setup
- **3-DoF Robotic Arm**: A custom-built prototype assembled using upcycled materials to promote sustainability.
- **Arduino Uno**: Serves as the controller for the servo motors, receiving joint angles data from Python and sending status updates back to Python. 
- **Servo Motors**: Control the movements of the robotic arm based on the computed joint angles. 
- **ESP32 Camera**: Positioned in a bird's eye view to provide a real-time video stream for object detection. 

### Image of Hardware Connections 
![Connection Setup Photo](hardware_connections.jpg)

## Software Setup
### **1. ESP32 Camera Setup**
The ESP32 camera is configured using the **CameraWebServer** example code from the ESP32 library, enabling video streaming accessible via a specific IP address. 

### **2. Object Detection with YOLOv3**
- **Pretrained YOLOv3 Weights**: Utilizes pretrained YOLOv3 weights and COCO classes for object detection.
- **Bounding Box and Non-Maximum Suprression**: After detecting object, redundant boxes are removed by NMS, and the most optimal bounding box is drawn around the identified object in the camera feed.

### **3. Kinematics**
- **Inverse Kinematics**: Applies geometric approach to calculates the joint angles required for the robotic arm to reach a specific 3D position of the object. 
- **Forward Kinematics**: Uses modified Denavit-Hartenberg (DH) parameters to verify that the computed joint angles correctly map to the desired 3D coordinates.

### **4. Serial Communication**
- **Python-Arduino Communication**: The Python script sends joint angles data to the Arduino, which interprets the data and moves the servo motors accordingly. 
- **Synchronization**: The Arduino updates the status after completing the servo motors movements. Communication is synchronized using a **handshaking system**, ensuring smooth data exchange.

### **5. Servo Motor Control**
- **VarSpeedServoLibrary**: Leverages the VarSpeedServo library to control the speed of servo motors, ensuring smooth movement and maintaining strcutural stability. 

## Installation  
### **1. Clone the Repository**  
```bash
git clone https://github.com/bitwizd03/computer_vision_controlled_robotic_arm.git
cd computer_vision_controlled_robotic_arm
```
### **2. Install Dependencies**
- **Python**: Install the necessary Python libraries using pip.
```bash
pip install numpy opencv-python pyserial sympy
```
- **Arduino**:
  1. Open the Arduino IDE and install the ESP32 library by Espressif from the Library Manager.
  2. Download the VarSpeedServo library from [github](https://github.com/netlabtoolkit/VarSpeedServo) as a ZIP file.
  3. Upload the ZIP file to the Arduino IDE using Sketch > Include Library > Add .ZIP Library.

### **3. ESP32 Camera Setup** 
- Open the CameraWebServer example code in the Arduino IDE and update the code with your local Wi-Fi network name and password.
- For detailed settings and connections, refer to this [video tutorial](https://www.youtube.com/watch?v=7-3piBHV1W0)! 

### **4. Run the Python Script**
- Update the **forward_kinematics** code with your robotic arm's modified DH parameters.
- Configure the **serial_communication** code to match your local Arduino port.
- Try video streaming by matching the IP address of your ESP32 camera in **video streaming** code. 
- Change the IP address to your ESP32 camera in **pixel coordinates** and **object detection** code as well. 
- Run the **main** scipt to begin object detection, calculate joint angles, and control the robotic arm:
```bash
python main.py
```
## Results
 Here's a performance video on YouTube: [Watch it here](https://youtu.be/zEXQ5RJERkk?si=U1XYCbymmyTSmgoa). For more details, feel free to check out my [blog post!](https://bitwiz03.medium.com/turning-ideas-into-reality-project-completion-and-new-frontier-43c1b88f5ec7)
- Stable structure and smooth movements
- Object detection through ESP32 camera
- Precise 2D-to-3D coordinate conversion


## Future Improvements 
- Incrementally adding degrees of freedom, up to six, will enable precise control of the end effector's orientation, thereby enhancing manipulation capabilities.
- Using pretrained YOLO weights limits the range of detectable objects, and the performance of object detection varies depending on the lighting conditions. To address this, standardizing the lighting conditions and and collecting images of specific objects for manipulation will improve accuracy and provide more freedom of object choices. 
- The robotic arm peforms repeated motion due to delayed responsiveness in the object detection algorithm. To mitigate this, I can utilize tiny YOLO weights, implement asychronous processing, optimize input resolution, and process fewer frames by skipping detections. 
  
## Acknolwedgements
- Numpy library for forward and inverse kinematics of robotic arm.
- Arduino and the VarSpeedServo library for servo control.
- The ESP32 Camera library and CameraWebServer example code.
- YOLOv3 weights and coco.names for object detection.
- OpenCV library for intrinsic and extrinsic parameters extraction.


