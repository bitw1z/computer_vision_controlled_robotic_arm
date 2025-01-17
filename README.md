# computer_vision_controlled_robotic_arm
## Introduction
This project enables a robotic arm to autonomously manipulate objects using ESP32 camera for real-time object detection and Arduino Uno for servo motor controls. 

## Features
- **Real-time Object Detection**: Utilizes an ESP32 camera for object detection and pretrained YOLOv3 weights to identify objects in the video stream.
- **Robotic Arm Control**: The arm is controlled using inverse kinematics to calculate joint angles required to move the arm to a desired position.
- **Forward Kinematics Verification**: Modified Denavit-Hartenberg(DH) parameters are used to verify the inverse kinematics calculations.
- **Serial Communication**: Python sends joint angles data to the Arduino via serial communication, allowing Arduino Uno to control the servo motors.
- **Servo Motor Control**: Uses the Arduino VarSpeedServo library to control the speed of servo motors for smoothness and structural stability.

## Hardware Setup
- **ESP32 Camera**: Positioned in a bird's eye view to provide a real-time video stream for object detection. 
- **Arduino Uno**: Serves as the controller for the servo motors, receiving joint angles data from Python and sending status updates back to Python. 
- **Servo Motors**: Control the movements of the robotic arm based on the computed joint angles. 
- **3-DoF Robotic Arm**: A custom-built prototype assembled using upcycled materials to promote sustainability. 

### Image of Hardware Connections 
![Connection Setup Photo](hardware_connections.jpg)
