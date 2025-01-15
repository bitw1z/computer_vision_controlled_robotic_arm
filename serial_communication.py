import serial
import time

arduino = serial.Serial(port='/dev/cu.usbmodem142101', baudrate=9600, timeout=.1)
time.sleep(1) # wait for the connection

def wait_for_ready():
    while True: 
        if arduino.in_waiting > 0: 
            response = arduino.readline().decode().strip()
            if response == "Arduino ready!":
                print("Connection established!")
                break

def send_angles(t1, t2, t3):
    coord_data = f"{t1},{t2},{t3}\n"
    coord = coord_data.encode()
    arduino.write(coord) 
    time.sleep(0.15) # provide arduino time to process
    
    print(f"Sent: {coord_data.strip()}")
    
    # wait for Arduino's response
    while True: 
        if arduino.in_waiting > 0:
            response = arduino.readline().decode().strip()
            if response == "Angles updated!":
                break
                
    
    