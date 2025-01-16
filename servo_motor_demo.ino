#include <VarSpeedServo.h>

// create servo objects
VarSpeedServo base, elbow, shoulder, gripper; 
String inputString = "";

void setup() {
  // attach servo to pins 
  base.attach(6);
  elbow.attach(9);
  shoulder.attach(10);
  gripper.attach(11);

  // initialize angles 
  base.write(10, 80, false); 
  elbow.write(30, 80, false);
  shoulder.write(130, 80, false);
  gripper.write(90, 80, false);

  Serial.begin(9600);
  Serial.println("Arduino ready!");
}

void loop() {
  while (Serial.available()) {
    char inputChar = (char)Serial.read();
    inputString += inputChar;
    if (inputChar == '\n') {
      inputString.trim(); // remove unnecessary space

      // parse angles 
      int angles[3];
      int count = 0;
      char * token = strtok(inputString.c_str(), ",");
      while (token != NULL && count < 3) {
        angles[count] = atoi(token);
        token = strtok(NULL, ",");
        count++;
      }

      if (count == 3) {
        base.write(angles[0], 80, false);
        elbow.write(angles[1], 80, false);
        shoulder.write(angles[2]+10, 80, false); 

         // Gradually move the elbow back to 0°
        int currentAngle = elbow.read();
        while (currentAngle > 10) {
          currentAngle--;
          elbow.write(currentAngle, 30, false); // Slowly decrease angle to 0
        }
        gripper.write(180, 60, false);
        elbow.write(45, 20, false);
        gripper.write(90, 60, false);

        Serial.println("Angles updated!");
      } else {
        Serial.println("Error!");
      }
      
      //reset for next command 
      inputString = "";
    }
  }
}
