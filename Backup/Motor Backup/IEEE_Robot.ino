
#include <Servo.h>
#include <Wire.h>
#include "C:\Users\LabView\Desktop\arduino-1.0.3\libraries\I2CEncoder\I2CEncoder.h"


I2CEncoder encoder1; //create an I2CEncoder for measurign speed

Servo Moto_LF;     // Control Pin for motor Left front
Servo Moto_LB;     // Control Pin for motor Left back
Servo Moto_RB;     // Control Pin for motor Right back
Servo Moto_RF;     // Control Pin for motor Right Front



int minSpeed = 35;    // vex motor servo values for min and max speed
int zeroSpeed = 95;  
int maxSpeed = 155;


void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  
  // Initialize the encoders for a 269 motors that are moving 2/3 of a
  // foot per motor output shaft rotation.
  encoder1.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  
  
  Moto_LF.attach(11); 
  Moto_RB.attach(10); 
  Moto_LB.attach(9); 
  Moto_RF.attach(8); 

}

void loop(){
//double speed = (leftEncoder.getSpeed()) / 60.0;
//Serial.print(speed);
//Serial.println(" feet per second.");
  
Moto_LF.write(95);
Moto_LB.write(95);

Moto_RF.write(95);
Moto_RB.write(95);

delay(500);

}
