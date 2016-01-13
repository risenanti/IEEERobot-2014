
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <PID_v1.h>
#include <TimerOne.h>


typedef double Motor_Speed;

I2CEncoder Motor_1_F; //create an I2CEncoder for measurign speed
I2CEncoder Motor_2_F; //create an I2CEncoder for measurign speed
I2CEncoder Motor_3_F; //create an I2CEncoder for measurign speed
I2CEncoder Motor_4_F; //create an I2CEncoder for measurign speed

//Define Variables we'll be connecting to
double Setpoint, Input;

Motor_Speed RPM, *Motor1_Speed = &RPM, Output, 

*isrOutput=&Output;

double Kp=1.5, Ki=0.5, Kd=0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


Servo Moto_LF;     // Control Pin for motor Left front
Servo Moto_LB;     // Control Pin for motor Left back
Servo Moto_RB;     // Control Pin for motor Right back
Servo Moto_RF;     // Control Pin for motor Right Front

int minSpeed = 35;    // vex motor servo values for min and max speed
int zeroSpeed = 95;  
int maxSpeed = 155;

char junk = ' ';
int c=0;
int c2=0;
double  rpm_input=0;
void setup() {
  Setpoint=0;
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200);
    // Initialize the encoders for a 269 motors that are moving 2/3 of a
    // foot per motor output shaft rotation.
    Motor_1_F.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_2_F.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_3_F.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_4_F.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    
     
    Moto_LF.attach(11); 
    Moto_RB.attach(10); 
    Moto_LB.attach(9); 
    Moto_RF.attach(8);
    
    Input=Motor_1_F.getSpeed();
    myPID.SetMode(AUTOMATIC);
    
    Timer1.initialize(500000); 
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here 
}

void loop(){
rpm_input=rpm_input+Motor_1_F.getSpeed();
c++;
if (c>=50){
  RPM=rpm_input/50;
  Input=RPM;  
  myPID.Compute();
  Moto_LF.write(0.235294*Output+95);
  c=0;
  rpm_input=0;
}
delayMicroseconds(10);
}



void serialEvent ()
  {
    while (Serial.available())
      {
      int serial_input = Serial.parseInt();
      if (serial_input != 0)
        Setpoint = serial_input;
      }    
  }

/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{

  Serial.print(*Motor1_Speed); // Get how fast it's rotating
  Serial.write(" , ");
  Serial.print(0.235294*(*isrOutput)+95); // Get how fast it's rotating
  Serial.write(" , ");
  Serial.print(Setpoint);
  Serial.write("\n");
  
  
}

