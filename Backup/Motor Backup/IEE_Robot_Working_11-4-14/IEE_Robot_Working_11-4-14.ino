#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <PID_v1.h>
#include <TimerOne.h>


typedef double Motor_Speed;

I2CEncoder Motor_1_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_2_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_3_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_4_S; //create an I2CEncoder for measurign speed

//Define Variables

double Setpoint, Input, Input2, Input3;// Input4;

Motor_Speed RPM, *Motor1_Speed=&RPM, Output, *isrOutput = &Output;

Motor_Speed RPM2, *Motor2_Speed=&RPM2, Output2, *isrOutput2=&Output2;

Motor_Speed RPM3, *Motor3_Speed=&RPM3, Output3, *isrOutput3=&Output3;

//Motor_Speed RPM4, *Motor4_Speed=&RPM4, Output4, *isrOutput4=&Output4;

double Kp=1.5, Ki=0.5, Kd=0.01;
PID motor1_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID motor2_PID(&Input2,&Output2,&Setpoint, Kp, Ki, Kd, DIRECT);

PID motor3_PID(&Input3,&Output3,&Setpoint, Kp, Ki, Kd, DIRECT); 
//PID motor4_PID(&Input4,&Output4,&Setpoint, Kp, Ki, Kd, DIRECT);

Servo Motor_1;     // Control Pin for motor Left front
Servo Motor_2;     // Control Pin for motor Left back
Servo Motor_3;     // Control Pin for motor Right back
Servo Motor_4;     // Control Pin for motor Right Front

int minSpeed = 35;    // vex motor servo values for min and max speed
int zeroSpeed = 95;  
int maxSpeed = 155;

char junk = ' ';
int c=0;
int c2=0;
double  rpm_input=0, rpm_input2=0, rpm_input3=0; //rpm_input4 = 0;

void setup() {
  Setpoint=0;
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200);
    // Initialize the encoders for a 269 motors that are moving 2/3 of a
    // foot per motor output shaft rotation.
    Motor_1_S.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_2_S.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_3_S.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    Motor_4_S.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
    
     
    Motor_1.attach(11); 
    Motor_2.attach(10); 
    Motor_3.attach(9); 
    Motor_4.attach(8);
    
    Input=Motor_1_S.getSpeed();
    Input2=Motor_2_S.getSpeed();
    Input3=Motor_3_S.getSpeed();
  //  Input4=Motor_4_S.getSpeed();
	
    motor1_PID.SetMode(AUTOMATIC);
    motor2_PID.SetMode(AUTOMATIC);
    motor3_PID.SetMode(AUTOMATIC);
   // motor4_PID.SetMode(AUTOMATIC);
	
    Timer1.initialize(500000); 
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here 
}

void loop(){
  
rpm_input +=Motor_1_S.getSpeed();
rpm_input2+=Motor_2_S.getSpeed();
rpm_input3+=Motor_3_S.getSpeed();
//rpm_input4+=Motor_4_S.getSpeed();
c++;

if (c>=50){
  RPM=rpm_input/50;
  RPM2=rpm_input2/50;
  RPM3=rpm_input3/50;
  //RPM4=rpm_input4/50;
  
  Input=RPM;
  Input2=RPM2;  
  Input3=RPM3;
  //Input4=RPM4;
  
  motor1_PID.Compute();
  motor2_PID.Compute();
  motor3_PID.Compute();
  //motor4_PID.Compute();
  
  Motor_1.write(0.235294*Output+95);
  Motor_2.write(0.235294*Output2+95);
  Motor_3.write(0.235294*Output3+95);
 // Motor_4.write(0.235294*Output4+95);
  
  c=0;
  rpm_input=0;
  rpm_input2=0;
  rpm_input3=0;
  //rpm_input4=0;
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
  Serial.print(*Motor2_Speed);
  Serial.print(" , ");
  Serial.print(*isrOutput2);
  Serial.print(" , ");
  Serial.print(*Motor3_Speed);
  Serial.print(" , ");
  Serial.print(*isrOutput3);
  Serial.print(" , ");
//  Serial.print(*Motor4_Speed);
//  Serial.print(" , ");
//  Serial.print(*isrOutput4);
//  Serial.print(" , ");
  Serial.print(Setpoint);
  Serial.write("\n"); 
}