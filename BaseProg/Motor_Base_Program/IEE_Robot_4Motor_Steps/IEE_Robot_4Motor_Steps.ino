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

double Setpoint[4], Input, Input2, Input3, Input4;
Motor_Speed RPM, *Motor1_Speed=&RPM, Output, *isrOutput = &Output;
Motor_Speed RPM2, *Motor2_Speed=&RPM2, Output2, *isrOutput2=&Output2;
Motor_Speed RPM3, *Motor3_Speed=&RPM3, Output3, *isrOutput3=&Output3;
Motor_Speed RPM4, *Motor4_Speed=&RPM4, Output4, *isrOutput4=&Output4;

double Kp=0.2, Ki=1, Kd=0.01;



PID motor1_PID(&Input, &Output, &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID motor2_PID(&Input2,&Output2,&Setpoint[1], Kp, Ki, Kd, DIRECT);
PID motor3_PID(&Input3,&Output3,&Setpoint[2], Kp, Ki, Kd, DIRECT); 
PID motor4_PID(&Input4,&Output4,&Setpoint[3], Kp, Ki, Kd, DIRECT);

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
double  rpm_input=0, rpm_input2=0, rpm_input3=0, rpm_input4 = 0;

void setup() {
  
  
  Setpoint[0]=60;
  Setpoint[1]=60;
  Setpoint[2]=60;
  Setpoint[3]=60;
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
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
    Motor_4.attach(6);
    
    Input=Motor_1_S.getSpeed();
    Input2=Motor_2_S.getSpeed();
    Input3=Motor_3_S.getSpeed();
    Input4=Motor_4_S.getSpeed();
	
    motor1_PID.SetMode(AUTOMATIC);
    motor2_PID.SetMode(AUTOMATIC);
    motor3_PID.SetMode(AUTOMATIC);
    motor4_PID.SetMode(AUTOMATIC);
	
//    Timer1.initialize(500000); 
//    Timer1.attachInterrupt( timerIsr ); // attach the service routine here 
}

void loop(){
  
rpm_input +=Motor_1_S.getSpeed();
rpm_input2+=Motor_2_S.getSpeed();
rpm_input3+=Motor_3_S.getSpeed();
rpm_input4+=Motor_4_S.getSpeed();
c++;
c2++;
if (c>=10){
  RPM=rpm_input/10;
  RPM2=rpm_input2/10;
  RPM3=rpm_input3/10;
  RPM4=rpm_input4/10;
  
  Input=RPM;
  Input2=RPM2;  
  Input3=RPM3;
  Input4=RPM4;
  
  motor1_PID.Compute();
  motor2_PID.Compute();
  motor3_PID.Compute();
  motor4_PID.Compute();
  
  run_sideways();
  
  c=0;
  rpm_input=0;
  rpm_input2=0;
  rpm_input3=0;
  rpm_input4=0;
}
//if (c2>=4000){
// Setpoint=130;
// digitalWrite(13, LOW);
// c2=0;
//}
delay(3);
}


void serialEvent ()



  {
    while (Serial.available())
      {
      int serial_input = Serial.parseInt();
      if (serial_input != 0)
        Setpoint[0] = serial_input;
      }    
  }
  
  //all 4 motors in time
  void run_straight(int x)
  
  {
	for (int z=0;z<4;z++)
	{
        Setpoint[z] = x;
	}
	Motor_1.write(0.235294*Output+95);
	Motor_2.write(0.235294*Output2+95);
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(0.235294*Output4+95);
  }
  
  
  //motor 3 + 4 forward
  //Motor 1+2 in reverse
 void turn_right ()
 {
	Setpoint[0] = 100;
	Setpoint[1] = 100;
	Setpoint[2] = 100;
	Setpoint[3] = 100;
	Motor_1.write(-0.058537*Output+95);  //reverse sloap
	Motor_2.write(-0.058537*Output2+95); //reverse sloap
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(0.235294*Output4+95);
 }
 
 //motor 1+2 Forward
 //motor 3+4 reverse
 void turn_left()
 {
   	Setpoint[0] = 100;
	Setpoint[1] = 100;
	Setpoint[2] = 100;
	Setpoint[3] = 100;
	Motor_1.write(0.235294*Output+95);
	Motor_2.write(0.235294*Output2+95);
	Motor_3.write(-0.058537*Output3+95);
	Motor_4.write(-0.058537*Output4+95);
 }

 //motor 1+2 faster
 //motor 3+4 in reverse
 //void turn_left()
 
 void run_sideways()
 //4+3 turn in   4 turn reverse   3 turn forward
 //1+2 turn out  1 turn forward  2 turn reverse
 {
   	Setpoint[0] = 100;
	Setpoint[1] = 100;
	Setpoint[2] = 100;
	Setpoint[3] = 100;
	Motor_1.write(0.235294*Output+95);  
	Motor_2.write(-0.058537*Output2+95); 
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(-0.058537*Output4+95);
 }
 
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
//void timerIsr()
//{
//
//  Serial.print(*Motor1_Speed); // Get how fast it's rotating
//  Serial.write(" , ");
//  Serial.print(0.235294*(*isrOutput)+95); // Get how fast it's rotating
//  Serial.write(" , ");
//  Serial.print(*Motor2_Speed);
//  Serial.print(" , ");
//  Serial.print(*isrOutput2);
//  Serial.print(" , ");
//  Serial.print(*Motor3_Speed);
//  Serial.print(" , ");
//  Serial.print(*isrOutput3);
//  Serial.print(" , ");
//  Serial.print(*Motor4_Speed);
//  Serial.print(" , ");
//  //Serial.print(*isrOutput4);
//  //Serial.print(" , ");
//  Serial.print(Setpoint);
//  Serial.write("\n"); 
//}
