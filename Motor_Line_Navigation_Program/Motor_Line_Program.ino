//IEEE Robot Fall 2014

//DEPENDENCIES
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <PID_v1.h>
#include <TimerOne.h>

typedef double Motor_Speed;


//Line Sense Variables
int lineReadpins[8] = {53, 51, 49, 47, 45, 43, 41, 39};
bool lineSENSE[8] = {false, false, false, false, false, false, false, false};

I2CEncoder Motor_1_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_2_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_3_S; //create an I2CEncoder for measurign speed
I2CEncoder Motor_4_S; //create an I2CEncoder for measurign speed

//Define Variables

double Setpoint1, Setpoint2, Setpoint3, Setpoint4, Input, Input2, Input3, Input4;
Motor_Speed RPM, *Motor1_Speed=&RPM, Output, *isrOutput = &Output;
Motor_Speed RPM2, *Motor2_Speed=&RPM2, Output2, *isrOutput2=&Output2;
Motor_Speed RPM3, *Motor3_Speed=&RPM3, Output3, *isrOutput3=&Output3;
Motor_Speed RPM4, *Motor4_Speed=&RPM4, Output4, *isrOutput4=&Output4;

double Kp=0.2, Ki=1, Kd=0.01;



PID motor1_PID(&Input, &Output, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID motor2_PID(&Input2,&Output2,&Setpoint2, Kp, Ki, Kd, DIRECT);
PID motor3_PID(&Input3,&Output3,&Setpoint3, Kp, Ki, Kd, DIRECT); 
PID motor4_PID(&Input4,&Output4,&Setpoint4, Kp, Ki, Kd, DIRECT);

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
  

  Setpoint1 = 60;
  Setpoint2 = 60;
  Setpoint3 = 60;
  Setpoint4 = 60;
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
  

lineSENSE[0] = lineREAD(lineReadpins[0]);
lineSENSE[1] = lineREAD(lineReadpins[1]);
lineSENSE[2] = lineREAD(lineReadpins[2]);
lineSENSE[3] = lineREAD(lineReadpins[3]);
lineSENSE[4] = lineREAD(lineReadpins[4]);
lineSENSE[5] = lineREAD(lineReadpins[5]);
lineSENSE[6] = lineREAD(lineReadpins[6]);
lineSENSE[7] = lineREAD(lineReadpins[7]);

  lineFollow();
}


  
  //all 4 motors in time
  void run_straight()
  
  {
	Motor_1.write(0.235294*Output+95);
	Motor_2.write(0.235294*Output2+95);
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(0.235294*Output4+95);
  }
  
  
  //motor 3 + 4 forward
  //Motor 1+2 in reverse
 void turn_right ()
 {
	Setpoint1 = 60;
	Setpoint2 = 60;
	Setpoint3 = 60;
	Setpoint4 = 60;
	Motor_1.write(-0.058537*Output+95);  //reverse sloap
	Motor_2.write(-0.058537*Output2+95); //reverse sloap
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(0.235294*Output4+95);
 }
 
 //motor 1+2 Forward
 //motor 3+4 reverse
 void turn_left()
 {
   	Setpoint1 = 60;
	Setpoint2 = 60;
	Setpoint3 = 30;
	Setpoint4 = 30;
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
   	Setpoint1 = 60;
	Setpoint2 = 60;
	Setpoint3 = 60;
	Setpoint4 = 60;
	Motor_1.write(0.235294*Output+95);  
	Motor_2.write(-0.058537*Output2+95); 
	Motor_3.write(0.235294*Output3+95);
	Motor_4.write(-0.058537*Output4+95);
 }
 
void lineFollow()
{
//negative numbers mean line on right side
//positive mean line on left side 
//sensor 3+4 = 0
//sensor 1&&2 = 5 each
//sensor 0 = 10

//sensor 5+6 = -5 each
//sensor 7 = -10



rpm_input +=Motor_1_S.getSpeed();
rpm_input2+=Motor_2_S.getSpeed();
rpm_input3+=Motor_3_S.getSpeed();
rpm_input4+=Motor_4_S.getSpeed();

c++;
if (c>=10)
{

  RPM=rpm_input/10;
  RPM2=rpm_input2/10;
  RPM3=rpm_input3/10;
  RPM4=rpm_input4/10;
  
  Input=RPM;
  Input2=RPM2;  
  Input3=RPM3;
  Input4=RPM4;
  
if (lineSENSE[2]==true)
{
  Input=(RPM-10);
  Input2=(RPM2-10);  
  Input3=RPM3;
  Input4=RPM4;
}
else if (lineSENSE[5]==true)
{
  Input=RPM;
  Input2=RPM2;  
  Input3=(RPM3-10);
  Input4=(RPM4-10);
}
if (lineSENSE[1]==true)
{
  Input=(RPM-10);
  Input2=(RPM2-10);  
  Input3=RPM3;
  Input4=RPM4;
}
else if (lineSENSE[6]==true)
{
  Input=RPM;
  Input2=RPM2;  
  Input3=(RPM3-10);
  Input4=(RPM4-10);
}


if (lineSENSE[0]==true)
{
  Input=(RPM-20);
  Input2=(RPM2-20);  
  Input3=RPM3;
  Input4=RPM4;
}
else if (lineSENSE[7]==true)
{
  Input=RPM;
  Input2=RPM2;  
  Input3=(RPM3-20);
  Input4=(RPM4-20);
}

if (lineSENSE[0]&&lineSENSE[1]&&lineSENSE[2]&&lineSENSE[3]&&lineSENSE[4]&&lineSENSE[5]&&lineSENSE[6]&&lineSENSE[7])
{
        Setpoint1=0;
        Setpoint2=0;
        Setpoint3=0;
        Setpoint4=0;
	Motor_1.write(95);
	Motor_2.write(95);
	Motor_3.write(95);
	Motor_4.write(95);
}

  motor1_PID.Compute();
  motor2_PID.Compute();
  motor3_PID.Compute();
  motor4_PID.Compute();
  c=0;
  rpm_input=0;
  rpm_input2=0;
  rpm_input3=0;
  rpm_input4=0;
}
run_straight();
}


bool lineREAD(int pinREAD)
//WHITE = FALSE
//BLACK = TRUE

{
	int COLORCUTOFF = 50;    //if under 50 microseconds white. Else  black
	int counter = 0;         //counter variable
	bool COLOR;              //color value
	
    //First Write HIGH to pin.
    //Wait 10 microseconds to fill up capacitor
    //switch to high imedence mode. wait for capacitor to drain and increment counter
    pinMode(pinREAD, OUTPUT);
    digitalWrite(pinREAD, HIGH);
    delayMicroseconds(10);
    pinMode(pinREAD,INPUT);
  
    while (digitalRead(pinREAD)==1){
          counter++;
          delayMicroseconds(1);
    }


  if (counter > 100)
  {
  COLOR = true;
  }
  else
  {
  COLOR = false;
  }
  return COLOR;
}
