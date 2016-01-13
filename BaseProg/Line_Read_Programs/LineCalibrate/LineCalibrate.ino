//Keith Conley
//Fall 2014

int lineReadpins[8] = {53, 51, 49, 47, 45, 43, 41, 39};
bool lineSENSE[8] = {false, false, false, false, false, false, false, false};

void setup()
{
	Serial.begin(115200);
	for (int i =0; i<8; i++)
		{
			pinMode(lineReadpins[i],INPUT);
		}
}

void loop()
  {
        for (int i = 0; i<8;i++)
        {
        	  lineCALIBRATE(lineReadpins[i]);
        
            		if (lineSENSE[i]==true)
          		{
                            Serial.print("Sensor ");
              		    Serial.print(i);
              		    Serial.print(" is White");
              		    Serial.println();
                       }
        }
  }


bool lineREAD(int pinREAD)
//WHITE = TRUE
//BLACK = FALSE

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
  if (counter < COLORCUTOFF)
  {
  COLOR = true;
  }
  else
  {
  COLOR = false;
  }
  return COLOR;
}

void lineCALIBRATE(int pinREAD)
//WHITE = TRUE
//BLACK = FALSE

{
	int counter = 0;         //counter variable

	
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
Serial.print("Sensor "); Serial.print(pinREAD); Serial.print("IS : "); Serial.println(counter);
}
