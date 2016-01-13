int pin53 = 53;

void setup()
{
  Serial.begin(115200);
  
  pinMode(53,INPUT);
}

void loop()
{
  int i=0;
  pinMode(pin53,OUTPUT);
  digitalWrite(pin53, HIGH);
  delayMicroseconds(10);
  pinMode(pin53,INPUT);
  
  while (digitalRead(pin53) == 1){
    i++;
    delayMicroseconds(1);
  }
  Serial.println(i);  
  delay(500);
}   


