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
  pinMode(pin51,OUTPUT);
  pinMode(pin49,OUTPUT);
  pinMode(pin47,OUTPUT);
  pinMode(pin45,OUTPUT);
  pinMode(pin43,OUTPUT);
  pinMode(pin41,OUTPUT);
  pinMode(pin39,OUTPUT);
  
  digitalWrite(pin53, HIGH);
  digitalWrite(pin51, HIGH);
  digitalWrite(pin49, HIGH);
  digitalWrite(pin47, HIGH);
  digitalWrite(pin45, HIGH);
  digitalWrite(pin43, HIGH);
  digitalWrite(pin41, HIGH);
  digitalWrite(pin39, HIGH);
  delayMicroseconds(10);
  
  pinMode(pin53,INPUT);
  pinMode(pin51,INPUT);
  pinMode(pin49,INPUT);
  pinMode(pin47,INPUT);
  pinMode(pin45,INPUT);
  pinMode(pin43,INPUT);
  pinMode(pin41,INPUT);
  pinMode(pin39,INPUT);
  
  while (digitalRead(pin53) == 1){
    i++;
    delayMicroseconds(1);
  }
  Serial.println(i);  
  delay(500);
}   


