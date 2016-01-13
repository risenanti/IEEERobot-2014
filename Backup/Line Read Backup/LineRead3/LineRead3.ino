int lineReadpins[8] = {53, 51, 49, 47, 45, 43, 41, 39};

//lineReadpins[0]=53;
//lineReadpins[1]=51;
//lineReadpins[2]=49;
//lineReadpins[3]=47;
//lineReadpins[4]=45;
//lineReadpins[5]=43;
//lineReadpins[6]=41;
//lineReadpins[7]=39;

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
  int j = 0;
	for (int i =0; i<8; i++)
		{
			pinMode(lineReadpins[i],OUTPUT);
		}
		
	for (int i =0; i<8; i++)
		{
			digitalWrite(lineReadpins[i],HIGH);
		}
	delayMicroseconds(10);

	for (int i =0; i<8; i++)
		{
			pinMode(lineReadpins[i],INPUT);
		}
	while (digitalRead(lineReadpins[0]) == 1){
    j++;
    delayMicroseconds(1);
  }
  Serial.println(j);  
  delay(500);
}
