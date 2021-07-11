int ANA_PIN[] = { 1, 2, 3, 4, 5};
int BTN1_PIN[] = {2, 3}; 

void setup() {
  pinMode(BTN1_PIN[0], INPUT_PULLUP);
  pinMode(BTN1_PIN[1], INPUT_PULLUP);

//  for(int i=0; i<5; i++)
//  {
//    pinMode(ANA_PIN[i], INPUT);
//  }

  Serial.begin(9600);
  
}

void loop() {
  
  int potVal[5];
  //header
  Serial.print("ss ");
  //read all value
  for(int i=0; i<5; i++)
  {
    potVal[i] = analogRead(ANA_PIN[i]);
    Serial.print(potVal[i]);
    Serial.print(" ");
  }
  if(digitalRead(BTN1_PIN[0]) == 0)
  {
    Serial.print("-1 ");
  }else if(digitalRead(BTN1_PIN[1]) == 0)
  {
    Serial.print("1 ");
  }else
  {
    Serial.print("0 ");
  }
  Serial.println("ee");

}
