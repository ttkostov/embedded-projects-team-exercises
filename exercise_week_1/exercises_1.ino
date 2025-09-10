void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(34, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  int sensorValue = analogRead(A0);
  Serial.println(sensorValue);


  float voltage= sensorValue * (5.0 / 1023.0);
  Serial.print(voltage);
  Serial.println("V");

  if(voltage <= 2) {
    Serial.println("inside if");
    digitalWrite(34, HIGH);
    //&analogWrite(34, 1023);
  }
  else {
    digitalWrite(34, LOW);
  }

  delay(1000);  

}
