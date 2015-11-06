void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600) ;
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead (A0) ;
  float volts = sensorValue * 5.0 / 1023. ;
  float degrees = (volts - 2.5) * 25. ;
  Serial.println (degrees) ;
  delay (800) ;
}
