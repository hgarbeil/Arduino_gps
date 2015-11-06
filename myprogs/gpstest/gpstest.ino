void setup() {
  // put your setup code here, to run once:
  //pinMode (0,INPUT);
  //pinMode (1, OUTPUT) ;
  Serial.begin (4800) ;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    Serial.write (Serial.read()) ;
  }
}
