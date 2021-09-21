int led = 13;

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  delay(1000);
}

void loop() {
  
  for(int i=0; i<5; i++){
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(100);
  }
  digitalWrite(led, HIGH);
  while(1){}
}
