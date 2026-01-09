void setup() {
  pinMode(A0, INPUT);
  Serial.begin(9600);

}

void loop() {
  int x = analogRead(A0);
  long int sum = 0;
  long int avg;
  for(long int i = 0; i < 50000; i++){
    sum += x;
  }
  avg = sum/50000;
  if(avg) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
  Serial.println(avg);

}
