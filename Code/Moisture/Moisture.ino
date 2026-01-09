void setup() {
  pinMode(36, INPUT);
  analogReadResolution(10);
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(36));
}
