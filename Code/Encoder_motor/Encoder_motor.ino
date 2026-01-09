volatile int prev_time;
volatile int current_time;

void IRAM_ATTR time_stamp() {
  current_time = millis() - prev_time;
  prev_time = millis();
  Serial.println((60000/current_time)/(244*2));
}

void setup() {
  // put your setup code here, to run once:
  pinMode(23, INPUT);
  Serial.begin(115200);
  attachInterrupt(23, time_stamp, FALLING);
  delay(5000);
}

void loop() {

}

