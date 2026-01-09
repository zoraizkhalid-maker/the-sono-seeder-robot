#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define S_1 15
#define S_2 2
#define Seed 23

#define in1 32
#define in2 33
#define in3 25
#define in4 26

Servo Seeding;
Servo Servo2;
Servo Servo1;

int angle = 75;


void setup() {

  Seeding.attach(Seed);
  Servo2.attach(S_2);
  Servo1.attach(S_1);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(115200);
  SerialBT.begin("ESP32test");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {

  if (SerialBT.available()) {
    char x = SerialBT.read();
    Serial.println(x);

    if (x == 'F') forward(200);
    if (x == 'L') left(200);
    if (x == 'R') right(200);
    if (x == 'B') stop();

    if (x == 'T') {
    }
    if (x == 'X') {
    }
    if (x == 'C') {
    }
    if (x == 'S') {
    }
  }
  Seeding.write(90);
  delay(1000);
  Seeding.write(90 - angle);
  delay(500);
  Seeding.write(90);
  delay(1000);
  Seeding.write(90 + angle);
  delay(500);
}

void forward(int speed) {
  analogWrite(in1, speed);
  analogWrite(in2, 0);
  analogWrite(in3, speed);
  analogWrite(in4, 0);
}

void right(int speed) {
  analogWrite(in1, speed);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  analogWrite(in4, speed);
}

void left(int speed) {
  analogWrite(in1, 0);
  analogWrite(in2, speed);
  analogWrite(in3, speed);
  analogWrite(in4, 0);
}

void stop() {
  analogWrite(in1, 0);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  analogWrite(in4, 0);
}
