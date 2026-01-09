#include <ESP32Servo.h>
#include <DFRobot_QMC5883.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

DFRobot_QMC5883 compass(&Wire, 0x0D);

#define Seeding_pin 13
#define Front_left_pin 12
#define Front_right_pin 14
#define Pump 19
#define Heat 18
#define Ultrasound 5

#define in1 33
#define in2 32
#define in3 26
#define in4 25

Servo Seeding;
Servo Front_left;
Servo Front_right;

int angle_err = 1;
int yaw = 0;
int heading;
int prev_heading;

const float hard_iron[3] = {
  65.203435, 54.533258, 164.871260
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  { 0.477467, -0.001965, -0.044405 },
  { -0.001965, 0.472935, -0.036471 },
  { -0.044405, -0.036471, 0.473685 }
};

int base_pwm = 100;
double tracking = 0;
int Seeding_pos = 0;
int state = 0;
long long d_prev_time, d_elapsed_time;
long long s_elapsed_time = 0;
volatile long long prev_time;
volatile long long elapsed_time;

double RPM, PWM, turn_PWM, Target_turn, Target;
double Distance_X = 0, Distance_Y = 0;
int in_Distance_X, in_Distance_Y, Iterations;
int Ultrasound_freq = 40;

void IRAM_ATTR time_stamp() {
  elapsed_time = micros() - prev_time;
  prev_time = micros();
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test");

  while (!SerialBT.available())
    ;
  SerialBT.println("Welcome to SonoSeeder");
  delay(1000);

  SerialBT.println("Please input the following operation parameters:");
  String temp = SerialBT.readString();

  SerialBT.println("Please enter field length /cm");
  while (!SerialBT.available())
    ;
  in_Distance_Y = SerialBT.readString().toInt();

  SerialBT.println("Please enter field width /cm");
  while (!SerialBT.available())
    ;
  in_Distance_X = SerialBT.readString().toInt();

  SerialBT.println("Please enter path divisions");
  while (!SerialBT.available())
    ;
  Iterations = SerialBT.readString().toInt();

  SerialBT.println("Please enter Ultrasound frequency /KHz");
  while (!SerialBT.available())
    ;
  Ultrasound_freq = SerialBT.readString().toInt();

  ledcAttachChannel(in1, 2000, 8, 1);
  ledcAttachChannel(in2, 2000, 8, 2);
  ledcAttachChannel(in3, 2000, 8, 3);
  ledcAttachChannel(in4, 2000, 8, 4);
  ledcAttachChannel(Pump, 2000, 8, 5);
  ledcAttachChannel(Heat, 2000, 8, 6);
  ledcAttachChannel(Ultrasound, Ultrasound_freq * 1000, 8, 7);
  ledcAttachChannel(Seeding_pin, 50, 10, 8);
  ledcAttachChannel(Front_left_pin, 50, 10, 9);
  ledcAttachChannel(Front_right_pin, 50, 10, 10);

  Seeding.attach(Seeding_pin);
  Front_left.attach(Front_left_pin);
  Front_right.attach(Front_right_pin);

  Seeding.write(Seeding_pos);

  delay(1000);

  Front_left.write(90);
  Front_right.write(90);

  delay(2000);

  Front_left.write(90 - 70);
  Front_right.write(90 + 70);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(Pump, OUTPUT);
  pinMode(Heat, OUTPUT);
  pinMode(Ultrasound, OUTPUT);

  pinMode(23, INPUT);
  attachInterrupt(23, time_stamp, RISING);

  Target = 10 - 1;  //err value
  Target_turn = 6 - 1;

  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  int yaw_offset = 0;
  for (int i = 0; i < 100; i++) {
    updateYaw();
    yaw_offset += yaw;
  }
  yaw_offset /= 100;
  yaw -= yaw_offset;
}

void loop() {
  for (int n = 0; n < Iterations; n++) {
    analogWrite(Pump, 100);
    Traverse_Y(in_Distance_Y, 0);

    Front_left.write(90);
    Front_right.write(90);
    analogWrite(Pump, 0);

    clockwise_turn(90);
    Traverse_X(in_Distance_X / (2 * Iterations), 90);
    clockwise_turn(180);

    Front_left.write(90 - 70);
    Front_right.write(90 + 70);

    analogWrite(Pump, 100);
    Traverse_Y(in_Distance_Y, 180);

    Front_left.write(90);
    Front_right.write(90);
    analogWrite(Pump, 0);

    anticlockwise_turn(90);
    Traverse_X(in_Distance_X / (2 * Iterations), 90);
    anticlockwise_turn(0);

    Front_left.write(90 - 70);
    Front_right.write(90 + 70);
  }

  Front_left.write(90);
  Front_right.write(90);

  while (1) {
    turn(0);
    Serial.print("done");
  }
}

void Traverse_Y(int Distance, int Maintain) {
  while (Distance_Y < Distance) {
    if (elapsed_time) RPM = ((60000000 / ((double)elapsed_time * 224 * 2)));
    Serial.print(RPM);
    Serial.print("   ");
    Serial.print(yaw);
    Serial.println();

    if (d_prev_time) d_elapsed_time = micros() - d_prev_time;
    d_prev_time = micros();

    seeding_mechanism(175, 15);

    double speed = ((RPM + 1) * 2 * 5 * M_PI) / 60;
    Distance_Y += (d_elapsed_time * speed) / 1000000;

    double err = Target - RPM;
    PWM += 0.02 * err;

    tracking = 10 * (yaw - Maintain);

    forward(PWM + base_pwm, tracking);
    updateYaw();
  }
  turn(0);
  Distance_Y = 0;
  Distance_X = 0;
  elapsed_time = 0;
  d_elapsed_time = 0;
  d_prev_time = 0;
  s_elapsed_time = 0;
  PWM = 0;
  turn_PWM = 0;
  tracking = 0;
}

void Traverse_X(int Distance, int Maintain) {
  while (Distance_Y < Distance) {
    if (elapsed_time) RPM = ((60000000 / ((double)elapsed_time * 224 * 2)));
    Serial.print(RPM);
    Serial.print("   ");
    Serial.print(tracking);
    Serial.println();

    if (d_prev_time) d_elapsed_time = micros() - d_prev_time;
    d_prev_time = micros();

    double speed = ((RPM + 1) * 2 * 5 * M_PI) / 60;
    Distance_Y += (d_elapsed_time * speed) / 1000000;

    double err = Target - RPM;
    PWM += 0.02 * err;

    tracking = 10 * (yaw - Maintain);

    forward(PWM + base_pwm, tracking);
    updateYaw();
  }
  turn(0);
  Distance_Y = 0;
  Distance_X = 0;
  elapsed_time = 0;
  d_elapsed_time = 0;
  d_prev_time = 0;
  s_elapsed_time = 0;
  PWM = 0;
  turn_PWM = 0;
  tracking = 0;
}

void clockwise_turn(int target_angle) {
  while (angle_err > 0) {
    if (elapsed_time) RPM = ((60000000 / ((double)elapsed_time * 224 * 2)));
    angle_err = (target_angle - yaw);

    double turn_err = Target_turn - RPM;
    turn_PWM += 0.02 * turn_err;

    turn(base_pwm + turn_PWM);
    updateYaw();
  }
  Distance_Y = 0;
  Distance_X = 0;
  elapsed_time = 0;
  d_elapsed_time = 0;
  d_prev_time = 0;
  s_elapsed_time = 0;
  PWM = 0;
  turn_PWM = 0;
  tracking = 0;
  angle_err = 1;
}

void anticlockwise_turn(int target_angle) {
  while (angle_err > 0) {
    if (elapsed_time) RPM = ((60000000 / ((double)elapsed_time * 224 * 2)));
    angle_err = -1 * (target_angle - yaw);

    double turn_err = Target_turn - RPM;
    turn_PWM -= 0.02 * turn_err;

    turn(turn_PWM - base_pwm);
    updateYaw();
  }
  Distance_Y = 0;
  Distance_X = 0;
  elapsed_time = 0;
  d_elapsed_time = 0;
  d_prev_time = 0;
  s_elapsed_time = 0;
  PWM = 0;
  turn_PWM = 0;
  tracking = 0;
  angle_err = 1;
}

void seeding_mechanism(int angle, int time) {
  s_elapsed_time += d_elapsed_time;

  if (s_elapsed_time >= time * 1000) {
    s_elapsed_time = 0;

    if (Seeding_pos == angle) {
      state = 1;
    }

    if (Seeding_pos == 0) {
      state = 0;
    }

    if (state == 0) Seeding_pos++;
    else Seeding_pos--;

    Seeding.write(Seeding_pos);
  }
}

void updateYaw() {
  sVector_t mag = compass.readRaw();
  static float hi_cal[3];

  float mag_data[] = { mag.XAxis / 10,
                       mag.YAxis / 10,
                       mag.ZAxis / 10 };

  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }

  heading = atan2(mag_data[1], mag_data[0]) * 180 / M_PI;

  if (prev_heading) {
    if (prev_heading > 90 && prev_heading <= 180 && heading < -90 && heading >= -180) {
      yaw += (180 - prev_heading) + (180 + heading);
    } else if (heading > 90 && heading <= 180 && prev_heading < -90 && prev_heading >= -180) {
      yaw -= (180 + prev_heading) + (180 - heading);
    } else yaw += heading - prev_heading;
  }

  prev_heading = heading;
}

void forward(int speed, int addon_val) {
  analogWrite(in1, speed);
  analogWrite(in2, 0);
  analogWrite(in3, speed + addon_val);
  analogWrite(in4, 0);
}

void turn(int speed) {
  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, speed);
  }
  if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, abs(speed));
    analogWrite(in3, abs(speed));
    analogWrite(in4, 0);
  }
  if (speed == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
  }
}
