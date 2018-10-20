
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

#include<Wire.h>

SoftwareSerial mySerial(9, 10); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

const int8_t SPEAKER_PIN   = 11;
const int8_t LED_PIN_START = 2;
const int8_t LED_PIN_END   = 4;
const int16_t BENDER_PHRASE_MAXLEN = 5000; // ms

// next/volume down pin (IO1) on MPU.  Don't pull this pin
// to the GND for the longer than ~0.4s because it causes the MPU
// to lower the volume.
const int8_t MPU_NEXT = 6;

void configure_leds() {
  for (int pin = LED_PIN_START; pin <= LED_PIN_END; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode (SPEAKER_PIN, OUTPUT);

  configure_leds();

  mySerial.begin(9600);
  if (!myDFPlayer.begin(mySerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true) {
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  myDFPlayer.volume(22);
  pinMode (MPU_NEXT, OUTPUT);
  digitalWrite(MPU_NEXT, HIGH);

  randomSeed(analogRead(A0));
}

long ac_x_prev = 0;
long ac_y_prev = 0;
long AC_BORDER = 3000;

void play_tone(int pin, float f, long len) {
  long p = 1000000 / f;
  long d = p / 2;
  int n = len / p;
  for (int i = 0; i < n; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(d);
    digitalWrite(pin, LOW);
    delayMicroseconds(d);

  }
}

void pwm(int pin, float duty, long len) {
  const int P = 1000;
  int d1 = P * duty;
  int d2 = P - d1;
  int count = len / P;
  for (int c = 0; c < count; c++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(d1);
    digitalWrite(pin, LOW);
    delayMicroseconds(d2);
  }
}

void blink_eyes_slowly() {
  for (int c = 0; c < 4; c++) {
    for (float d = 0.01; d < 1.0; d += 0.01) {
      pwm(2, d, 5000);
    }
    for (float d = 0.99; d > 0; d -= 0.01) {
      pwm(2, d, 5000);
    }
  }
}

void speak() {
  Serial.print("Speaking... ");
  int n = random(1, 7);
  Serial.println(n);
  myDFPlayer.play(n);
  blink_eyes_slowly();
}

boolean is_time_to_speak_come() {
  return (millis() % 100 == 0) && (random(0, 4) == 0);
}

boolean is_shaken() {
  return (abs(GyX) > AC_BORDER) || (abs(GyY) > AC_BORDER);
}

void generate_sounds(int16_t input_value) {
  play_tone(SPEAKER_PIN, map(abs(input_value), 0, 32768, 100, 1200), 10000);
}

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  if (is_shaken()) {
    int a = random(LED_PIN_START, LED_PIN_END + 1);
    digitalWrite(a, ! digitalRead(a));
    generate_sounds(GyX);
    if (is_time_to_speak_come()) {
      speak();
    }
  }
}
