#include "Wire.h"
#include <FastLED.h>

#define FORCE_SCALING 200
#define FORCE_RESISTANCE 1000
#define FORCE_THRESHOLD 5000
#define STOMP_THRESHOLD 60
#define COOLDOWN_TIME 150
#define DECAY_RATE 2
#define HUE_ROTATION 90
#define MAX_LUMINANCE 300
#define LUMINANCE_SLEW_LIMIT 10 // How quickly can we get brighter?

#define LED_COUNT 8
#define NEOPIXEL_PIN 7
#define MEMORY_DEPTH 1

const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data

int xAccelScaled;
int yAccelScaled;
int zAccelScaled;
float rho;
float theta;
int hue = 0;
int luminance = 0;
byte cooldown = 0;
CRGB leds[LED_COUNT];

void showHSV(CHSV color) {
    fill_solid( &(leds[0]), LED_COUNT, color);
    FastLED.show();
}

void stomp() {
  for (int i=255; i>0; i -= 2) {
    showHSV(CHSV(200, 200, i)); // Bright pink
  }
}

void debugFloat(char* label, float value) {
  Serial.print(label);
  Serial.print(":");
  Serial.print(value);
  Serial.print("\t");
}

void debugByte(char* label, byte value) {
  Serial.print(label);
  Serial.print(":");
  Serial.print(value);
  Serial.print("\t");
}

void debugInt(char* label, int value) {
  Serial.print(label);
  Serial.print(":");
  Serial.print(value);
  Serial.print("\t");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, LED_COUNT);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  accelX = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelY = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelZ = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  //gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  //gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  //gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  int xAccelScaled = accelX / FORCE_SCALING;
  int yAccelScaled = accelY / FORCE_SCALING;
  int zAccelScaled = ((accelZ + 16000) / FORCE_SCALING);

  rho = sqrt(pow(accelX, 2) + pow(accelY, 2));
  //debugFloat("rho", rho);

  theta = atan2(float(accelY), float(accelX)) * 57.2957795;
  if (theta < 0) {theta += 360;}
  theta += HUE_ROTATION;
  if (theta > 360) {
    theta -= 360;
  }

  if (rho > FORCE_THRESHOLD) {
    luminance += max((rho / FORCE_RESISTANCE), LUMINANCE_SLEW_LIMIT);

    if (cooldown == 0) {
      hue = theta / 1.40625; // Degrees (360) to 8-bit (256).
      cooldown = COOLDOWN_TIME; // Restart the colour change cooldown timer.
    }
  }

  // Normalize
  luminance = min(luminance, MAX_LUMINANCE);
  luminance = max(luminance - DECAY_RATE, 0);
  if (cooldown > 0) {cooldown -= 1;}
  //debugByte("cooldown", cooldown);

  // Take the average angle as the color hue;
  //hue = 0;
  //for(int i=0; i<MEMORY_DEPTH; i++) {
  //  hue += thetas[i];
  //}
  //hue = hue / MEMORY_DEPTH;

  showHSV(CHSV(hue, 255, min(max(luminance, 1), 255)));

  // Flash on stomp.
  debugInt("stompyness", zAccelScaled);
  if (zAccelScaled < -STOMP_THRESHOLD) {
    stomp();
  }

  Serial.println("");
}
