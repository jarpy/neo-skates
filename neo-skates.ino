#include "Wire.h"
#include <FastLED.h>

#define FORCE_SCALING 200
#define FORCE_RESISTANCE 2000
#define FORCE_THRESHOLD 5000
#define COOLDOWN_TIME 50

#define LED_COUNT 8
#define MAX_BRIGHT 255
#define NEOPIXEL_PIN 7
#define MEMORY_DEPTH 1

const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

CRGB leds[LED_COUNT];

int red = 0;
int green = 0;
int blue = 0;
int loopCounter = 0;

int hue = 0;
int saturation = 0;
int luminance = 0;

float rho;
float theta;
byte theta8bit;
byte thetas[MEMORY_DEPTH];
byte cooldown = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, LED_COUNT);
}

void showHSV(CHSV color) {
    fill_solid( &(leds[0]), LED_COUNT, color);
    FastLED.show();
}

void debugFloat(char* label, float value) {
  if(loopCounter == 0) {
    Serial.print(label);
    Serial.print(":");
    Serial.print(value);
    Serial.print("\t");
  }
}

void debugByte(char* label, byte value) {
  if(loopCounter == 0) {
    Serial.print(label);
    Serial.print(":");
    Serial.print(value);
    Serial.print("\t");
  }
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

  //if (abs(xAccelScaled) > 30 && blue < MAX_BRIGHT) {
  //  blue += abs(xAccelScaled) / 20;
  //}

  // blue += xAccelScaled;
  // red  -= xAccelScaled;
  //Serial.print(convert_int16_to_str(blue));
  //Serial.print(convert_int16_to_str(xAccelScaled));
  //Serial.print(convert_int16_to_str(yAccelScaled));
  //Serial.print(convert_int16_to_str(zAccelScaled));

  //Serial.print(accelX);
  //Serial.print("\t");
  //Serial.print(accelY);
  //Serial.print("\t");

  rho = sqrt(pow(accelX, 2) + pow(accelY, 2));
  //debugFloat("rho", rho);

  //theta = atan(float(accelY) / float(accelX));
  theta = atan2(float(accelY), float(accelX));
  theta *= 57.2957795;
  debugFloat("theta_raw", theta);
  if (theta < 0) {theta += 360;}

  // if(loopCounter == 0) {
  //   Serial.print("x:");
  //   Serial.print(accelX);
  //   Serial.print("\t");
  //   Serial.print("y:");
  //   Serial.print(accelY);
  //   Serial.print("\t");
  //   Serial.print("thetaQ:");
  //   Serial.print(theta);
  //   Serial.print("\t");
  // }

  theta /= 1.40625;
  // Serial.print("rho:");
  // Serial.print(rho);
  // Serial.print("\t");
  // Serial.print(theta);

  //if (abs(yAccelScaled) > 30 && red < MAX_BRIGHT) {
  //  red += abs(yAccelScaled) / 20;
  //}

  //if (abs(zAccelScaled) > 30 && green < MAX_BRIGHT) {
  //  green += abs(zAccelScaled) / 20;
  //}

  thetas[loopCounter] = theta;
  loopCounter++;
  if (loopCounter == MEMORY_DEPTH) {
    loopCounter = 0;
  }

  if (rho > FORCE_THRESHOLD) {
    luminance += rho / FORCE_RESISTANCE;

    if (cooldown == 0) {
      hue = theta;
      cooldown = COOLDOWN_TIME;
    }
  }

  // Normalize
  luminance = min(luminance, 255);
  luminance = max(luminance - 2, 0);
  if (cooldown > 0) {cooldown -= 1;}
  debugByte("cooldown", cooldown);

  // Take the average angle as the color hue;
  //hue = 0;
  //for(int i=0; i<MEMORY_DEPTH; i++) {
  //  hue += thetas[i];
  //}
  //hue = hue / MEMORY_DEPTH;

  showHSV(CHSV(hue, 255, max(luminance, 1)));

  if(loopCounter == 0) {
  FastLED.show();
  Serial.print("hue:");
  Serial.print(hue);
  Serial.print("\t");
  Serial.print("luminance:");
  Serial.print(luminance);
  Serial.print("\t");
  Serial.println("");
  }
}
