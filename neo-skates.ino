#include "Wire.h"
#include <Adafruit_NeoPixel.h>

#define PHYSICAL_LEDS 8
#define VIRTUAL_LEDS 8
#define MAX_BRIGHT 255


const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PHYSICAL_LEDS, 7, NEO_GRB + NEO_KHZ800);

int red = 0;
int green = 0;
int blue = 0;
int loopCounter = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  strip.begin();
  strip.show();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  //gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  //gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  //gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  int xAccelScaled = accelerometer_x / 100;
  int yAccelScaled = accelerometer_y / 100;
  int zAccelScaled = (accelerometer_z / 100) + 170;

  if (abs(xAccelScaled) > 30 && blue < MAX_BRIGHT) {
    blue += abs(xAccelScaled) / 20;
  }

  if (abs(yAccelScaled) > 30 && red < MAX_BRIGHT) {
    red += abs(yAccelScaled) / 20;
  }

  if (abs(zAccelScaled) > 30 && green < MAX_BRIGHT) {
    green += abs(zAccelScaled) / 20;
  }

  // Decay
  if (loopCounter == 0) {
    red = max(1, red - (0.05 * red));
    green = max(1, green - (0.05 * green));
    blue = max(1, blue - (0.05 * blue));
  }
  loopCounter++;
  if (loopCounter == 2) {
    loopCounter = 0;
  }
  
  // Display
  for (uint16_t i = 0; i < VIRTUAL_LEDS; i++) {
    //for (uint16_t i = 0; i < 1; i++) {
    strip.setPixelColor(i, strip.Color(
                          min(red, MAX_BRIGHT),
                          min(green, MAX_BRIGHT),
                          min(blue, MAX_BRIGHT)
                        ));
    strip.show();
  }
}
