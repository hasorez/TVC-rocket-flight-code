#include <Wire.h>
#include <Adafruit_BMP280.h>  // HW611 is usually BMP280

bool initialized = false;
float groundPressure = 1013.25;  // hPa (default sea level)
float groundAltitude = 0.0;

Adafruit_BMP280 bmp;  // I2C instance

bool initBaro() {
  if (!bmp.begin(0x77)) {  // Most HW611 boards use 0x76 (sometimes 0x77)
    return false;
  }

  // Set up oversampling & filter for rockets (fast response, less noise)
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // temperature oversampling
    Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
    Adafruit_BMP280::FILTER_X4,     // IIR filter
    Adafruit_BMP280::STANDBY_MS_1   // fastest update rate
  );

  initialized = true;
  return true;
}

void calibrateGround() {
  if (!initialized) return;
  groundPressure = bmp.readPressure() / 100.0f;  // Pa → hPa
  groundAltitude = bmp.readAltitude(groundPressure);
}

float getPressure() {
  if (!initialized) return 0;
  return bmp.readPressure() / 100.0f;  // hPa
}

float getTemperature() {
  if (!initialized) return 0;
  return bmp.readTemperature();  // °C
}

float getAltitude() {
  if (!initialized) return 0;
  // Altitude relative to calibration
  float alt = bmp.readAltitude(groundPressure);
  return alt - groundAltitude;
}
