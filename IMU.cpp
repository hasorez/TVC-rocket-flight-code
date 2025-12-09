#include "BMI088.h"
#include <Wire.h>
#include "Orientation.h"
#include "Arduino.h"

// Create the sensor instance using default I2C addresses
BMI088 bmi088(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

Vector3 gyroBias = {0, 0, 0};

bool initIMU() {
    Wire.begin();
    delay(100);

    Serial2.println("Initializing BMI088...");

    if (!bmi088.isConnection()) {
        Serial2.println("BMI088 not detected!");
        return false;
    }

    bmi088.initialize();

    // Accelerometer setup
    bmi088.setAccPoweMode(ACC_ACTIVE);
    bmi088.setAccScaleRange(RANGE_24G);
    bmi088.setAccOutputDataRate(ODR_400);

    // Gyroscope setup
    bmi088.setGyroPoweMode(GYRO_NORMAL);
    bmi088.setGyroScaleRange(RANGE_2000);
    bmi088.setGyroOutputDataRate(ODR_400_BW_47);

    delay(200);

    // === Calibrate gyro ===
    Serial2.println("Calibrating gyro...");
    const int N = 500;
    Vector3 gSum = {0, 0, 0};

    for (int i = 0; i < N; i++) {
        digitalWrite(PB0, (i % 20 < 10) ? HIGH : LOW);
        float gx, gy, gz;
        bmi088.getGyroscope(&gx, &gy, &gz);
        gSum.x += gx;
        gSum.y += gy;
        gSum.z += gz;
        delay(2);
    }

    gyroBias.x = (gSum.x / N) * (PI / 180.0f);
    gyroBias.y = (gSum.y / N) * (PI / 180.0f);
    gyroBias.z = (gSum.z / N) * (PI / 180.0f);

    digitalWrite(PB0, HIGH);
    Serial2.println("BMI088 calibration complete!");
    return true;
}

Vector3 getGyroData() {
    Vector3 gyro;
    float gx, gy, gz;
    bmi088.getGyroscope(&gx, &gy, &gz);

    // convert dps → rad/s and apply bias correction
    gyro.x = -(gx * (PI / 180.0f)- gyroBias.x);
    gyro.y = gy * (PI / 180.0f) - gyroBias.y;
    gyro.z = -(gz * (PI / 180.0f) - gyroBias.z);
    // Serial2.println(gyro.x);
    // Serial2.println(gyro.y);
    // Serial2.println(gyro.z);

    return gyro;
}

Vector3 getAccData() {
    Vector3 accel;
    float ax, ay, az;
    bmi088.getAcceleration(&ax, &ay, &az);

    accel.x = -(ax / 1000);
    accel.y = ay / 1000;
    accel.z = -(az / 1000);

    return accel;
}
