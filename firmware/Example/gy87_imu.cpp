#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMP085.h"
#include "HMC5883L.h"
#include "Wire.h"

MPU6050 accelgyro;
BMP085 barometer;
HMC5883L mag;

int16_t mx, my, mz;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float temperature;
float pressure;
int32_t altitude;

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    barometer.initialize();
    mag.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
}

void loop()
{
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    temperature = barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    pressure = barometer.getPressure();
    altitude = barometer.getAltitude(pressure);

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // mag.getHeading(&mx, &my, &mz);

    float heading = atan2(my, mx);
    if (heading < 0)
        heading += 2 * M_PI;
    // these methods (and a few others) are also available
    // accelgyro.getAcceleration(&ax, &ay, &az);
    // accelgyro.getRotation(&gx, &gy, &gz);
    Serial.print("a/g/T/P/A/mag/heading:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\t");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print(pressure);
    Serial.print("\t");
    Serial.print(altitude);
    Serial.print("\t");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t");
    Serial.println(heading * 180 / M_PI);

    delay(100);
}
