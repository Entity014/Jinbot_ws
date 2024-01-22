#include <Arduino.h>
#include <MPU9250.h>

MPU9250 gyroscope_;
MPU9250 accelerometer_;
// float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    gyroscope_.initialize();
    accelerometer_.initialize();
}

void loop()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    gyroscope_.getRotation(&gx, &gy, &gz);
    accelerometer_.getAcceleration(&ax, &ay, &az);
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
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.println("\t");
}
