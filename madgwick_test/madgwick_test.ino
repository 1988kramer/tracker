#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include "MadgwickAHRS.h"

LIS3MDL mag;
LSM6 imu;
Madgwick filter;

#define UPDATE_HZ 100
#define PUB_HZ 1

unsigned long micros_per_reading, micros_previous;
unsigned long last_pub_time, millis_per_pub;
const float accel_scale = 0.000061;
const float gyro_scale = 0.00875; 
const float mag_scale = 6482;

float roll, pitch, yaw;

void setup() {
  Serial.begin(9600);

  if (!imu.init())
  {
    Serial.println("failed to initialize imu");
  }
  imu.enableDefault();
  if (!mag.init())
  {
    Serial.println("failed to initialize magnetometer");
  }
  mag.enableDefault();

  filter.begin(UPDATE_HZ);
  micros_per_reading = 1000000 / UPDATE_HZ;
  millis_per_pub = 1000 / PUB_HZ;
  micros_previous = micros();
  last_pub_time = millis();
}

void loop() {

  if (micros() - micros_previous >= micros_per_reading)
  {
    float a_x, a_y, a_z;
    float g_x, g_y, g_z;
    float m_x, m_y, m_z;
    
    mag.read();
    imu.read();
    
    convertRawAccel(a_x,a_y,a_z);
    convertRawGyro(g_x,g_y,g_z);
    convertRawMag(m_x,m_y,m_z);

    filter.update(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    micros_previous += micros_per_reading;
  }
  if (millis() - last_pub_time >= millis_per_pub)
  {
    Serial.print("roll: ");
    Serial.println(roll);
    Serial.print("pitch: ");
    Serial.println(pitch);
    Serial.print("yaw: ");
    Serial.println(yaw);

    last_pub_time += millis_per_pub;
  }
}

void convertRawAccel(float &x, float &y, float &z)
{
  x = (float)imu.a.x * accel_scale;
  y = (float)imu.a.y * accel_scale;
  z = (float)imu.a.z * accel_scale;
}

void convertRawGyro(float &x, float &y, float &z)
{
  x = (float)imu.g.x * gyro_scale;
  y = (float)imu.g.y * gyro_scale;
  z = (float)imu.g.z * gyro_scale;
}

void convertRawMag(float &x, float &y, float &z)
{
  x = (float)mag.m.x / mag_scale;
  y = (float)mag.m.y / mag_scale;
  z = (float)mag.m.z / mag_scale;
}

