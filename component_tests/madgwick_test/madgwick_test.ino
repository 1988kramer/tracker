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
unsigned long start_time;
const unsigned long warm_up_time = 10000;
const float accel_scale = 0.000061;
const float gyro_scale = 0.00875; 
const float mag_scale = 6482.0;

float roll, pitch, yaw;

const float x_offset = 1809.5;
const float x_scale = 0.97; // -1696, 5315, 7011
const float y_offset = -2592;
const float y_scale = 1.014; // -5945, 761, 6706
const float z_offset = 6747;
const float z_scale = 1.017; // 3401, 10093, 6692
// 6803

int zero_rate_err[] = {265, -837, -360};
int zero_gee_err[] = {-885, -390, -284};

long sum_gyro[] = {0,0,0};
long sum_acc[] = {0,0,0};
int meas_count = 0;

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
  start_time = millis();
}

void loop() {
  
  if (micros() - micros_previous >= micros_per_reading)
  {
    float a_x, a_y, a_z;
    float g_x, g_y, g_z;
    float m_x, m_y, m_z;
    
    mag.read();
    imu.read();
    /*
    if (millis() - start_time > warm_up_time)
    {
      int num_meas = 100;
      if (meas_count++ < num_meas)
      {
        sum_gyro[0] += imu.g.x;
        sum_gyro[1] += imu.g.y;
        sum_gyro[2] += imu.g.z;
        sum_acc[0] += imu.a.x;
        sum_acc[1] += imu.a.y;
        sum_acc[2] += imu.a.z;
      }
      else
      {
        zero_rate_err[0] = sum_gyro[0] / num_meas;
        zero_rate_err[1] = sum_gyro[1] / num_meas;
        zero_rate_err[2] = sum_gyro[2] / num_meas;
        zero_gee_err[0] = sum_acc[0] / num_meas;
        zero_gee_err[1] = sum_acc[1] / num_meas;
        zero_gee_err[2] = sum_acc[2] / num_meas;
      }
    }
    */
    convertRawAccel(a_x,a_y,a_z);
    convertRawGyro(g_x,g_y,g_z);
    convertRawMag(m_x,m_y,m_z);

    filter.update(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    micros_previous += micros_per_reading;
    
    if (millis() - last_pub_time >= millis_per_pub)
    {
      Serial.print("roll: ");
      Serial.println(roll);
      Serial.print("pitch: ");
      Serial.println(pitch);
      Serial.print("yaw: ");
      Serial.println(yaw);
      
      Serial.print("acc readings: ");
      Serial.print(a_x);
      Serial.print(", ");
      Serial.print(a_y);
      Serial.print(", ");
      Serial.println(a_z);

      Serial.print("gyro readings: ");
      Serial.print(g_x);
      Serial.print(", ");
      Serial.print(g_y);
      Serial.print(", ");
      Serial.println(g_z);
      /*
      Serial.print("zero rate errors: ");
        for (int i = 0; i < 3; i++)
        {
          Serial.print(zero_rate_err[i]);
          Serial.print(", ");
        }
        Serial.println();
        Serial.print("zero gee errors: ");
        for (int i = 0; i < 3; i++)
        {
          Serial.print(zero_gee_err[i]);
          Serial.print(", ");
        }
        Serial.println();
        */
      last_pub_time += millis_per_pub;
    }
  }
}

void convertRawAccel(float &x, float &y, float &z)
{
  x = ((float)imu.a.x - (float)zero_gee_err[0]) * accel_scale;
  y = ((float)imu.a.y - (float)zero_gee_err[1]) * accel_scale;
  z = ((float)imu.a.z - (float)zero_gee_err[2]) * accel_scale;
}

void convertRawGyro(float &x, float &y, float &z)
{
  x = ((float)imu.g.x - (float)zero_rate_err[0]) * gyro_scale;
  y = ((float)imu.g.y - (float)zero_rate_err[1]) * gyro_scale;
  z = ((float)imu.g.z - (float)zero_rate_err[2]) * gyro_scale;
}

void convertRawMag(float &x, float &y, float &z)
{
  float x_raw = ((float)mag.m.x - x_offset) * x_scale;
  float y_raw = ((float)mag.m.y - y_offset) * y_scale;
  float z_raw = ((float)mag.m.z - z_offset) * z_scale;
  x = x_raw / mag_scale;
  y = y_raw / mag_scale;
  z = z_raw / mag_scale;
}

