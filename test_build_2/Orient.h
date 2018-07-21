#include <LIS3MDL.h>
#include <LSM6.h>
#include "MadgwickAHRS.h"

/* - - - - - - - - - - Madgwick Filter Constants - - - - - - - - - */

#define FILTER_UPDATE_HZ   100
#define FILTER_PUB_HZ      1


/* - - - - - - - - - - Madgwick Filter Variables - - - - - - - - - */

unsigned long filter_update_time_;
unsigned long last_filter_update_;
unsigned long last_filter_pub_;
unsigned long filter_pub_time_;
const float accel_scale_ = 0.000061;
const float gyro_scale_ = 0.00875;
const float mag_scale_ = 6482;
float roll_, pitch_, yaw_;
LIS3MDL mag_;
LSM6 imu_;
Madgwick filter_;


/*- - - - - - - - - - Madgwick Filter Functions - - - - - - - - - */

void initOrientNode(stateNode *auto_orient)
{
  auto_orient->num_options = 1;
  auto_orient->num_lines = 3;
  strcpy(auto_orient->disp_options[0], "press OK when done");
  auto_orient->init_func = initIMU;
  auto_orient->action = orientFilter;
  auto_orient->end_func = NULL;
  auto_orient->switch_active = true;
}

// starts communication with IMU and magnetometer
// and initializes the madgwick filter
void initIMU()
{
  DEBUG_PORT.println("initializing IMU");
  if (!imu_.init())
  {
    DEBUG_PORT.println("failed to initialize IMU");
    DEBUG_PORT.flush();
  }
  imu_.enableDefault();
  DEBUG_PORT.println("initializing magnetometer");
  if (!mag_.init())
  {
    DEBUG_PORT.println("failed to initialize magnetometer");
    DEBUG_PORT.flush();
  }
  mag_.enableDefault();

  DEBUG_PORT.println("starting madgwick filter");
  filter_.begin(FILTER_UPDATE_HZ);
  filter_update_time_ = 1000000/ FILTER_UPDATE_HZ;
  filter_pub_time_ = 1000 / FILTER_PUB_HZ;
  last_filter_update_ = micros();
  last_filter_pub_ = millis();
  DEBUG_PORT.println("IMU initialized!");
}

// updates the Madgwick filter and publishes orientation to 
// the OLED screen
void orientFilter()
{
  // Need to add functions to initialize IMU when needed
  // and sleep the imu when not needed
  if (micros() - last_filter_update_ >= filter_update_time_)
  {
    updateMadgwick();
  }
  if (millis() - last_filter_pub_ >= filter_pub_time_)
  {
  	disp_state_change_ = true;
    display_.clearDisplay();
    display_.setCursor(0,0);
    printOrientation();
    printText();
    display_.display();
  }
}

// updates the Madgwick filter
void updateMadgwick()
{
  float a_x, a_y, a_z;
  float g_x, g_y, g_z;
  float m_x, m_y, m_z;
   
  mag_.read();
  imu_.read();
    
  convertRawAccel(a_x,a_y,a_z);
  convertRawGyro(g_x,g_y,g_z);
  convertRawMag(m_x,m_y,m_z);
  
  filter_.update(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);

  roll_ = filter_.getRoll();
  pitch_ = filter_.getPitch();
  yaw_ = filter_.getYaw();

  last_filter_update_ += filter_update_time_;
}

// prints the device's current and target orientation 
// to the OLED screen
void printOrientation()
{
  display_.print("roll: ");
  display_.println(roll_);
  display_.println("target: 0.0");

  int8_t target_pitch = -1 * latitude_;
  display_.print("pitch: ");
  display_.println(pitch_);
  display_.print("target: ");
  display_.println(target_pitch);
  
  display_.print("yaw: ");
  display_.println(yaw_);
  display_.print("target: ");
  display_.println(declination_);
  display_.println();

  last_filter_pub_ += filter_pub_time_;
}

// converts raw accelerometer output to g's
void convertRawAccel(float &x, float &y, float &z)
{
  x = (float)imu_.a.x * accel_scale_;
  y = (float)imu_.a.y * accel_scale_;
  z = (float)imu_.a.z * accel_scale_;
}

// converts raw gyro output to deg/sec
void convertRawGyro(float &x, float &y, float &z)
{
  x = (float)imu_.g.x * gyro_scale_;
  y = (float)imu_.g.y * gyro_scale_;
  z = (float)imu_.g.z * gyro_scale_;
}

// converts raw magnetometer output to gauss
void convertRawMag(float &x, float &y, float &z)
{
  x = (float)mag_.m.x / mag_scale_;
  y = (float)mag_.m.y / mag_scale_;
  z = (float)mag_.m.z / mag_scale_;
}