#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include "MadgwickAHRS.h"

// encoder_pins
#define ENCODER_A       10
#define ENCODER_B       11
#define ENCODER_SWITCH  12

// OLED pins
#define OLED_CLK        27
#define OLED_MOSI       26
#define OLED_CS         25
#define OLED_DC         28
#define OLED_RESET      24

// IMU pins
#define IMU_POWER      18 // need to actually connect this one
                          // IMU only draws 5mA so this should be safe

// Madgwick filter constants
#define FILTER_UPDATE_HZ   100
#define FILTER_PUB_HZ      1


/* - - - - - - - - - encoder variables - - - - - - - - - - */

int encoder_pos_;
int last_encoder_pos_;
uint8_t last_a_state_;
uint8_t last_b_state_;
unsigned long last_encoder_time_;
unsigned long last_switch_time_;
const uint8_t enc_bounce_time_ = 10;
const uint8_t switch_bounce_time_ = 200;
bool switch_pressed_;


/* - - - - - - - - OLED display variables - - - - - - - - - - */

Adafruit_SSD1305 display_(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
bool disp_state_change_;


/* - - - - - - - Madgwick Filter Variables - - - - - - - - - */

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


/* - - - - - - - - State Control Structure - - - - - - - - */

/* The program's state and behavior are managed through
 * a graph structure. Each node in the graph holds state
 * information, pointers to possible next states, and
 * a function pointer defining the behavior of that node.
 */

struct stateNode
{
  sateNode() : init(NULL), end(NULL) {}
  char disp_options[][];
  uint8_t num_options;
  uint8_t num_lines;
  stateNode *next_states;
  void (*action)(int8_t);
  void (*init)(void);
  void (*end)(void);
};

stateNode cur_state_;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Testing user interface with orientation filter");
  digitalWrite(IMU_POWER, LOW); // start with IMU off 
  initEncoder();
  initIMU();
  initDisplay();
  cur_state_ = initStates();
  delay(500);
}

void loop() 
{
  // need to add filter update step
  unsigned long cur_time = millis();
  checkEncoderState(cur_time);
  checkSwitchState(cur_time);
  updateDeviceState();
}

// starts communication with IMU and magnetometer
// and initializes the madgwick filter
void initIMU()
{
  digitalWrite(IMU_POWER, HIGH);
  // may need to add delay here if IMU doesn't initialize properly
  if (!imu.init())
    Serial.println("failed to initialize IMU");
  imu.enableDefault();

  if (!mag.init())
    Serial.println("failed to initialize magnetometer");
  mag.enableDefault();

  filter.begin(FILTER_UPDATE_HZ);
  filter_update_time_ = 1000000/ FILTER_UPDATE_HZ;
  filter_pub_time_ = 1000 / FILTER_PUB_HZ;
  last_filter_update_ = micros();
  last_filter_pub_ = millis();
}

// put IMU into low-power state and stop madgwick filter
void stopIMU()
{
  digitalWrite(IMU_POWER, LOW); // power off imu
}

// calls the action function for the device's current state and
// updates the device's state if necessary
void updateDeviceState()
{
  // find which option should be highlighted
  int8_t highlighted = encoder_pos_ % cur_state_.num_options;
  if (highlighted < 0)
    highlighted += cur_state_.num_options;

  cur_state_.action(highlighted);

  // advance state if switch is pressed
  if (switch_pressed_)
  {
    switch_pressed_ = false;
    disp_state_change_ = true;
    if (cur_state_.end != NULL)
      cur_state_.end();
    cur_state_ = cur_state_.next_states[highlighted];
    if (cur_state_.init != NULL)
      cur_state_.init();
    encoder_pos_ = 0;
  }
}

void selectFromList(int8_t highlighted)
{
  // update the display only if the display state has changed
  if (encoder_pos_ != last_encoder_pos_ || disp_state_change_)
  {
    if (disp_state_change_)
      disp_state_change_ = false;
    last_encoder_pos_ = encoder_pos_;
    display_.clearDisplay();
    display_.setCursor(0,0);
    Serial.print("highlighted option: ");
    Serial.println(highlighted);
    for (uint8_t i = 0; i < cur_state_.num_options; i++)
    {
      if (i == highlighted)
        display_.setTextColor(BLACK,WHITE);
      else
        display_.setTextColor(WHITE);
      display_.println(cur_state_.options[i]);
    }
    display_.display();
  }
}

void checkEncoderState(unsigned long cur_time)
{
  if (cur_time - last_encoder_time_ > enc_bounce_time_)
  {
    uint8_t enc_a_state = digitalRead(ENCODER_A);
    uint8_t enc_b_state = digitalRead(ENCODER_B);
    if (enc_a_state != last_a_state_)
    {
      encoderAUpdate(enc_a_state, enc_b_state);
      last_encoder_time_ = cur_time;
    }
    else if (enc_b_state != last_b_state_)
    {
      encoderBUpdate(enc_a_state, enc_b_state);
      last_encoder_time_ = cur_time;
    }
  }
}

void checkSwitchState(unsigned long cur_time)
{
  if (cur_time - last_switch_time_ > switch_bounce_time_)
  {
    if (digitalRead(ENCODER_SWITCH) == LOW)
    {
      switch_pressed_ = true;
      last_switch_time_ = cur_time;
      Serial.println("switch press detected");
    }
  }
  
  /*
  else if (switch_pressed_)
  {
    switch_pressed_ = false;
    Serial.println("switch state reset");
  }
  */
}

void displayText(int8_t highlighted)
{
  display_.clearDisplay();
  display_.setCursor(0,0);
  for (int i = 0; i < cur_state_.num_lines; i++)
    display_.println(cur_state_.options[i]);
  display_.display();
}

void orientFilter(int8_t highlighted)
{
  // Need to add functions to initialize IMU when needed
  // and sleep the imu when not needed
  if (micros() - last_filter_update_ >= filter_update_time_)
  {
    updateMadgwick();
  }
  if (millis() - last_filter_pub_ >= filter_pub_time_)
  {
    publishOrientation();
  }
}

stateNode initStates()
{
  stateNode orient_node;
  char orient_options[][] = {{"1) automatic orientation\0"},
                             {"2) manual orientation\0"},
                             {"3) help"}};
  orient_node.options = orient_options;
  orient_node.num_lines = 3;
  orient_node.num_options = 3;
  orient_node.action = &selectFromList;

  stateNode auto_orient;
  stateNode manual_orient;
  stateNode orient_help;
  stateNode mode_next_states[] = {auto_orient, manual_orient, orient_help};

  orient_mode.next_states = &mode_next_states;

  stateNode dead_end;
  char dead_end_text[][] = {{"nothing here"}};
  dead_end.options = dead_end_text; 
  dead_end.num_options = 1;
  dead_end.num_lines = 1;
  dead_end.action = &displayText;

  auto_orient.num_options = 1;
  auto_orient.init = &initIMU;
  auto_orient.action = &orientFilter;
  auto_orient.end = &stopIMU; // needs to be implemented
  auto_orient.next_states = &dead_end;

  char manual_orient_msg[][] = {{"align the hinge axis"},
                                {"with the pole star"},
                                {"and press OK"}};
  manual_orient.options = manual_orient_msg;
  manual_orient.num_options = 1;
  manual_orient.num_lines = 3;
  manual_orient.action = &displayText;
  manual_orient.next_states = &dead_end;

  char orient_help_msg[][] = {{"help message stub"}};
  orient_help.options = orient_help_msg;
  orient_help.num_options = 1;
  orient_help.num_lines = 1;
  orient_help.action = &displayText;
  orient_help.next_states = &orient_node;

  return orient_node;
}

void initDisplay()
{
  display_.begin();
  display_.clearDisplay();
  display_.setCursor(0,0);
  display_.setTextColor(WHITE);
  display_.println("Limited UI test using");
  display_.println("IMU orientation filter");
  display_.display();
  delay(1000);
  disp_state_change_ = true;
}


void initEncoder()
{
  // set encoder pin states
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH, INPUT_PULLUP);

  // initialize encoder variables
  encoder_pos_ = 0;
  last_encoder_pos_ = 0;
  last_a_state_ = digitalRead(ENCODER_A);
  last_b_state_ = digitalRead(ENCODER_B);
  last_encoder_time_ = millis();
  last_switch_time_ = millis();
  switch_pressed_ = false;
}

void encoderAUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_a_state_ = enc_a_state;
  if (enc_a_state == HIGH)
  {
    if (enc_b_state == LOW)
      encoder_pos_++;
    else
      encoder_pos_--;
  }
  else
  {
    if (enc_b_state == HIGH)
      encoder_pos_++;
    else
      encoder_pos_--;
  }
  Serial.print("encoder updated to: ");
  Serial.println(encoder_pos_);
}

void encoderBUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_b_state_ = enc_b_state;
  if (enc_b_state == HIGH)
  {
    if (enc_a_state == HIGH)
      encoder_pos_++;
    else
      encoder_pos_--;
  }
  else
  {
    if (enc_a_state == LOW)
      encoder_pos_++;
    else
      encoder_pos_--;
  }
  Serial.print("encoder updated to: ");
  Serial.println(encoder_pos_);
}

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

  roll_ = filter.getRoll();
  pitch_ = filter.getPitch();
  yaw_ = filter.getYaw();

  last_filter_update_ += filter_update_time_;
}

void publishOrientation()
{
  display_.clearDisplay();
  display_.setCursor(0,0);
  display_.print("roll: ");
  display_.println(roll_);
  display_.print("pitch: ");
  display_.println(pitch_);
  display_.print("yaw: ");
  display_.println(yaw_);
  display_.println();
  display_.println("Press enter to continue.");

  last_filter_pub_ += filter_pub_time_;
}

// converts raw accelerometer output to g's
void convertRawAccel(float &x, float &y, float &z)
{
  x = (float)imu.a.x * accel_scale_;
  y = (float)imu.a.y * accel_scale_;
  z = (float)imu.a.z * accel_scale_;
}

// converts raw gyro output to deg/sec
void convertRawGyro(float &x, float &y, float &z)
{
  x = (float)imu.g.x * gyro_scale_;
  y = (float)imu.g.y * gyro_scale_;
  z = (float)imu.g.z * gyro_scale_;
}

// converts raw magnetometer output to gauss
void convertRawMag(float &x, float &y, float &z)
{
  x = (float)mag.m.x / mag_scale_;
  y = (float)mag.m.y / mag_scale_;
  z = (float)mag.m.z / mag_scale_;
}

