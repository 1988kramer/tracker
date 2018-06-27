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

/* - - - - - - - - State Control Structure - - - - - - - - */

struct stateNode
{
  char disp_options[][];
  uint8_t num_options;
  uint8_t num_lines;
  stateNode *next_states;
  void (*function)(int8_t);
};

stateNode cur_state_;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Testing user interface with orientation filter");
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
  updateDisplay();
}

// starts communication with IMU and magnetometer
// and initializes the madgwick filter
void initIMU()
{
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

void updateDisplay()
{
  // find which option should be highlighted
  int8_t highlighted = encoder_pos_ % cur_state_.num_options;
  if (highlighted < 0)
    highlighted += cur_state_.num_options;

  cur_state_.function(highlighted);

  // advance state if switch is pressed
  if (switch_pressed_)
  {
    switch_pressed_ = false;
    disp_state_change_ = true;
    cur_state_ = cur_state_.next_states[highlighted];
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

}

void orientFilter(int8_t highlighted)
{

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
  orient_node.function = &selectFromList;

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
  dead_end.function = &displayText;

  auto_orient.num_options = 1;
  auto_orient.function = &autoOrient;
  auto_orient.next_states = &dead_end;

  char manual_orient_msg[][] = {{"align the hinge axis"},
                                {"with the pole star"},
                                {"and press OK"}};
  manual_orient.options = manual_orient_msg;
  manual_orient.num_options = 1;
  manual_orient.num_lines = 3;
  manual_orient.function = &displayText;
  manual_orient.next_states = &dead_end;

  char orient_help_msg[][] = {{"help message stub"}};
  orient_help.options = orient_help_msg;
  orient_help.num_options = 1;
  orient_help.num_lines = 1;
  orient_help.function = &orientFilter;
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

