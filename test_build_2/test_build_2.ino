#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include "MadgwickAHRS.h"
#include <NMEAGPS.h>
#include <GPSport.h>
#include <stdio.h>
#include <stdlib.h>
#include <SD.h>

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

// GPS module constants
#define GPS_ENABLE_PIN 18


/* - - - - - - - - - encoder variables - - - - - - - - - - */

int8_t encoder_pos_;
int8_t last_encoder_pos_;
int8_t last_a_state_;
int8_t last_b_state_;
unsigned long last_encoder_time_;
unsigned long last_switch_time_;
const uint8_t enc_bounce_time_ = 10;
const uint8_t switch_bounce_time_ = 200;
bool advance_state_;
bool last_state_printed_;


/* - - - - - - - - OLED display variables - - - - - - - - - - */

Adafruit_SSD1305 display_(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
bool disp_state_change_;
const uint8_t chars_per_line_ = 21;
const uint8_t max_lines_ = 3;


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

/* - - - - - - - - - - GPS Variables - - - - - - - - - - - */

static NMEAGPS gps;
int latitude_, longitude_;


/* - - - - - - - - - SD Card Variables - - - - - - - - - - */

float declination_;


/* - - - - - - - - State Control Structure - - - - - - - - */

/* The program's state and behavior are managed through
 * a graph structure. Each node in the graph holds state
 * information, pointers to possible next states, and
 * a function pointer defining the behavior of that node.
 */

typedef void (*stateFunction)();

typedef struct stateNode
{
  char disp_options[max_lines_][chars_per_line_];
  uint8_t num_options;
  uint8_t num_lines;
  bool switch_active;
  stateNode *next_states[5];
  stateFunction action;
  stateFunction init_func;
  stateFunction end_func;
  //sateNode() : init_func(NULL), end_func(NULL) {}
};

stateNode *cur_state_;
volatile int8_t highlighted_state_;

void setup() 
{
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.println("Testing user interface with orientation filter");
  digitalWrite(GPS_ENABLE_PIN, LOW); // start with GPS off
  DEBUG_PORT.println("Initializing Encoder");
  initEncoder();
  DEBUG_PORT.println("Initializing display");
  initDisplay();
  DEBUG_PORT.println("initializing states");
  cur_state_ = initStatePointers();
  DEBUG_PORT.println("states initialized");
}

void loop() 
{
  // need to add filter update step
  unsigned long cur_time = millis();
  checkSwitchState(cur_time);
  updateDeviceState();
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

// calls the action function for the device's current state and
// updates the device's state if necessary
void updateDeviceState()
{
  // find which option should be highlighted
  if (encoder_pos_ != last_encoder_pos_)
  {
    DEBUG_PORT.print("encoder pos: ");
    DEBUG_PORT.println(encoder_pos_);
    DEBUG_PORT.print("highlighted option: ");
    DEBUG_PORT.println(highlighted_state_);
  }

  cur_state_->action();
  last_encoder_pos_ = encoder_pos_;

  // advance state if switch is pressed
  if (advance_state_)
  {
    DEBUG_PORT.println("resetting switch state");
    advance_state_ = false;
    disp_state_change_ = true;

    if (cur_state_->end_func != NULL)
    {
      DEBUG_PORT.println("running state end func");
      cur_state_->end_func();
    }

    DEBUG_PORT.println("advancing state");
    cur_state_ = cur_state_->next_states[highlighted_state_];

    if (cur_state_->init_func != NULL)
    {
      DEBUG_PORT.println("running state init func");
      cur_state_->init_func();
    }
    encoder_pos_ = 0;
    highlighted_state_ = 0;
  }
}

void selectFromList()
{
  // update the display only if the display state has changed
  if (encoder_pos_ != last_encoder_pos_ || disp_state_change_)
  {
    if (disp_state_change_)
      disp_state_change_ = false;
    display_.clearDisplay();
    display_.setCursor(0,0);
    printSelectList();
    display_.display();
  }
}

void printSelectList()
{
  for (uint8_t i = 0; i < cur_state_->num_options; i++)
  {
    if (i == highlighted_state_)
      display_.setTextColor(BLACK,WHITE);
    else
      display_.setTextColor(WHITE);
    display_.println(cur_state_->disp_options[i]);
  }
  display_.setTextColor(WHITE);
}

void checkEncoderState()
{
  unsigned long cur_time = millis();
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

    highlighted_state_ = encoder_pos_ % cur_state_->num_options;
  	if (highlighted_state_ < 0)
    	highlighted_state_ += cur_state_->num_options;

    DEBUG_PORT.print("encoder updated to: ");
    DEBUG_PORT.println(encoder_pos_);
    DEBUG_PORT.print("highlighted option: ");
    DEBUG_PORT.println(highlighted_state_);
  }
}

void checkSwitchState(unsigned long cur_time)
{
  if (cur_time - last_switch_time_ > switch_bounce_time_)
  {
    if (digitalRead(ENCODER_SWITCH) == LOW)
    {
      if (cur_state_->switch_active)
      	advance_state_ = true;
      last_switch_time_ = cur_time;
      DEBUG_PORT.println("switch press detected");
    }
  }
}

void displayText()
{
  if (disp_state_change_)
  {
    display_.clearDisplay();
    display_.setCursor(0,0);
    printText();
    display_.display();
    disp_state_change_ = false;
  }
}

void printText()
{
  for (int i = 0; i < cur_state_->num_lines; i++)
    display_.println(cur_state_->disp_options[i]);
}

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

stateNode* initStatePointers()
{
  stateNode *orient_node = new stateNode();
  strcpy(orient_node->disp_options[0],"1) auto orient");
  strcpy(orient_node->disp_options[1],"2) manual orient");
  strcpy(orient_node->disp_options[2],"3) help");
  orient_node->num_lines = 3;
  orient_node->num_options = 3;
  orient_node->action = selectFromList;
  orient_node->init_func = NULL;
  orient_node->end_func = NULL;
  orient_node->switch_active = true;

  stateNode *gps_node = new stateNode();
  stateNode *dec_lookup = new stateNode();
  stateNode *auto_orient = new stateNode();
  stateNode *manual_orient = new stateNode();
  stateNode *orient_help = new stateNode();

  orient_node->next_states[0] = gps_node;
  orient_node->next_states[1] = manual_orient;
  orient_node->next_states[2] = orient_help;

  gps_node->next_states[0] = dec_lookup;
  gps_node->num_options = 1;
  gps_node->num_lines = 2;
  strcpy(gps_node->disp_options[0],"finding latitude");
  strcpy(gps_node->disp_options[1],"and longitude.");
  gps_node->init_func = initGPS; 
  gps_node->action = updateGPS; // not yet implemented
  gps_node->end_func = endGPS; // not yet implemented
  gps_node->switch_active = false;

  dec_lookup->next_states[0] = auto_orient;
  dec_lookup->num_options = 1;
  dec_lookup->num_lines = 2;
  strcpy(dec_lookup->disp_options[0],"finding local");
  strcpy(dec_lookup->disp_options[1],"magnetic declination");
  dec_lookup->init_func = initSD; // not yet implemented
  dec_lookup->action = lookupDeclination; // not yet implemented
  dec_lookup->end_func = endSD; // not yet implemented
  dec_lookup->switch_active = false;

  stateNode *dead_end = new stateNode();
  strcpy(dead_end->disp_options[0],"nothing here");
  dead_end->num_options = 1;
  dead_end->num_lines = 1;
  dead_end->action = displayText;
  dead_end->init_func = NULL;
  dead_end->end_func = NULL;
  dead_end->next_states[0] = dead_end;

  auto_orient->num_options = 1;
  auto_orient->num_lines = 1;

  strcpy(auto_orient->disp_options[0], "press OK when done");
  auto_orient->init_func = initIMU;
  auto_orient->action = orientFilter;
  auto_orient->end_func = NULL;
  auto_orient->next_states[0] = dead_end;
  auto_orient->switch_active = true;
  
  strcpy(manual_orient->disp_options[0], "align the hinge axis");
  strcpy(manual_orient->disp_options[1], "with the pole star");
  strcpy(manual_orient->disp_options[2], "and press OK");
  manual_orient->num_options = 1;
  manual_orient->num_lines = 3;
  manual_orient->action = displayText;
  manual_orient->init_func = NULL;
  manual_orient->end_func = NULL;
  manual_orient->next_states[0] = dead_end;
  manual_orient->switch_active = true;

  strcpy(orient_help->disp_options[0], "orient help stub");
  orient_help->num_options = 1;
  orient_help->num_lines = 1;
  orient_help->action = displayText;
  orient_help->init_func = NULL;
  orient_help->end_func = NULL;
  orient_help->next_states[0] = orient_node;
  orient_help->switch_active = true;

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

  // set interrupt functions
  attachInterrupt(0, checkEncoderState, CHANGE);
  attachInterrupt(1, checkEncoderState, CHANGE);

  // initialize encoder variables
  encoder_pos_ = 0;
  last_encoder_pos_ = 0;
  last_a_state_ = digitalRead(ENCODER_A);
  last_b_state_ = digitalRead(ENCODER_B);
  last_encoder_time_ = millis();
  last_switch_time_ = millis();
  advance_state_ = false;
  last_state_printed_ = false;
}

void initGPS()
{
  DEBUG_PORT.println("enabling gps");
  digitalWrite(GPS_ENABLE_PIN, HIGH);
  delay(100);
  DEBUG_PORT.println("beginning communication with gps");
	gpsPort.begin(9600);
  DEBUG_PORT.println("gps initialized");
}

void endGPS()
{
  digitalWrite(GPS_ENABLE_PIN, LOW);
}

void updateGPS()
{
  while (gps.available(gpsPort))
    printGPSStatus(gps.read());
}

static void printGPSStatus(const gps_fix &fix)
{
  if (fix.valid.location)
  {
    DEBUG_PORT.println("gps fix acquired");
    display_.clearDisplay();
    display_.setCursor(0,0);
    display_.println("gps fix acquired");
    display_.print("latitude: ");
    display_.println(fix.latitude());
    display_.print("longitude: ");
    display_.println(fix.longitude());
    display_.display();
    delay(1000);
    longitude_ = fix.longitude();
    latitude_ = fix.longitude();
    advance_state_ = true;
  }
  else
  {
    DEBUG_PORT.println("waiting for gps fix");
    display_.clearDisplay();
    display_.setCursor(0,0);
    printText();
    display_.display();
  }
}

void initSD()
{
  
}

void endSD()
{
  
}

void lookupDeclination()
{
  
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

  roll_ = filter_.getRoll();
  pitch_ = filter_.getPitch();
  yaw_ = filter_.getYaw();

  last_filter_update_ += filter_update_time_;
}

void printOrientation()
{
  display_.print("roll: ");
  display_.println(roll_);
  display_.print("pitch: ");
  display_.println(pitch_);
  display_.print("yaw: ");
  display_.println(yaw_);
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

void strAppend(char *str1, uint8_t size1, char *str2, uint8_t size2)
{
  uint8_t index1 = 0;
  uint8_t index2 = 0;
  while (index1 < size1 && str1[index1] != '\0')
  {
    index1++;
  }
  while (index1 < size1 && index2 < size2 && str2[index2] != '\0')
  {
    str1[index1++] = str2[index2++];
  }
}

void readLine(char *buf, File dec_file)
{
  int index = 0;
  do 
  {
    buf[index++] = dec_file.read();
  } while (dec_file.peek() != '\n');
}
