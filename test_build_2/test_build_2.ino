#include <NMEAGPS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include "MadgwickAHRS.h"
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
#define GPS_ENABLE_PIN 19

// SD card chip select pin
#define SD_CS_PIN 29

// motor control pins
#define LIMIT_SWITCH_PIN 20
#define MOTOR_ENC_A      2
#define MOTOR_ENC_B      1
#define MOTOR_PWM        3
#define MOTOR_DIR_PIN    0

// power switch pin
#define OFF_SWITCH  14

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

#include <GPSport.h>
static NMEAGPS gps;
int latitude_, longitude_, year_;


/* - - - - - - - - - SD Card Variables - - - - - - - - - - */

float declination_;


/* - - - - - - - - - Motor Control Variables - - - - - - - */

unsigned long exposure_time_ = 0;
volatile long motor_count_;
long new_motor_count_, old_motor_count_;
const double kP_ = 0.3;
const double kI_ = 0.1;
const double kD_ = 0.005;

unsigned long last_pid_update_time_;
double last_pid_speed_;
double last_speed_;
double set_point_;
int pwm_setting_;
double i_term_;
const int pid_delta_t_target_ = 250;
unsigned long motor_start_time_;


/* - - - - - - - - - Timeout Variables - - - - - - - - - - */

unsigned long last_action_time_;
const unsigned long timeout_time_ = 120000;
bool timeout_active_;


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
  while(!DEBUG_PORT);
  DEBUG_PORT.println("Testing user interface with orientation filter");
  pinMode(GPS_ENABLE_PIN, OUTPUT);
  digitalWrite(GPS_ENABLE_PIN,LOW); // start with GPS disabled
  //digitalWrite(GPS_ENABLE_PIN, HIGH); // start with GPS on
  //gpsPort.begin(9600);
  DEBUG_PORT.println("Initializing Encoder");
  initEncoder();
  DEBUG_PORT.println("Initializing display");
  initDisplay();
  DEBUG_PORT.println("initializing states");
  cur_state_ = initStatePointers();
  DEBUG_PORT.println("states initialized");
  
  attachInterrupt(2, readMotorEncoder, CHANGE);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENC_A, INPUT);
  pinMode(MOTOR_ENC_B, INPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(OFF_SWITCH, OUTPUT);
  last_action_time_ = millis();
  timeout_active_ = true;
}

void loop() 
{
  // need to add filter update step
  unsigned long cur_time = millis();
  checkSwitchState(cur_time);
  updateDeviceState();

  // power off if device is inactive for too long
  if (timeout_active_ 
      && millis() - last_action_time_ > timeout_time_)
    digitalWrite(OFF_SWITCH, HIGH);
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

    last_action_time_ = millis();

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
      last_action_time_ = millis();
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

void initDisplay()
{
  display_.begin();
  display_.clearDisplay();
  display_.setCursor(0,0);
  display_.setTextColor(WHITE);
  display_.println("Alicia's star");
  display_.println("tracking camera");
  display_.println("mount for");
  display_.println("astrophotography");
  display_.println();
  display_.println("Happy Anniversary!");
  display_.display();
  delay(3000);
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
  timeout_active_ = false;
  DEBUG_PORT.println("enabling gps");
  digitalWrite(GPS_ENABLE_PIN, HIGH);
  delay(100);
  DEBUG_PORT.println("beginning communication with gps");
	gpsPort.begin(9600);
  DEBUG_PORT.println("gps initialized");
}

void endGPS()
{
  timeout_active_ = true;
  digitalWrite(GPS_ENABLE_PIN, LOW);
  last_action_time_ = millis();
}

void updateGPS()
{
  while (gps.available(gpsPort))
  {
    DEBUG_PORT.println("printing GPS status");
    printGPSStatus(gps.read());
  }
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
    latitude_ = fix.latitude();
    year_ = fix.dateTime.year + 2000;
    DEBUG_PORT.print("latitude: ");
    DEBUG_PORT.println(latitude_);
    DEBUG_PORT.print("longitude: ");
    DEBUG_PORT.println(longitude_);
    DEBUG_PORT.print("year: ");
    DEBUG_PORT.println(year_);
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

void getFileName(int8_t lat, int8_t lon, int year, char *filename, uint8_t name_size)
{
  uint8_t str_size = 10;
  memset(filename, '\0', name_size);
  
  char year_str[str_size];
  memset(year_str, '\0', str_size);
  itoa(year, year_str, 10);
  strAppend(filename, name_size, year_str, str_size);
  strAppend(filename, name_size, "_0/", 3);

  char lat_str[str_size];
  memset(lat_str, '\0', str_size);
  itoa(abs(lat), lat_str, 10);

  if (lat < 0)
    strAppend(filename, name_size, "n", 1);
  strAppend(filename, name_size, lat_str, str_size);

  strAppend(filename, name_size, ".csv", 4);
}

void readLine(char *buf, File dec_file)
{
  int index = 0;
  do 
  {
    buf[index++] = dec_file.read();
  } while (dec_file.peek() != '\n');
}

void initSD()
{
  if (!SD.begin(SD_CS_PIN))
    DEBUG_PORT.println("SD card initialization failed");
  DEBUG_PORT.println("SD card initialized");
}

void lookupDeclination()
{
  // hardcoding latitude and longitude until GPS is working
  char filename[40];
  memset(filename, '\0', 40);
  getFileName(latitude_, longitude_, year_, filename, 40);

  File dec_file;
  dec_file = SD.open(filename, FILE_READ);

  if (dec_file)
  {
    DEBUG_PORT.print("file ");
    DEBUG_PORT.print(filename);
    DEBUG_PORT.println(" opened successfully");
    bool target_found = false;
    while (dec_file.available() && !target_found)
    {
      char line[25];
      memset(line, '\0', 25);
      readLine(line, dec_file);
      char *token = strtok(line, ",");
      int8_t this_lat = atoi(token);
      token = strtok(NULL, ",");
      int8_t this_lon = atoi(token);
      token = strtok(NULL, ",");
      declination_ = atof(token);

      target_found = (this_lat == latitude_ && 
                      this_lon == longitude_);
    }
    advance_state_ = true;
    DEBUG_PORT.println("found declination");
  }
  DEBUG_PORT.println("unable to find declination");
}

void setExposure()
{
  if (disp_state_change_ || encoder_pos_ != last_encoder_pos_)
  {
    if (encoder_pos_ < 0)
    {
      encoder_pos_ = 0;
      last_encoder_pos_ = 0;
    }
    exposure_time_ = encoder_pos_ * 5;
    display_.clearDisplay();
    display_.setCursor(0,0);
    display_.println("Select exposure time");
    display_.println("(in seconds)");
    display_.println(exposure_time_);
    printText();
    display_.display();
  }
}

void encoderAUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_a_state_ = enc_a_state;
  if (enc_a_state == HIGH)
  {
    if (enc_b_state == LOW)
      encoder_pos_--;
    else
      encoder_pos_++;
  }
  else
  {
    if (enc_b_state == HIGH)
      encoder_pos_--;
    else
      encoder_pos_++;
  }
}

void encoderBUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_b_state_ = enc_b_state;
  if (enc_b_state == HIGH)
  {
    if (enc_a_state == HIGH)
      encoder_pos_--;
    else
      encoder_pos_++;
  }
  else
  {
    if (enc_a_state == LOW)
      encoder_pos_--;
    else
      encoder_pos_++;
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

void updatePID()
{
  unsigned long cur_time = millis();
  if (cur_time - last_pid_update_time_ >= pid_delta_t_target_)
  {
    double deltaT = (double)(cur_time - last_pid_update_time_) / 1000.0;
    double cur_speed = getSpeed(deltaT);
    updatePWM(cur_speed);
    analogWrite(MOTOR_PWM, pwm_setting_);
    last_pid_update_time_ = cur_time;
  }
}

double getSpeed(double deltaT)
{
  old_motor_count_ = new_motor_count_;
  new_motor_count_ = motor_count_;
  int difference = new_motor_count_ - old_motor_count_;
  if (difference < 0) difference *= -1;
  double cur_speed;
  if (difference < 50000)
  {
    cur_speed = (double)difference / deltaT;
    last_speed_ = cur_speed;
  }
  else
  {
    cur_speed = last_speed_;
  }
  DEBUG_PORT.print("speed: ");
  DEBUG_PORT.println(cur_speed);
  return cur_speed;
}

void updatePWM(double cur_speed)
{
  double error = set_point_ - cur_speed;
  i_term_ += kI_ * error;
  double d_input = cur_speed - last_pid_speed_;
  int adjustment = (kP_ * error) + i_term_ - (kD_ * d_input);
  pwm_setting_ += adjustment;
  if (pwm_setting_ > 255) pwm_setting_ = 255;
  else if (pwm_setting_ < 0) pwm_setting_ = 0;
  last_pid_speed_ = cur_speed;
}

void resetPIDVars()
{
  i_term_ = 0.0;
  last_speed_ = 0.0;
  last_pid_speed_ = 0.0;
  motor_count_ = 0;
  new_motor_count_ = 0;
  old_motor_count_ = 0;
  pwm_setting_ = 0;
  last_pid_update_time_ = millis();
}
void initMotor()
{
  timeout_active_ = false;
  
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  set_point_ = 38.99;
  //set_point_ = 100;
  
  motor_start_time_ = millis();
  display_.clearDisplay();
  display_.display();
}

void runMotor()
{
  while (millis() - motor_start_time_ < exposure_time_ * 1000)
    updatePID();
  advance_state_ = true;
}

void stopMotor()
{
  analogWrite(MOTOR_PWM, 0);
  resetPIDVars();
  timeout_active_ = true;
  last_action_time_ = millis();
}

void resetInit()
{
  timeout_active_ = false;
  digitalWrite(MOTOR_DIR_PIN, LOW);
  set_point_ = 800.0;
}

void resetMotor()
{
  while(digitalRead(LIMIT_SWITCH_PIN) == HIGH)
  {
    updatePID();
    displayText();
  }
  advance_state_ = true;
}

void readMotorEncoder()
{
  if (digitalRead(MOTOR_ENC_A) == HIGH)
  {
    if (digitalRead(MOTOR_ENC_B) == LOW)
      motor_count_++;
    else
      motor_count_--;
  }
  else
  {
    if (digitalRead(MOTOR_ENC_B) == LOW)
      motor_count_--;
    else
      motor_count_++;
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
  stateNode *set_exposure = new stateNode();
  stateNode *run_motor = new stateNode();
  stateNode *reset_motor = new stateNode();

  orient_node->next_states[0] = gps_node;
  //orient_node->next_states[0] = dec_lookup;
  orient_node->next_states[1] = manual_orient;
  orient_node->next_states[2] = orient_help;

  gps_node->next_states[0] = dec_lookup;
  gps_node->num_options = 1;
  gps_node->num_lines = 2;
  strcpy(gps_node->disp_options[0],"finding latitude");
  strcpy(gps_node->disp_options[1],"and longitude.");
  gps_node->init_func = initGPS; 
  gps_node->action = updateGPS;
  gps_node->end_func = endGPS; 
  gps_node->switch_active = false;

  dec_lookup->next_states[0] = auto_orient;
  dec_lookup->num_options = 1;
  dec_lookup->num_lines = 2;
  strcpy(dec_lookup->disp_options[0],"looking up local");
  strcpy(dec_lookup->disp_options[1],"magnetic declination");
  dec_lookup->init_func = initSD; 
  dec_lookup->action = lookupDeclination;
  dec_lookup->end_func = NULL;
  dec_lookup->switch_active = false;

  auto_orient->num_options = 1;
  auto_orient->num_lines = 3;

  strcpy(auto_orient->disp_options[0], "press OK when the");
  strcpy(auto_orient->disp_options[1], "orientations shown");
  strcpy(auto_orient->disp_options[2], "match the targets");
  auto_orient->init_func = initIMU;
  auto_orient->action = orientFilter;
  auto_orient->end_func = NULL;
  auto_orient->next_states[0] = set_exposure;
  auto_orient->switch_active = true;
  
  strcpy(manual_orient->disp_options[0], "align the hinge axis");
  strcpy(manual_orient->disp_options[1], "with the pole star");
  strcpy(manual_orient->disp_options[2], "and press OK");
  manual_orient->num_options = 1;
  manual_orient->num_lines = 3;
  manual_orient->action = displayText;
  manual_orient->init_func = NULL;
  manual_orient->end_func = NULL;
  manual_orient->next_states[0] = set_exposure;
  manual_orient->switch_active = true;

  strcpy(set_exposure->disp_options[0], "press OK to");
  strcpy(set_exposure->disp_options[1], "start exposure");
  set_exposure->num_options = 1;
  set_exposure->num_lines = 2;
  set_exposure->init_func = NULL;
  set_exposure->action = setExposure; 
  set_exposure->end_func = NULL;
  set_exposure->next_states[0] = run_motor;
  set_exposure->switch_active = true;

  run_motor->num_options = 1;
  run_motor->num_lines = 0;
  run_motor->init_func = initMotor; 
  run_motor->action = runMotor; 
  run_motor->end_func = stopMotor;
  run_motor->next_states[0] = reset_motor;
  run_motor->switch_active = false;

  strcpy(reset_motor->disp_options[0], "resetting camera");
  reset_motor->num_options = 1;
  reset_motor->num_lines = 1;
  reset_motor->init_func = resetInit; 
  reset_motor->action = resetMotor; 
  reset_motor->end_func = stopMotor;
  reset_motor->next_states[0] = set_exposure;
  reset_motor->switch_active = false;

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

