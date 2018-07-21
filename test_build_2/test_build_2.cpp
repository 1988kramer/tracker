#include <NMEAGPS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>
#include <stdio.h>
#include <stdlib.h>

#include "Motor.h"
#include "DecLookup.h"

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

// GPS module constants
#define GPS_ENABLE_PIN 19


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


/* - - - - - - - - - - GPS Variables - - - - - - - - - - - */

#include <GPSport.h>
static NMEAGPS gps;
int latitude_, longitude_;


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
}

void loop() 
{
  // need to add filter update step
  unsigned long cur_time = millis();
  checkSwitchState(cur_time);
  updateDeviceState();
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

void setExposure()
{
  if (disp_state_change_ || encoder_pos_ != last_encoder_pos_)
  {
    if (encoder_pos_ < 0)
    {
      encoder_pos_ = 0;
      last_encoder_pos_ = 0;
    }
    exposure_time_ = encoder_pos_;
    display_.clearDisplay();
    display_.setCursor(0,0);
    display_.println("Select exposure");
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

  //orient_node->next_states[0] = gps_node;
  orient_node->next_states[0] = dec_lookup;
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

  initDeclinationNode(dec_lookup);
  dec_lookup->next_states[0] = auto_orient;

  initOrientNode(auto_orient);
  auto_orient->next_states[0] = set_exposure;
  
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
  set_exposure->action = setExposure; // not implemented
  set_exposure->end_func = NULL;
  set_exposure->next_states[0] = run_motor;
  set_exposure->switch_active = true;

  initMotorNode(run_motor);
  run_motor->next_states[0] = reset_motor;

  initResetNode(reset_motor);
  reset_motor->next_states[0] = set_exposure;

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

