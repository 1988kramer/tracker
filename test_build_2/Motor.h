// Motor.h
// Andrew Kramer
//
// Part of star-tracker program
// Defines variables and functions necessary for
// PID control of the tracker's DC motor

/* - - - - - - - - - Motor Control Pins - - - - - - - - - - */

#define LIMIT_SWITCH_PIN 20
#define MOTOR_ENC_A      2
#define MOTOR_ENC_B      1
#define MOTOR_PWM        3
#define MOTOR_DIR_PIN    0


/* - - - - - - - - - Motor Control Variables - - - - - - - */

int exposure_time_;            // exposure time in seconds
volatile long motor_count_;    // number of encoder ticks registered
long new_motor_count_;         // number of ticks at current PID update
long old_motor_count_;         // number of ticks at last PID update
const double kP_ = 0.018;
const double kI_ = 0.00005;
const double kD_ = 0.0001;

unsigned long last_pid_update_time_; // time PID was last updated (ms)
double last_pid_speed_;              // motor speed at last PID update 
									 // in ticks/sec
double last_speed_;					 // motor speed at last speed calc
									 // in ticks/sec
double set_point_;                   // motor speed setpoint
int pwm_setting_;
double i_term_;
const int pid_delta_t_target_ = 250; // PID update period
unsigned long motor_start_time_;     // time the motor started running (ms)


/* - - - - - - - - Motor Control Functions - - - - - - - - */

void initMotorNode(stateNode *run_motor)
{
  run_motor->num_options = 1;
  run_motor->num_lines = 0;
  run_motor->init_func = initMotor; // not implemented
  run_motor->action = runMotor; // not implemented
  run_motor->end_func = stopMotor;
  run_motor->switch_active = false;
}

void initResetNode(stateNode *reset_motor)
{
  strcpy(reset_motor->disp_options[0], "resetting motor");
  reset_motor->num_options = 1;
  reset_motor->num_lines = 1;
  reset_motor->init_func = resetInit; // not implemented
  reset_motor->action = resetMotor; // not implemented
  reset_motor->end_func = stopMotor;
  reset_motor->switch_active = false;
}

// updates the voltage to the motor using a PID control loop
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

// returs the motor's average speed over  
// the last deltaT microseconds
double getSpeed(double deltaT)
{
  old_motor_count_ = new_motor_count_;
  new_motor_count_ = motor_count_;
  int difference = new_motor_count_ - old_motor_count_;
  if (difference < 0) difference *= -1; 
  double cur_speed;

  // check if overflow has occurred
  if (difference < 50000)
  {
    cur_speed = (double)difference / deltaT;
    last_speed_ = cur_speed;
  }
  else // if so, assume speed is unchanged from last interval
  {
    cur_speed = last_speed_;
  }
  return cur_speed;
}

// updates PWM output to maintoin the set_point_ speed
// accepts the motor's current speed, calculated elsewhere
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

// sets the motor direction and speed for a tracking motion
// clears the display to avoid producing unnecessary light
void initMotor()
{
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  set_point_ = 98.0;
  resetPIDVars();
  
  motor_start_time_ = millis();
  display_.clearDisplay();
  display_.display();
}

// runs the motor forward (extending the actuator) for 
// a specified amount of time (exposure_time_)
void runMotor()
{
  while (millis() - motor_start_time_ < exposure_time_ * 1000)
    updatePID();
  advance_state_ = true;
}

// stops the motor when motion is complete
void stopMotor()
{
  analogWrite(MOTOR_PWM, 0);
}

// sets the motor direction and speed to 
// reverse the previous motion
void resetInit()
{
  digitalWrite(MOTOR_DIR_PIN, LOW);
  set_point_ = 800.0;
  resetPIDVars();
}

// moves the actuator back to the starting 
// position after an exposure is completed
void resetMotor()
{
  while(digitalRead(LIMIT_SWITCH_PIN) == HIGH)
  {
    updatePID();
    displayText();
  }
  advance_state_ = true;
}

// resets pid variables so there is no "kick"
// after changing directions and speeds
// still not working properly
void resetPIDVars()
{
  i_term_ = 0.0;
  last_speed_ = 0.0;
  last_pid_speed_ = 0.0;
  motor_count_ = 0;
  new_motor_count_ = 0;
  old_motor_count_ = 0;
  last_pid_update_time_ = millis();
}

// updates the count of ticks for the motor encoder 
// should be attached to a pin change interrupt
// for MOTOR_ENC_A
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