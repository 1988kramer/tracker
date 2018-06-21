#define ENCODER_A 2
#define ENCODER_B 1
#define PWM_PIN   3
#define DIR_PIN1  0

#define DELTA_T_TARGET  250

volatile long count;
long new_count, old_count;
const double kP = 0.18;
const double kI = 0.00005;
const double kD = 0.0001;

int last_time, pid_last_speed;
double last_speed;
double set_point;
int pwm;
double i_term;

double av_speed;
double prev_time;

void setup()
{
  Serial.begin(9600);
  attachInterrupt(2, readEncoder, CHANGE);
  //speed_control.setGains(kP,kI,kD);
  last_time = millis();
  i_term = 0.0;
  last_speed = 0.0;
  digitalWrite(DIR_PIN1, HIGH);
  //speed_control.setSpeed(1500);
  set_point = 98.0;
  av_speed = 0.0;
  prev_time = 0.0;
}

void loop()
{
  int current_time = millis();
  if (current_time - last_time >= DELTA_T_TARGET)
  {
    /*
    speed_control.adjustPWM();
    last_time = current_time;
    Serial.println(motor_encoder.getSpeed());
    */
    double deltaT = (double)(current_time - last_time) / 1000.0;
    double cur_speed = getSpeed(deltaT);
    updatePWM(cur_speed);
    //Serial.println(cur_speed);
    analogWrite(PWM_PIN,pwm);
    last_time = current_time;
  }
}

void updatePWM(double cur_speed)
{
  double error = set_point - cur_speed;
  i_term += kI * error;
  double d_input = cur_speed - pid_last_speed;
  int adjustment = (kP * error) + i_term - (kD * d_input);
  pwm += adjustment;
  if (pwm > 255) pwm = 255;
  else if (pwm < 0) pwm = 0;
  pid_last_speed = cur_speed;
}

double getSpeed(double deltaT)
{
  old_count = new_count;
  new_count = count;
  int difference = new_count - old_count;
  /*
  Serial.print(difference);
  Serial.print("   ");
  Serial.println(deltaT);
  */
  double cur_speed;
  if (difference < 50000 && difference > -50000)
  {
    cur_speed = (double)difference / deltaT;
    last_speed = cur_speed;
  }
  else
  {
    cur_speed = last_speed;
  }
  //updateAvSpeed(cur_speed, deltaT);
  return cur_speed;
}

void updateAvSpeed(double cur_speed, double deltaT)
{
  av_speed *= prev_time;
  av_speed += cur_speed * deltaT;
  prev_time += deltaT;
  av_speed /= prev_time;
  Serial.print(av_speed);
  Serial.print(" ticks/second at t = ");
  Serial.println(prev_time);
}

void readEncoder()
{
  //motor_encoder.updateCount();
  if (digitalRead(ENCODER_A) == HIGH)
  {
    if (digitalRead(ENCODER_B) == LOW)
      count++;
    else
      count--;
  }
  else
  {
    if (digitalRead(ENCODER_B) == LOW)
      count--;
    else
      count++;
  }
}


