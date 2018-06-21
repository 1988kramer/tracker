#define ENCODER_A 10
#define ENCODER_B 11

uint8_t last_a_state;
uint8_t last_b_state;
volatile unsigned long last_time;
const int bounce_time = 10;

int encoderPos = 0;

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  Serial.begin(9600);

  last_a_state = digitalRead(ENCODER_A);
  last_b_state = digitalRead(ENCODER_B);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t enc_a_state = digitalRead(ENCODER_A);
  uint8_t enc_b_state = digitalRead(ENCODER_B);
  unsigned long cur_time = millis();
  if (cur_time - last_time > bounce_time)
  {
    last_time = cur_time;
    if (enc_a_state != last_a_state)
    {
      last_a_state = enc_a_state;
      if (enc_a_state == HIGH)
      {
        if (enc_b_state == LOW)
          encoderPos++;
        else
          encoderPos--;
      }
      else
      {
        if (enc_b_state == HIGH)
          encoderPos++;
        else
          encoderPos--;
      }
      Serial.println(encoderPos);
    }
    else if (enc_b_state != last_b_state)
    {
      last_b_state = enc_b_state;
      if (enc_b_state == HIGH)
      {
        if (enc_a_state == HIGH)
          encoderPos++;
        else
          encoderPos--;
      }
      else
      {
        if (enc_a_state == LOW)
          encoderPos++;
        else
          encoderPos--;
      }
      Serial.println(encoderPos);
    }
  }
}

