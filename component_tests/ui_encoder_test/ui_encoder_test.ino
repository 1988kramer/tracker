#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>

// encoder_pins
#define ENCODER_A 10
#define ENCODER_B 11
#define ENCODER_SWITCH 12

// OLED pins
#define OLED_CLK 27
#define OLED_MOSI 26
#define OLED_CS 25
#define OLED_DC 28
#define OLED_RESET 24

// encoder variables
int encoder_pos;
int last_encoder_pos;
uint8_t last_a_state;
uint8_t last_b_state;
unsigned long last_encoder_time;
unsigned long last_switch_time;
const uint8_t enc_bounce_time = 10;
const uint8_t switch_bounce_time = 200;
bool switch_pressed;

Adafruit_SSD1305 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup() 
{
  initEncoder();
  initDisplay();
  Serial.begin(9600);
  delay(500);
  Serial.println("Testing user interface control with encoder");
}

void loop() 
{
  unsigned long cur_time = millis();
  checkEncoderState(cur_time);
  checkSwitchState(cur_time);
  updateDisplay();
}

void updateDisplay()
{
  if (switch_pressed)
    encoder_pos = 0;
  uint8_t num_options = 3;
  int8_t highlighted = encoder_pos % num_options;
  if (highlighted < 0)
    highlighted += num_options;
  if (encoder_pos != last_encoder_pos)
  {
    last_encoder_pos = encoder_pos;
    display.clearDisplay();
    display.setCursor(0,0);
    Serial.print("highlighted option: ");
    Serial.println(highlighted);
    for (uint8_t i = 0; i < num_options; i++)
    {
      if (i == highlighted)
        display.setTextColor(BLACK,WHITE);
      else
        display.setTextColor(WHITE);
      display.print(i);
      display.print(") Option #");
      display.println(i);
    }
    display.display();
  }
}

void checkEncoderState(unsigned long cur_time)
{
  if (cur_time - last_encoder_time > enc_bounce_time)
  {
    uint8_t enc_a_state = digitalRead(ENCODER_A);
    uint8_t enc_b_state = digitalRead(ENCODER_B);
    if (enc_a_state != last_a_state)
    {
      encoderAUpdate(enc_a_state, enc_b_state);
      last_encoder_time = cur_time;
    }
    else if (enc_b_state != last_b_state)
    {
      encoderBUpdate(enc_a_state, enc_b_state);
      last_encoder_time = cur_time;
    }
  }
}

void checkSwitchState(unsigned long cur_time)
{
  if (cur_time - last_switch_time > switch_bounce_time)
  {
    if (digitalRead(ENCODER_SWITCH) == LOW)
    {
      switch_pressed = true;
      last_switch_time = cur_time;
      Serial.println("switch press detected");
    }
  }
  else if (switch_pressed)
  {
    switch_pressed = false;
    Serial.println("switch state reset");
  }
}

void initDisplay()
{
  display.begin();
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(WHITE);
  display.println("UI test with OLED");
  display.println("and rotary encoder");
  display.display();
  delay(1000);
}

void initEncoder()
{
  // set encoder pin states
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH, INPUT_PULLUP);

  // initialize encoder variables
  encoder_pos = 0;
  last_encoder_pos = 1;
  last_a_state = digitalRead(ENCODER_A);
  last_b_state = digitalRead(ENCODER_B);
  last_encoder_time = millis();
  last_switch_time = millis();
  switch_pressed = false;
}

void encoderAUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_a_state = enc_a_state;
  if (enc_a_state == HIGH)
  {
    if (enc_b_state == LOW)
      encoder_pos++;
    else
      encoder_pos--;
  }
  else
  {
    if (enc_b_state == HIGH)
      encoder_pos++;
    else
      encoder_pos--;
  }
  Serial.print("encoder updated to: ");
  Serial.println(encoder_pos);
}

void encoderBUpdate(uint8_t enc_a_state, uint8_t enc_b_state)
{
  last_b_state = enc_b_state;
  if (enc_b_state == HIGH)
  {
    if (enc_a_state == HIGH)
      encoder_pos++;
    else
      encoder_pos--;
  }
  else
  {
    if (enc_a_state == LOW)
      encoder_pos++;
    else
      encoder_pos--;
  }
  Serial.print("encoder updated to: ");
  Serial.println(encoder_pos);
}

