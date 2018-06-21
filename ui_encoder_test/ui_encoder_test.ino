#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>

#define ENCODER_A 10
#define ENCODER_B 11
#define ENCODER_SWITCH 30

#define OLED_CLK 27
#define OLED_MOSI 26
#define OLED_CS 25
#define OLED_DC 28
#define OLED_RESET 24

volatile unsigned int encoderPos = 0;

Adafruit_SSD1305 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(ENCODER_SWITCH, INPUT);

  attachInterrupt(0, encoderAFunc, CHANGE);
  attachInterrupt(1, encoderBFunc, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void encoderAFunc()
{
  if (digitalRead(ENCODER_A) == HIGH)
  {
    if (digitalRead(ENCODER_B) == LOW)
      encoderPos++;
    else
      encoderPos--;
  }
  else
  {
    if (digitalRead(ENCODER_B) == HIGH)
      encoderPos++;
    else
      encoderPos--;
  }
  Serial.println(encoderPos);
}

void encoderBFunc()
{
  if (digitalRead(ENCODER_B) == HIGH)
  {
    if (digitalRead(ENCODER_A) == HIGH)
      encoderPos++;
    else
      encoderPos--;
  }
  else
  {
    if (digitalRead(ENCODER_A) == LOW)
      encoderPos++;
    else
      encoderPos--;
  }
  Serial.println(encoderPos);
}

