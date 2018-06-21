/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>
#include <stdio.h>
#include <stdlib.h>

File dec_file;

void readLine(char *buf)
{
  int index = 0;
  do 
  {
    buf[index++] = dec_file.read();
  } while (dec_file.peek() != '\n');
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(29)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dec_file = SD.open("2018_0/n90.csv", FILE_READ);

  unsigned long start_time = millis();
  // if the file opened okay, process line-by-line
  // until target is found
  if (dec_file) {
    Serial.println("searching file");
    bool target_found = false;
    int target_lat = -90;
    int target_lon = 180;
    double declination = 0.0;
    while (dec_file.available() && !target_found)
    {
      char line[25];
      memset(line,'\0',25);
      readLine(line);
      //Serial.print("raw line: ");
      //Serial.println(line);
      char *token = strtok(line,",");
      //Serial.print("parsed output: ");
      int lat = atoi(token);
      //Serial.print(lat);
      //Serial.print(" ");
      token = strtok(NULL,",");
      int lon = atoi(token);
      //Serial.print(lon);
      //Serial.print(" ");
      token = strtok(NULL,",");
      declination = atof(token);
      //Serial.println(declination);
      //delay(500);
      
      target_found = (lat == target_lat && lon == target_lon);
    }
    unsigned long elapsed_time = millis() - start_time;
    int secs = elapsed_time / 1000;
    if (target_found)
    {
      Serial.print("Entry found with declination of ");
      Serial.println(declination);
    }
    else
    {
      Serial.println("Entry not found");
    }
    Serial.print("Time elapsed: ");
    Serial.println(secs);
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}


