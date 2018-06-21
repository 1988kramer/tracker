#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

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

void getFileName(int lat, int lon, int year, char *filename, uint8_t name_size)
{
  uint8_t str_size = 10;
  //Serial.println("building filename");
  memset(filename, '\0', name_size);
  
  char year_str[str_size];
  memset(year_str, '\0', str_size);
  itoa(year, year_str, 10);
  strAppend(filename, name_size, year_str, str_size);
  //Serial.println(filename);
  strAppend(filename, name_size, "_0/", 3);
  //Serial.println(filename);

  char lat_str[str_size];
  memset(lat_str, '\0', str_size);
  itoa(abs(lat), lat_str, 10);

  if (lat < 0)
    strAppend(filename, name_size, "n", 1);
  strAppend(filename, name_size, lat_str, str_size);
  //Serial.println(filename);

  strAppend(filename, name_size, ".csv", 4);
  //Serial.println(filename);  
}

void readLine(char *buf, File dec_file)
{
  int index = 0;
  do 
  {
    buf[index++] = dec_file.read();
  } while (dec_file.peek() != '\n');
}

bool findDeclination(int lat, int lon, char *filename, float &declination)
{
  if (!SD.begin(29))
  {
    Serial.println("SD initialization failed");
  }
  Serial.println("SD card initialized");
  
  File dec_file;
  dec_file = SD.open(filename, FILE_READ);

  if (dec_file)
  {
    Serial.println("file opened");
    bool target_found = false;
    while (dec_file.available() && !target_found)
    {
      char line[25];
      memset(line,'\0',25);
      readLine(line, dec_file);
      char *token = strtok(line,",");
      int this_lat = atoi(token);
      token = strtok(NULL,",");
      int this_lon = atoi(token);
      token = strtok(NULL,",");
      declination = atof(token);

      target_found = (this_lat == lat && this_lon == lon);
    }
    return target_found;
  }
  else
  {
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  delay(500);
  char filename[40];
  memset(filename, '\0', 40);
  getFileName(-90,180,2018,filename,40);
  Serial.println(filename);
  float declination;
  if(findDeclination(-90, 180, filename, declination))
  {
    Serial.print("found correct declination: ");
    Serial.println(declination);
  } 
  else
  {
    Serial.println("did not find correct declination"); 
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
