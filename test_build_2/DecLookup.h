/* - - - - - - - - - SD Card Reader Pins - - - - - - - - */

#define SD_CS_PIN 29 // chip select pin


/* - - - - - - - - - SD Reader Variables - - - - - - - - */

double declination_;


/* - - - - - - - - - SD Reader Functions - - - - - - - - */

void initDeclinationNode(stateNode *dec_lookup)
{
  dec_lookup->num_options = 1;
  dec_lookup->num_lines = 2;
  strcpy(dec_lookup->disp_options[0],"finding local");
  strcpy(dec_lookup->disp_options[1],"magnetic declination");
  dec_lookup->init_func = initSD; 
  dec_lookup->action = lookupDeclination;
  dec_lookup->end_func = NULL;
  dec_lookup->switch_active = false;
}

// starts communication with the SD card
void initSD()
{
  if (!SD.begin(SD_CS_PIN))
    DEBUG_PORT.println("SD card initialization failed");
  DEBUG_PORT.println("SD card initialized");
}

// sets the magnetic declination for the device's current
// location and time given latitude, longitude, and
// a year found by GPS
void lookupDeclination()
{
  // hardcoding latitude and longitude until GPS is working
  latitude_ = 40;
  longitude_ = -105;
  int year = 2018;
  char filename[40];
  memset(filename, '\0', 40);
  getFileName(latitude_, longitude_, year, filename, 40);

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

// accepts the current lattitude, longitude, and year (found
// by GPS) a C-string for returning a filename, and the size
// of that C-string as parameters
// constructs the name of the appropriate declination file
// from the lattitude, longitude, and year and returns the
// filename in the given C-string
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

// accepts a C-string and a file descriptor as parameters
// reads a line from the given file into the C-string
void readLine(char *buf, File dec_file)
{
  int index = 0;
  do 
  {
    buf[index++] = dec_file.read();
  } while (dec_file.peek() != '\n');
}

// accepts two C strings and their sizes (in bytes) as parameters
// appends str2 to the end of str1
// if str1 does not have enough room for str2, str2 is truncated
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