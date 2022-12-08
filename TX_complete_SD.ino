#include <LoRa.h>
#include <Wire.h>
#include <DFRobot_LIS2DH12.h>
#include <Arduino_MKRGPS.h>
#include <SPI.h>
#include <SD.h>
#include <movingAvg.h>

const int chipSelect = 6; // Arduino pin connected to D3 of SD card breakout module
String filename; // Name of file to log data on the SD card

bool error_GPS = false; // Becomes true if GPS can't be started, to avoid being stuck
// If the GPS doesn't work, the accelerometer freezes (because they are connected in I2C ?)

DFRobot_LIS2DH12 LIS; // Accelerometer

struct data_struct{ // Structure that is sent wirelessly as much as possible
  uint16_t count; // ID number
  uint8_t acceleration; // Divided by 8 to fit on 8 bits
  uint16_t potentiometer;
  uint16_t current;
  uint16_t voltage;
};

struct GPS_struct{ // Structure that is sent wirelessly less often (because takes time)
  uint16_t count;
  uint16_t speed; // Multiplied by 32 and rounded to keep integer values
  float latitude, longitude;
  uint16_t time; // Epoch time (12-hour clock)
};

float record_altitude = -1; // Save altitude to write on the SD card but not send it wirelessly
movingAvg avgSpeedSmooth(10); // Smoother speed
float avgSpeedAll = 0; // Average of the speed since last reset
int countAvg = 0; // Counter of nb of averaged points
int last_time = 0; // Time of the last position sample
int distance = 0; // Distance traveled since last reset
float electric_power = 0; // Electric power used at this time, in W
int electric_energy = 0; // Electric energy used since last reset, in J

data_struct data = {-1, -1, -1, -1}; // Initial value
GPS_struct position = {-1, -1, -1, -1}; // Initial value
int count = 1;

void setup(){
  avgSpeedSmooth.begin();
  Serial.begin(9600); // Serial is the serial between the WAN and the other Arduino through Tx/Rx ports
  Serial1.begin(9600); // Serial1 is the serial between the WAN and the computer through USB
  //while (!Serial); // Wait for the Serial to be opened
  Serial.println("LoRa Sender");
  
  Wire.begin(); // Join I2C bus as a master (for accelerometer and GPS)

  // Init of LoRa communication
  if (!LoRa.begin(868E6)) {
    Serial.println("################## ERROR : Starting LoRa failed!");
  }
  LoRa.setSpreadingFactor(8); // Don't forget to put it as well on RX
  LoRa.setTxPower(25);
  //LoRa.setSignalBandwidth(7.8E3);

  // Init of accelerometer
  if (LIS.init(LIS2DH12_RANGE_2GA) == -1){ // The argument of init specifies the measurement range of the accelerometer (2GA, 4GA, 8GA, 16GA)
    Serial.println("################## ERROR : No I2C devices found"); // Equipment connection exception or I2C address error
  }

  // Init of GPS
  if (!GPS.begin()) { // No argument to .begin() because I2C communication and not UART
    Serial.println("################## ERROR : Failed to initialize GPS!");
    error_GPS = true;
  }

  // Init of SD card log
  Serial.print("Initializing SD card...");
  // See if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("################## ERROR : Card failed, or not present");
  }
  Serial.println("Card initialized.");
  // Choose name of file to log data : log_nbOfLog.csv
  int cpt = 0;
  do{
    cpt++;
    filename = "log_" + String(cpt) + ".csv";
    Serial.println(filename);
  } while (SD.exists(filename)); // Choose the first nb of log that doesn't exist yet on the SD card
  
  data.count = 1; // Number of sent data
}

void loop(){
  // Retrieves acceleration value
  int16_t acc = acceleration();
  Serial.print("Acceleration value : ");
  Serial.println(acc);
  data.acceleration = acc/8;

  // Checks if receiving data by UART
  if (Serial1.available()){
    Serial.print("Data received (");
    Serial.print(Serial1.available()-6); // Number of data in buffer after reading the 6 next ones
    Serial.print(" left) : ");
    data.potentiometer = Serial1.read()*4+Serial1.read(); // Value has been divided in 2 frames to fit on 8 bits (max for UART)
    data.current = Serial1.read()*4+Serial1.read();
    data.voltage = Serial1.read()*4+Serial1.read();
    Serial.print(data.potentiometer);
    Serial.print(" ");
    Serial.print(data.current);
    Serial.print(" ");
    Serial.println(data.voltage);

    while (Serial1.available() > 1){
      for (int i = 0; i < 6; i++) Serial1.read(); // Discard all data stored in buffer, to avoid sending old values
      // Discard 6 by 6 to avoid deleting a potentiometer value but not a current value for example
    }
  }

  if (count % 1 == 0){ // We send only once every x packets (but all data are stored on the SD card)
    // Sends LoRa packet with data
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&data, sizeof(data));
    LoRa.endPacket();
    print_data_struct();
    data.count++;
  
    // Retrieves GPS data
    Serial.print("GPS value : ");
    readGPSData();
  
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&position, sizeof(position));
    LoRa.endPacket();
    print_GPS_struct();
    position.count++;
  }

  
  avgSpeedSmooth.reading(position.speed);
  avgSpeedAll = (avgSpeedAll*countAvg + position.speed)/(countAvg + 1); // Add one measurement to the avg
  countAvg++;
  if (last_time != 0) distance += position.speed*(position.time-last_time); // Update traveled distance (to be divided by 3.6*32 to be in meters)
  electric_power = ((float)data.current*5000/1024-2500)/66*((float)data.voltage*5/1024);
  if (last_time != 0) electric_energy += round(electric_power*(position.time-last_time)); // Update total electric energy used, using last recorded electric power
  last_time = position.time; // Update time of last position sample

  // Write data on SD card
  String dataString = "1," + String(data.count) + "," + String(position.count) + ","; // Add a 1 as title of the line
  uint16_t t = position.time; // Epoch time
  dataString += String((t/3600+2)%12) + "," + String((t/60)%60) + "," + String(t%60) + ",";
  dataString += String((float)data.potentiometer*100/1024) + "," + String(((float)data.current*5000/1024-2500)/66) + ",";
  dataString += String((float)data.voltage*5/1024) + "," + String(electric_power) + "," + String(electric_energy) + ",";
  dataString += String(position.latitude, 7) + "," + String(position.longitude, 7) + "," + String(record_altitude) + ",";
  dataString += String((float)position.speed/32) + "," + String((float)avgSpeedSmooth.getAvg()/32) + ",";
  dataString += String((float)avgSpeedAll/32) + "," + String((float)distance/3.6/32) + "," + String((float)data.acceleration*8*9.81/1000);
  
  File dataFile = SD.open(filename, FILE_WRITE); // Open file

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("Error opening file ");
    Serial.println(filename);
  }

  count++;
}

int16_t acceleration(void) {
  if (error_GPS) return 0; // If the GPS doesn't work, the accelerometer freezes for some reason (surely because they both use I2C connection)
  int16_t x, y, z;
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  return round(sqrt(x*x+y*y+z*z));
}

void readGPSData(){
  if (error_GPS) return;
  for (int i=0; i<5000; i++){ // Poll the GPS 5000 times, break if we get new data
    if (GPS.available()) {
      // read GPS values
      float latitude   = GPS.latitude();
      float longitude  = GPS.longitude();
      float altitude   = GPS.altitude();
      float speed      = GPS.speed();
      int   satellites = GPS.satellites();
      uint16_t t = GPS.getTime()%(60*60*12); // We only keep the 12-hour clock, not the day, to fit on 16 bits
      int s = t%60;
      int m = (t/60)%60;
      int h = (t/3600+2)%12; // +2 because the clock gives UTC time
  
      // print GPS values
      Serial.print("Location: ");
      Serial.print(latitude, 7);
      Serial.print(", ");
      Serial.println(longitude, 7);
  
      Serial.print("Altitude: ");
      Serial.print(altitude);
      Serial.println("m");
  
      Serial.print("Ground speed: ");
      Serial.print(speed);
      Serial.println(" km/h");
  
      Serial.print("Number of satellites: ");
      Serial.println(satellites);

      Serial.println("Time (12-hour clock) : " + String(h) + ":" + String(m) + ":" + String(s));
      
      Serial.println();

      // Stores data
      position.latitude = latitude;
      position.longitude = longitude;
      position.speed = round(32*speed);
      position.time = t;
      record_altitude = altitude; // Save altitude to write on the SD card but not send it wirelessly
      break;
    }
  }
  Serial.println("");
}

void print_data_struct(){
  Serial.print(data.count);
  Serial.print(", ");
  Serial.print(8*data.acceleration);
  Serial.print(", ");
  Serial.print(data.potentiometer);
  Serial.print(", ");
  Serial.print(data.current);
  Serial.print(", ");
  Serial.println(data.voltage);
}

void print_GPS_struct(){
  Serial.print(position.count);
  Serial.print(", ");
  Serial.print(position.latitude, 7);
  Serial.print(", ");
  Serial.print(position.longitude, 7);
  Serial.print(", ");
  Serial.print((float)position.speed/32);
  Serial.print(", ");
  uint16_t t = position.time;
  Serial.println(String((t/3600+2)%12) + ":" + String((t/60)%60) + ":" + String(t%60));
}

void increase_speed_GPS(){ // Increase the refresh rate of the GPS
  // To use it, change measRate and on the last line of the file GPS.cpp of the library, change 9600 to 460800
  byte payload[6];

  memset(payload, 0x00, sizeof(payload));
    
  uint16_t measRate = 200;    // ms between gps data publish  (min of 100)
  uint16_t navRate  =   1;    // ratio between measurements and nav solution calculations
  uint16_t timeRef  =   1;    // The time system to which measurements are aligned (0: UTC, 1:GPS, ...)

  // write out the unsigned shorts in little-endian form
  payload[0] = measRate & 0xff;    
  payload[1] = (measRate >> 8) & 0xff;    
  payload[2] = navRate & 0xff;
  payload[3] = (navRate >> 8) & 0xff;
  payload[4] = timeRef & 0xff;
  payload[5] = (timeRef >>8) && 0xff;

  GPS.sendUbx(0x06, 0x08, payload, sizeof(payload));
}
