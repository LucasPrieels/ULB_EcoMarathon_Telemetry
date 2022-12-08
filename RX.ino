#include <LoRa.h>
#include <movingAvg.h>

bool show_packets = false;
bool send_serial_studio = true;

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
  uint16_t time;
};

data_struct data;
GPS_struct position;
movingAvg avgSpeedSmooth(10); // Smoother speed
float avgSpeedAll = 0; // Average of the speed since last reset
int countAvg = 0; // Counter of nb of averaged points
int last_time = 0; // Time of the last position sample
int distance = 0; // Distance traveled since last reset
float electric_power = 0; // Electric power used at this time, in W
int electric_energy = 0; // Electric energy used since last reset, in J

void setup() {
  avgSpeedSmooth.begin();
  Serial.begin(9600);
  while (!Serial);
  
  if (!send_serial_studio) Serial.println("LoRa Receiver");
  
  if (!LoRa.begin(868E6)) {
    if (!send_serial_studio) Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(8);
  //LoRa.setSignalBandwidth(7.8E3);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    if (show_packets){
      Serial.print("Received packet (");
      Serial.print(packetSize);
      Serial.print(" byte long) '");
    }

    if (packetSize == 16){ // If position packet
      // read packet
      if (show_packets == false && send_serial_studio == false) Serial.println("Position updating");
      while (LoRa.available()) {
        LoRa.readBytes((uint8_t*)&position, packetSize);
        avgSpeedSmooth.reading(position.speed);
        avgSpeedAll = (avgSpeedAll*countAvg + position.speed)/(countAvg + 1); // Add one measurement to the avg
        countAvg++;
        if (last_time != 0) distance += position.speed*(position.time-last_time); // Update traveled distance (to be divided by 3.6*32 to be in meters)
        if (last_time != 0) electric_energy += round(electric_power*(position.time-last_time)); // Update total electric energy used, using last recorded electric power
        last_time = position.time; // Update time of last position sample
        if (show_packets) print_GPS_struct();
      }
    }
    else{
      // read packet
      if (show_packets == false && send_serial_studio == false) Serial.println("Data updating");
      while (LoRa.available()) {
        LoRa.readBytes((uint8_t*)&data, packetSize);
        electric_power = ((float)data.current*5000/1024-2500)/66*((float)data.voltage*5/1024);
        if (show_packets) print_data_struct();
      }
    }

    if (show_packets){
      // print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
    }
    else if (!send_serial_studio){
      Serial.print("Acceleration : ");
      Serial.print((float)data.acceleration*8/1000, 2);
      Serial.println(" g");

      Serial.print("Potentiometer : ");
      Serial.print((double)data.potentiometer/10.24, 1);
      Serial.println(" %");

      Serial.print("Current : ");
      Serial.print(((float)data.current*5000/1024-2500)/66, 1); // Transform in mV, then put in around 0 and turn it into A because for the 30A sensor, 66mV/A
      Serial.println(" A");

      Serial.print("Voltage : ");
      Serial.print((float)data.voltage*5/1024, 1);
      Serial.println(" V");

      Serial.print("Speed : ");
      Serial.print((float)position.speed/32, 1);
      Serial.println(" km/h");

      Serial.print("Smooth (averaged) speed : ");
      Serial.print((float)avgSpeedSmooth.getAvg()/32, 1);
      Serial.println(" km/h");

      Serial.print("Average speed since last reset : ");
      Serial.print((float)avgSpeedAll/32, 1);
      Serial.println(" km/h");

      Serial.print("Distance traveled since last reset : ");
      Serial.print((float)distance/3.6/32, 1); // Divided by 3.6 to go from km/h to m/s, and by 32 because speed must be divided by 32
      Serial.println(" m");

      Serial.print("Electric power used : ");
      Serial.print(electric_power, 1);
      Serial.println(" W");

      Serial.print("Electric energy used since last reset : ");
      Serial.print(electric_energy);
      Serial.println(" J");

      Serial.print("Position : ");
      Serial.print(position.latitude, 7);
      Serial.print("°N ");
      Serial.print(position.longitude, 7);
      Serial.println("°E ");

      Serial.print("Time (12-hour clock) : ");
      uint16_t t = position.time; // Epoch time
      Serial.println(String((t/3600+2)%12) + ":" + String((t/60)%60) + ":" + String(t%60));

      Serial.println();
    }
    else{
      Serial.print("/*");
      Serial.print(data.count);
      Serial.print(",");
      Serial.print(position.count);
      Serial.print(",");
      uint16_t t = position.time; // Epoch time (UTC)
      Serial.print(String((t/3600+2)%12) + "," + String((t/60)%60) + "," + String(t%60));
      Serial.print(",");
      Serial.print((double)data.potentiometer*100/1024); // To be in %
      Serial.print(",");
      Serial.print(((float)data.current*5000/1024-2500)/66); // Transform in mV, then put in around 0 and turn it into A because for the 30A sensor, 66mV/A
      Serial.print(",");
      Serial.print((float)data.voltage*5/1024); // To be between 0 and 5
      Serial.print(",");
      Serial.print(electric_power);
      Serial.print(",");
      Serial.print(electric_energy);
      Serial.print(",");
      Serial.print(position.latitude, 7);
      Serial.print(",");
      Serial.print(position.longitude, 7);
      Serial.print(",");
      Serial.print(0); // Altitude
      Serial.print(",");
      Serial.print((float)position.speed/32);
      Serial.print(",");
      Serial.print((float)avgSpeedSmooth.getAvg()/32);
      Serial.print(",");
      Serial.print((float)avgSpeedAll/32);
      Serial.print(",");
      Serial.print((float)distance/3.6/32); // Divided by 3.6 to go from km/h to m/s, and by 32 because speed must be divided by 32
      Serial.print(",");
      Serial.print(data.acceleration*8*9.81/1000); // In m/s^2
      Serial.println("*/");
    }
  }
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
