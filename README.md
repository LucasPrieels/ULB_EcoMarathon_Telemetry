# ULB Eco-Marathon Telemetry

## Transmission side
Upload the Uno.ino file in the Arduino Uno. It takes care of the following part:
- Read potentiometer, current and voltage value
- Send these values via Serial to the Arduino MKR WAN 1300

Upload the TX_complete_SD.ino file in the Arduino MKR WAN 1300 in the car, taking care of the transmission. It takes care of the following parts:
- Initialisation and controlling of the accelerometer
- Initialisation and controlling of the GPS device
- Receive data from the Arduino Uno
- Create data packets and send them to the receiver via the antenna
- Writing data on the SD card

## Receiving side
Upload the RX.ino file in the Arduino MKR WAN 1300 at the computed side. It takes care of the following parts:
- Receive data packets via the antenna
- Send data via Serial to the receiving computer

On the receiving computer, run the Serial Studio software. It will take care of receiving data via Serial from the Rx Arduino MKR WAN 1300 and plot data. To let the software know the structure of data it is receiving and how to plot it, import the file Telemetry.json as "JSON project file".

## Current status and next steps
The report explains the implementation of a fully-working telemetry system measuring potentiometer value, current and voltage in the motor/battery, acceleration, GPS position, speed etc, send them wirelessly to a computer and stores it on an SD card. Because the car electrical circuit was far from complete last year, I have not yet been able to put this telemetry system in the car. The next step is therefore to install all the components in the car. The Arduino Uno used in the current telemetry system should be replaced by the Arduino Uno currently in use in the car, and the code Uno.ino should be merged with the current file implementing the speed regulation, which is currently in the Uno in the car. Then, the car should be tested and data should be analyzed to understand how the car should be driven to decrease the energy consumption. A possible improvement is to show live data to the driver (current speed, energy left etc)

For more details, please refer to the report. 
