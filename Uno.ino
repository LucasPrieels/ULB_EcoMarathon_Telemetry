int potentiometerPin = A0; // https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
int currentPin = A1;
int voltagePin = A2;
int potentiometerValue = 0, currentValue = 0, voltageValue = 0;
uint8_t potentiometerValue_8bits = 0, currentValue_8bits = 0, voltageValue_8bits = 0;
int cpt = 0;

void setup() {
  Serial.begin(9600);
  pinMode(potentiometerPin, INPUT);
  pinMode(currentPin, INPUT);
  pinMode(voltagePin, INPUT);
}

void loop() {
  potentiometerValue = analogRead(potentiometerPin);
  currentValue = analogRead(currentPin);
  voltageValue = analogRead(voltagePin);

  potentiometerValue_8bits = potentiometerValue/4;  // Divided by 4 to fit on 8 bits (max size for UART)
  currentValue_8bits = currentValue/4;
  voltageValue_8bits = voltageValue/4;

  Serial.write(potentiometerValue_8bits);
  Serial.write(potentiometerValue%4);
  Serial.write(currentValue_8bits);
  Serial.write(currentValue%4);
  Serial.write(voltageValue_8bits);
  Serial.write(voltageValue%4);
  delay(200);
  cpt++;
}
