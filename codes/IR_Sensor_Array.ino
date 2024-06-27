// Include necessary libraries
#include <Arduino.h>

//Global Variable Creation
int pure_Sensor_Values[8];
// Define pin numbers for IRArray sensors
const int IR_array[] = {A0, A1, A2, A3, A4, A5, A6, A7}; //A0-RightMost


void setup() {
  // Define PinMode for IRArray sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(IR_array[i], INPUT);
  }
}

void read_Sensors() {
  for (int i = 0; i < 8; i++) {
    pure_Sensor_Values[i] = analogRead(IR_array[i]);
  }
}


void loop() {
// Function to read sensor values
}
