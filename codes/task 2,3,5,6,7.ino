#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

const int IR_array[] = {A0, A1, A2, A3, A4, A5, A6, A7};  //A0-RightMost
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


// Define pins for the first sensor
const int trigPin1 = 2;
const int echoPin1 = 3;

// Define pins for the second sensor
const int trigPin2 = 4;
const int echoPin2 = 5;

// Define pins for the third sensor
const int trigPin3 = 6;
const int echoPin3 = 7;

// Define pins for the fourth sensor
const int trigPin4 = 8;
const int echoPin4 = 9;

// Define pins for the fifth sensor
const int trigPin5 = 10;
const int echoPin5 = 11;

// Define motor control pins
const int leftMotorPWM = 22;   // PWM pin for speed control
const int leftMotorDir1 = 23;  // Digital pin for direction control 1
const int leftMotorDir2 = 24;  // Digital pin for direction control 2

const int rightMotorPWM = 25;  // PWM pin for speed control
const int rightMotorDir1 = 26; // Digital pin for direction control 1
const int rightMotorDir2 = 27; // Digital pin for direction control 2

// Analog input pin for the sound sensor
const int soundSensorPin = A0;  

int ultradist1 = 0;
int ultradist2 = 0;
int ultradist3 = 0;
int ultradist4 = 0;
int ultradist5 = 0;

int getDistance(int trigPin, int echoPin) {
  // Generate a pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate and return the distance in centimeters
  return duration * 0.034 / 2;
}

void moveForward() {
  // Set motor directions to move forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);

  // Set motor speeds
  analogWrite(leftMotorPWM, 255);  // Adjust the speed as needed
  analogWrite(rightMotorPWM, 255); // Adjust the speed as needed
}

void moveBackward() {
  // Set motor directions to move backward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);

  // Set motor speeds
  analogWrite(leftMotorPWM, 255);  // Adjust the speed as needed
  analogWrite(rightMotorPWM, 255); // Adjust the speed as needed
}

void stopRobot() {
  // Set motor directions to stop
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);

  // Stop the motors
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}

void turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]){
  int leftSpeed=0;
  int rightSpeed=0;
  delay(500);

  while (Digitalized_Sensor_Values[3]!=1 && Digitalized_Sensor_Values[4]!=1){
  int leftSpeed = +60 ;
  int rightSpeed = -60 ;
  delay(500);
  }
}

void turnleft(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]){
  int leftSpeed=0;
  int rightSpeed=0;
  delay(500);

  while (Digitalized_Sensor_Values[3]!=1 && Digitalized_Sensor_Values[4]!=1){
  int leftSpeed = -60 ;
  int rightSpeed = +60 ;
  delay(500);
  }
}

void turn180(){
  int leftSpeed=0;
  int rightSpeed=0;
  delay(500);
  }

  while (Digitalized_Sensor_Values[3]!=1 && Digitalized_Sensor_Values[4]!=1){
  int leftSpeed = -60 ;
  int rightSpeed = +60 ;
  delay(500);
  }
}

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Set up pins for ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);

  // Defining pinMode of motor control pins
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
}

void loop() {
  // Measure distance for the first sensor
  ultradist1 = getDistance(trigPin1, echoPin1);

  // Measure distance for the second sensor
  ultradist2 = getDistance(trigPin2, echoPin2);

  // Measure distance for the third sensor
  ultradist3 = getDistance(trigPin3, echoPin3);

  // Measure distance for the fourth sensor
  ultradist4 = getDistance(trigPin4, echoPin4);

  // Measure distance for the fifth sensor
  ultradist5 = getDistance(trigPin5, echoPin5);

  delay(1000); // Wait for a second before the next set of measurements

  ////checkpoint2////

  ultradist= min(min(min(min(ultradist1,ultradist2),ultradist3),ultradist4),ultradist5);
  if (ultradist<700){
    stage=2;
    pidout=pidcontrol(ultradist-500);
    if (digitalizedSensorValues[3]==0||digitalizedSensorValues[4]==0){
      stage=1;
      }
    }

   ////checkpoint3////

    BaseSpeed=50;
   
   ////checkpoint5////
  
  if (ultradist1<200){
    moveForward();
    }
  Serial.println("Color sensor initialized.");
  uint16_t red, green, blue, clear;
  tcs.getRawData(&red, &green, &blue, &clear);

  get_the_box();
  BaseSpeed=50; //reduce speed 
  if (digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
    moveBackward();
  }else{
    stopRobot();
    }
 
  if (red>blue){
    turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
    callibarate();  //change IR threshold for red
    BaseSpeed=100; //increase speed 
    while (pureAnalogSensorReadings[7]<100){
    if (digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
    if (digitalizedSensorValues[0] != 1 || digitalizedSensorValues[1] != 1 || digitalizedSensorValues[2] != 1 || digitalizedSensorValues[3] != 1 || digitalizedSensorValues[4] != 1 || digitalizedSensorValues[5] != 1 || digitalizedSensorValues[6] != 1 || digitalizedSensorValues[7] != 1 ) {
        turn180();
    }
    moveForward();
    }else
    {
      turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
    }
   }
   
   ////checkpoint6////
   
   keepbox();  //keep box on red square
   int soundValue = analogRead(soundSensorPin);
   int soundThreshold=50;
   if (soundValue>soundThreshold){
    while(digitalizedSensorValues[0] != 0 || digitalizedSensorValues[0] != 0){
      moveForward();
      }
      turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
      while not(digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
      moveForward();
      }
     stopRobot(); 
    }
  }
  
  else{
    turnleft(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
    callibarate();  //change IR threshold for red
    BaseSpeed=100; //increase speed 
    while (pureAnalogSensorReadings[7]<100){
    if (digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
    if (digitalizedSensorValues[0] != 1 || digitalizedSensorValues[1] != 1 || digitalizedSensorValues[2] != 1 || digitalizedSensorValues[3] != 1 || digitalizedSensorValues[4] != 1 || digitalizedSensorValues[5] != 1 || digitalizedSensorValues[6] != 1 || digitalizedSensorValues[7] != 1 ) {
        turn180();
    }
    moveForward();
    }else
    {
      turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
    }
   }
   
   ////checkpoint6////
   
   keepbox();  //keep box on red square
   int soundValue = analogRead(soundSensorPin);
   int soundThreshold=50;
   if (soundValue>soundThreshold){
    while(digitalizedSensorValues[0] != 0 || digitalizedSensorValues[0] != 0){
      moveForward();
      }
      turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
      while not(digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
      moveForward();
      }
     stopRobot(); 
    }
  }

  ////checkpoint7////
  
  if (ultradist3<200){
    if (ultradist1>200){
       stopRobot(); 
      }
    else{
      serial.println("Bot moves away");
      while (digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
      BaseSpeed=50; //reduce speed 
      moveForward();
      }
      BaseSpeed=100; //increase speed
      turnright(Digitalized_Sensor_Values[3],Digitalized_Sensor_Values[4]);
      while (digitalizedSensorValues[0] != 0 || digitalizedSensorValues[1] != 0 || digitalizedSensorValues[2] != 0 || digitalizedSensorValues[3] != 0 || digitalizedSensorValues[4] != 0 || digitalizedSensorValues[5] != 0 || digitalizedSensorValues[6] != 0 || digitalizedSensorValues[7] != 0 ) {
      moveForward();
      serial.println("Victory");
      }
    }
}

