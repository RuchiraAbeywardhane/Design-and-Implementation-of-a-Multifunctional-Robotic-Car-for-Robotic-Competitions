#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>


#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 A6
#define IR8 A7

#define RMotorA 4
#define RMotorB 2
#define RMotorPWM 3

#define LMotorA 7
#define LMotorB 6
#define LMotorPWM 5

#define S0 A9
#define S1 A10
#define S2 A15
#define S3 A14
#define sensorOut A12
#define colourVcc A13
#define colourGnd A11

#define sensor A14 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

#define HighSwitch 36
#define RightSwitch 34
#define MiddleSwitch 32
#define LeftSwitch 30

#define MAX_DISTANCE 20 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


// Stores frequency read by the photodiodes
int red = 0;
int green = 0;
int blue = 0;


// Define pins for the  sensor-Front left sensor
const int trigPin_FL = 45;
const int echoPin_FL = 47;
const int gndPin_FL = 49;

// Define pins for the sensor-left
const int trigPin_L = 51;
const int echoPin_L = 53;

// Define pins for the third sensor-middle sensor
const int trigPin_M = 39;
const int echoPin_M = 41;
const int gndPin_M = 43;


// Define pins for the fourth sensor-right  sensor
const int trigPin_R = 36;
const int echoPin_R = 38;
const int gndPin_R = 40;

// Define pins for the fifth sensor-right front sensor
const int trigPin_FR = 46;
const int echoPin_FR = 48;
const int gndPin_FR = 52;


//Ground sharpir
const int sharpGND = 22;

// Declare the Servo pin
int servoPin1 = 9;
int servoPin2 = 8;

// Declare distances to walls
int ultradist = 0;
int ultradist1 = 0;
int ultradist2 = 0;
int ultradist3 = 0;
int ultradist4 = 0;
int ultradist5 = 0;

int start = 0;

int BaseSpeed = 80;
int MinSpeed = 0;
int MaxSpeed = 160;

int IR_D_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int Temp_IR_D_val[8];
int IR_A_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//int IR_Weight[8]= {-20, -10, -5, 0 , 0 , 5, 10, 20};
int IR_Weight[8] = { -35, -20, -10, -5 , 5 , 10, 20, 35};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int SpeedAdjust = 0;

float P, I, D, P_dist, I_dist, D_dist;
float Error = 0;
float PreviousError = 0;
float PreviousError_dist = 0;
float Kp = 10; //2.8
float Kd = 0;
float Ki = 0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

//Calibeartion Arrays
const int Buzzer = 51;
const int BuzzerGND = 44;
int caliberation_threshold[8];

// Create a servo object
Servo Servo1; //servo which controls rotation of arm
Servo Servo2; //servo which controls gripper part

int stage = 2;

const int soundSensorPin = A1;

void setup() {
  Serial.begin(9600);


  // Setting colour sensor outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);  // Setting the sensorOut as an input
  digitalWrite(S0, HIGH);     // Setting frequency scaling to 20%
  digitalWrite(S1, LOW);
  digitalWrite(colourVcc, HIGH);
  digitalWrite(colourGnd, LOW);

  // Set up pins for ultrasonic sensors
  pinMode(trigPin_FL, OUTPUT);
  pinMode(echoPin_FL, INPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(trigPin_M, OUTPUT);
  pinMode(echoPin_M, INPUT);
  pinMode(trigPin_R, OUTPUT);
  pinMode(echoPin_R, INPUT);
  pinMode(trigPin_FR, OUTPUT);
  pinMode(echoPin_FR, INPUT);

  //sharpir pins
  pinMode(sharpGND, OUTPUT);

  digitalWrite(sharpGND, LOW);


  //Defining Pin modes for the Buzzer(Caliberation)
  pinMode(Buzzer, OUTPUT);
  pinMode(BuzzerGND, OUTPUT);
  digitalWrite(Buzzer, LOW);
  digitalWrite(Buzzer, LOW);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);

  set_forward();
  Servo1.write(70);
  Servo2.write(90);
  delay(500);

  //ultrasonic grounds
  digitalWrite(gndPin_FR, LOW);
  //  digitalWrite(gndPin2, LOW);
  digitalWrite(gndPin_R, LOW);
  digitalWrite(gndPin_M, LOW);
  digitalWrite(gndPin_FL, LOW);

}

/////////////////Direction///////////////////////////
int turnTo(int dataset[]) { //Determine where to turn to lose the deviation
  int sumRight = 0;
  int sumLeft = 0;
  for (int i = 0; i < 4; i++) {
    sumRight += dataset[i];
    sumLeft += dataset[i + 4];
  }
  if (sumRight > sumLeft) {
    return 1; //1 means turn right, if the vehicle is turning wrong change this to 0
  }
  else {
    return 0; //0 means Turn Left
  }
}
//////////////////////////////////////////////////////


/////////////////Deviator///////////////////////////
int deviator(int dataset[]) { ///Check if the robot has deviated from the line
  if (dataset[0] == 1 && dataset[1] == 1 && dataset[2] == 1 && dataset[3] == 1 && dataset[4] == 1 && dataset[5] == 1 && dataset[6] == 1 && dataset[7] == 1 ) {
    return 1;
  }
  else {
    return 0;
  }
}
//////////////////////////////////////////////////////


/////////////////Checkpoint///////////////////////////
int checkpoint(int dataset[]) { ///Check if the robot has reached a checkpoint
  if (dataset[0] == 0 && dataset[1] == 0 && dataset[2] == 0 && dataset[3] == 0 && dataset[4] == 0 && dataset[5] == 0 && dataset[6] == 0 && dataset[7] == 0 ) {
    return 1;
  }
  else {
    return 0;
  }
}
//////////////////////////////////////////////////////
void PID_control() {
  Error = 0;
  for (int i = 0; i < 8; i++) {
    Error += IR_Weight[i] * IR_D_val[i];
  }
  P = Error;
  I = I + Error;
  D = Error - PreviousError;
  PreviousError = Error;

  SpeedAdjust = (Kp * P + Ki * I + Kd * D);

  LMotorSpeed = BaseSpeed - SpeedAdjust;
  RMotorSpeed = BaseSpeed + SpeedAdjust;
  // delay(5000);

  if (LMotorSpeed < 0 ) {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MaxSpeed) {
    LMotorSpeed = MaxSpeed;
  }
  if (RMotorSpeed > MaxSpeed) {
    RMotorSpeed = MaxSpeed;
  }

}
void stage5() {
  /*if (ultradist1<8){
    stop();
    }*/
  //Colour sensor

  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  red = pulseIn(sensorOut, LOW);
  Serial.println("R:");
  Serial.println(red);

  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  green = pulseIn(sensorOut, LOW);
  Serial.println("G:");
  Serial.println(green);

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blue = pulseIn(sensorOut, LOW);
  Serial.println("B:");
  Serial.println(blue);

  if (red > blue) {
    Serial.println('red');
  }
  else if (red < blue) {
    Serial.println('blue');
  }
}
void read_IR() {
  IR_A_val[0] = analogRead(IR1);
  IR_A_val[1] = analogRead(IR2);
  IR_A_val[2] = analogRead(IR3);
  IR_A_val[3] = analogRead(IR4);
  IR_A_val[4] = analogRead(IR5);
  IR_A_val[5] = analogRead(IR6);
  IR_A_val[6] = analogRead(IR7);
  IR_A_val[7] = analogRead(IR8);

  for (int i = 0; i < 8; i++)
  {
    if (IR_A_val[i] <= 300) {  //IR_A_val[i] >= caliberation_threshold[i] For Black Line
      IR_D_val[i] = 0;
    }
    else if (IR_A_val[i] >= 300) {
      IR_D_val[i] = 1;
    }
  }
}


void set_speed() {
  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}

void set_forward() {
  digitalWrite(LMotorB, HIGH);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, HIGH);
  digitalWrite(RMotorA, LOW);
}

void TurnRight() {
  analogWrite(LMotorPWM, 100);
  analogWrite(RMotorPWM, 100);
  digitalWrite(LMotorB, HIGH);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
}


void TurnLeft() {
  analogWrite(LMotorPWM, 100);
  analogWrite(RMotorPWM, 100);
  digitalWrite(LMotorB, LOW);
  digitalWrite(LMotorA, HIGH);
  digitalWrite(RMotorB, HIGH);
  digitalWrite(RMotorA, LOW);
}

void stop() {
  digitalWrite(LMotorB, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(RMotorA, LOW);
}


/////////////////////Caliberation//////////////////////
void caliberation() {
  int caliberation_data[8][12] = {
    {}, // Inner array 1
    {}, // Inner array 2
    {}, // Inner array 3
    {}, // Inner array 4
    {}, // Inner array 5
    {}, // Inner array 6
    {}, // Inner array 7
    {}  // Inner array 8
  };
  delay(2000);
  digitalWrite(Buzzer, HIGH);
  delay(100);
  digitalWrite(Buzzer, LOW);
  delay(3000);

  int n = 0;
  for (int i = 0; i < 12; i++) {
    Serial.println(n);
    for (int j = 0; j < 8; j++) {
      caliberation_data[j][i] = analogRead(IR_A_val[j]); //j value represents the IR single modules i represents the number of times values are taken
    }
    if (n == 2) {
      //Go forward few steps
      analogWrite(LMotorPWM, 255);
      analogWrite(RMotorPWM, 255);
      set_forward();
      delay(500);
      stop();
    }
    else if (n == 5) {
      //Double beep to notify to move the car
      digitalWrite(Buzzer, HIGH);
      delay(100);
      digitalWrite(Buzzer, LOW);
      delay(100);
      digitalWrite(Buzzer, HIGH);
      delay(100);
      digitalWrite(Buzzer, LOW);
      delay(4000);
    }
    else if (n == 8) {
      //Go forward few steps
      analogWrite(LMotorPWM, 255);
      analogWrite(RMotorPWM, 255);
      set_forward();
      delay(500);
      stop();
    }
    else if (n == 11) {
      digitalWrite(Buzzer, HIGH);
      delay(100);
      digitalWrite(Buzzer, LOW);
      delay(100);
      digitalWrite(Buzzer, HIGH);
      delay(100);
      digitalWrite(Buzzer, LOW);
      delay(100);
      digitalWrite(Buzzer, HIGH);
      delay(100);
      digitalWrite(Buzzer, LOW);
      delay(4000);
      //End
    }
    n = n + 1;
  }

  for (int j = 0; j < 8 ; j++) {
    int maxi = 0;
    int mini = 0;
    for (int i = 0; i < 12; i++) {
      if (caliberation_data[j][i] > maxi) {
        maxi = caliberation_data[j][i];
      }
      else if (caliberation_data[j][i] < mini) {
        mini = caliberation_data[j][i];
      }
    }
    int avg = (maxi + mini) / 2;
    caliberation_threshold[j] = avg;
  }
}
///////////////////////////////////////////////////////
///////////////////GetBackTothePath///////////////////
void getBackPath(int hasDeviate) {
  //direction
  int turningTo = turnTo(Temp_IR_D_val);
  if (turningTo == 0) {
    ////turnleft
    ///when black detected
    delay(250);
    while (hasDeviate == 1) {
      TurnLeft();
      read_IR();
      hasDeviate = deviator(IR_D_val);
    }

  }
  else {
    //turn right
    ///when black detected
    delay(250);
    while (hasDeviate == 1) {
      TurnRight();
      read_IR();
      hasDeviate = deviator(IR_D_val);
    }

  }
}

/////////////////GetDistance///////////////////////
float getDistance(int trigPin, int echoPin) {
  NewPing sonar(trigPin, echoPin, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
  float distance;
  delay(50);
  distance = sonar.ping_cm(); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  return distance;
}

////////////////////////////////////////////////////
////////////////////LineFollow//////////////////////
void LineFollow() {
  set_forward();
  PID_control();
  set_speed();
  for (int i = 0; i < 8; i++) {
    Temp_IR_D_val[i] = IR_D_val[i];
  }
}
////////////////////////////////////////

//////////////////////////////////////////////////////
void PID_control_dist(float Error) {
  P_dist = Error;
  I_dist = I_dist + Error;
  D_dist = Error - PreviousError_dist;
  PreviousError_dist = Error;

  SpeedAdjust = (Kp * P_dist + Ki * I_dist + Kd * D_dist);

  LMotorSpeed = BaseSpeed - SpeedAdjust;
  RMotorSpeed = BaseSpeed + SpeedAdjust;
  // delay(5000);

  if (LMotorSpeed < 0 ) {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MaxSpeed) {
    LMotorSpeed = MaxSpeed;
  }
  if (RMotorSpeed > MaxSpeed) {
    RMotorSpeed = MaxSpeed;
  }

}

void loop() {
  //////Caliberate at the start
  //if (start == 0) {
  //  caliberation();
  //  start += 1;
  //}
  read_IR(); //Read from IR sensor Array
  stage5();

  ////Detect Checkpoints///
  if (checkpoint(IR_D_val)) {
    delay(200);
    //read_IR();
    stop();
  }
  else { //Car is not at a checkpoint


    if (stage == 1) {
      Serial.println("Stage01");
      int hasDeviate = deviator(IR_D_val);
      if (hasDeviate == 1) {
        getBackPath(hasDeviate);
      }///////End of Has Deviate If statement
      else {
        LineFollow();
      }
    }
    //////End of Stage01



    else if (stage == 2) {
      
      /////Stage02
      analogWrite(LMotorPWM, 150);
      analogWrite(RMotorPWM, 150);
      set_forward();
      
      if (getDistance(trigPin_FL, echoPin_FL) < 15) {
        analogWrite(LMotorPWM, 150);
        analogWrite(RMotorPWM, 0);
        TurnLeft();
        delay(500);
        set_forward();
      }
      else if ((getDistance(trigPin_R, echoPin_R)) < 15) {
        ///Obs
        analogWrite(LMotorPWM, 0);
        analogWrite(RMotorPWM, 150);
        TurnRight();
        delay(500);
        set_forward();
      }
    }
  }
}
