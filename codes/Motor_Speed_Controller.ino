void setup() {
  // put your setup code here, to run once:
  // Define motor control pins
  const int leftMotorPWM = 2;   // PWM pin for speed control
  const int leftMotorDir1 = 23;  // Digital pin for direction control 1
  const int leftMotorDir2 = 24;  // Digital pin for direction control 2
  
  const int rightMotorPWM = 3;  // PWM pin for speed control
  const int rightMotorDir1 = 25; // Digital pin for direction control 1
  const int rightMotorDir2 = 26; // Digital pin for direction control 2
  
  
  // Defining pinMode of motor control pins
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
}


void control_Motors_Speed(int scaledPidOut) {
  // Set motor speeds based on PID output
  int leftSpeed = BaseSpeed - scaledPidOut;
  int rightSpeed = BaseSpeed + scaledPidOut;

  // Clip motor speeds to valid range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Control motors
  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:

}
