// Define variables for PID control
float lastError = 0;
float integral = 0;

// Define constants for PID control
const float Kp = 0.5;  // Proportional gain
  const float Ki = 0;  // Integral gain
  const float Kd = 0;  // Derivative gain

void setup() {
  // put your setup code here, to run once:  
}

int pidControl(int sensorValues[]) {
  // Calculate error
  float error = 0;
  error =  (-40) * sensorValues[0] + 
           (-30) * sensorValues[1] + 
           (-20) * sensorValues[2] + 
             (0) * sensorValues[3] + 
             (0) * sensorValues[4] + 
           (+20) * sensorValues[5] + 
           (+30) * sensorValues[6] + 
           (+40) * sensorValues[7];
  
  // Calculate integral
  integral += error;

  // Calculate derivative
  float derivative = error - lastError;

  // Calculate PID control
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;

  // Update last error
  lastError = error;
  return pidOutput;
}


void loop() {
  // put your main code here, to run repeatedly:
}
