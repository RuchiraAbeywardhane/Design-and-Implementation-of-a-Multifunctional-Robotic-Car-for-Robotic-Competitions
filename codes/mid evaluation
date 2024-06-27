#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4 
#define IR6 A5
#define IR7 A6
#define IR8 A7

#define RMotorA 27
#define RMotorB 26
#define RMotorPWM 3

#define LMotorA 25
#define LMotorB 24
#define LMotorPWM 2

int start = 0;

int MotorBaseSpeed = 80;
int MinSpeed=0;
int MaxSpeed= 160;

int IR_D_val[8]={0,0,0,0,0,0,0,0};
int Temp_IR_D_val[8];
int IR_A_val[8]={0,0,0,0,0,0,0,0};
//int IR_Weight[8]= {-20, -10, -5, 0 , 0 , 5, 10, 20};
int IR_Weight[8]= {-35,-20, -10, -5 , 5 , 10, 20, 35};

int LMotorSpeed =0;
int RMotorSpeed =0;
int SpeedAdjust=0;

float P,I,D;
float Error=0;
float PreviousError=0;
float Kp=7;//2.8
float Kd=14;
float Ki=0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

//Calibeartion Arrays
const int Buzzer = 50;
const int BuzzerGND = 44;
int caliberation_threshold[8];

void setup(){
    Serial.begin(9600);
    
    //Defining Pin modes for the Buzzer(Caliberation)
    pinMode(Buzzer,OUTPUT);
    pinMode(BuzzerGND,OUTPUT);
    digitalWrite(Buzzer,LOW);
    digitalWrite(Buzzer,LOW);
  
    pinMode(IR1,INPUT);
    pinMode(IR2,INPUT);
    pinMode(IR3,INPUT);
    pinMode(IR4,INPUT);
    pinMode(IR5,INPUT);
    pinMode(IR6,INPUT);
    pinMode(IR7,INPUT);
    pinMode(IR8,INPUT);

    pinMode(RMotorA,OUTPUT);
    pinMode(RMotorB,OUTPUT);
    pinMode(RMotorPWM,OUTPUT);

    pinMode(LMotorA,OUTPUT);
    pinMode(LMotorB,OUTPUT);
    pinMode(LMotorPWM,OUTPUT);

    set_forward();
    delay(2000);

}



/////////////////Direction///////////////////////////
int turnTo(int dataset[]){ //Determine where to turn to lose the deviation
  int sumRight = 0;
  int sumLeft = 0;
  for(int i=0; i<4; i++){
    sumRight += dataset[i];
    sumLeft += dataset[i+4];
  }
  if(sumRight>sumLeft){
    return 0; //1 means turn right, if the vehicle is turning wrong change this to 0
  }
  else{
    return 1; //0 means Turn Left
  }
}
//////////////////////////////////////////////////////


/////////////////Deviator///////////////////////////
int deviator(int dataset[]){ ///Check if the robot has deviated from the line
  if(dataset[0]==1 && dataset[1]==1 && dataset[2]==1 && dataset[3]==1 && dataset[4]==1 && dataset[5]==1 && dataset[6]==1 && dataset[7]==1 ){ 
    return 1;
  }
  else{
    return 0;
  }
}
//////////////////////////////////////////////////////


/////////////////Checkpoint///////////////////////////
int checkpoint(int dataset[]){ ///Check if the robot has reached a checkpoint
  if(dataset[0]==0 && dataset[1]==0 && dataset[2]==0 && dataset[3]==0 && dataset[4]==0 && dataset[5]==0 && dataset[6]==0 && dataset[7]==0 ){ 
    return 1;
  }
  else{
    return 0;
  }
}
//////////////////////////////////////////////////////



void PID_control(){
    Error=0;
    for(int i=0;i<8;i++){
        Error += IR_Weight[i]*IR_D_val[i];  
          }
    P=Error;
    I= I+Error;
    D= Error -PreviousError;
    PreviousError = Error;

    SpeedAdjust=(Kp*P + Ki*I + Kd*D);

    LMotorSpeed= MotorBaseSpeed + SpeedAdjust;
    RMotorSpeed = MotorBaseSpeed - SpeedAdjust;

    Serial.print("SADj : ");
    Serial.print(SpeedAdjust);
    Serial.print(" LM  : ");
    Serial.print(LMotorSpeed);
    Serial.print(" RM  : ");
    Serial.print(RMotorSpeed);
    Serial.println(" ");
   // delay(5000);

    if (LMotorSpeed <0 ){
        LMotorSpeed=0;
    }
    if (RMotorSpeed<0){
        RMotorSpeed=0;
    }
    if (LMotorSpeed > MaxSpeed){
        LMotorSpeed = MaxSpeed;
    }
    if (RMotorSpeed > MaxSpeed){
        RMotorSpeed = MaxSpeed;
    }

}

void read_IR(){
    IR_A_val[0] = analogRead(IR1);
    IR_A_val[1] = analogRead(IR2);
    IR_A_val[2] = analogRead(IR3);
    IR_A_val[3] = analogRead(IR4);
    IR_A_val[4] = analogRead(IR5);
    IR_A_val[5] = analogRead(IR6);
    IR_A_val[6] = analogRead(IR7);
    IR_A_val[7] = analogRead(IR8);

  for (int i =0; i<8; i++)
  {
    if (IR_A_val[i]>=caliberation_threshold[i]) {IR_D_val[i]=1;}
    else if (IR_A_val[i]<=caliberation_threshold[i]) {IR_D_val[i]=0;}
    //Serial.print(i);
    //Serial.print("   : ");
    //Serial.print(IR_A_val[i]);
    //Serial.print(":A -> D :");
    //Serial.println(IR_D_val[i]);
    
  }
  //delay(5000);
}


void set_speed(){
    analogWrite(LMotorPWM,LMotorSpeed);
    analogWrite(RMotorPWM,RMotorSpeed);
}

void set_forward(){
    digitalWrite(LMotorB,HIGH);
    digitalWrite(LMotorA,LOW);
    digitalWrite(RMotorB,HIGH);
    digitalWrite(RMotorA,LOW);
}

void TurnRight(){
    analogWrite(LMotorPWM,100);
    analogWrite(RMotorPWM,100);
    digitalWrite(LMotorB,HIGH);
    digitalWrite(LMotorA,LOW);
    digitalWrite(RMotorB,LOW);
    digitalWrite(RMotorA,HIGH);
}


void TurnLeft(){
    analogWrite(LMotorPWM,100);
    analogWrite(RMotorPWM,100);
    digitalWrite(LMotorB,LOW);
    digitalWrite(LMotorA,HIGH);
    digitalWrite(RMotorB,HIGH);
    digitalWrite(RMotorA,LOW);
}

void stop(){
    digitalWrite(LMotorB,LOW);
    digitalWrite(LMotorA,LOW);
    digitalWrite(RMotorB,LOW);
    digitalWrite(RMotorA,LOW);
}


/////////////////////Caliberation//////////////////////
void caliberation(){
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
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(3000);

    int n=0;
    for (int i = 0; i < 12; i++) {
    Serial.println(n);
      for(int j = 0; j < 8; j++){         
        caliberation_data[j][i] = analogRead(IR_A_val[j]); //j value represents the IR single modules i represents the number of times values are taken
      }
    if(n==2){
      //Go forward few steps
    analogWrite(LMotorPWM,255);
    analogWrite(RMotorPWM,255);
    set_forward();
    delay(500);
    stop();
      }
    else if(n==5){
    //Double beep to notify to move the car
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(4000);
    }
    else if(n==8){
    //Go forward few steps
    analogWrite(LMotorPWM,255);
    analogWrite(RMotorPWM,255);
    set_forward();
    delay(500);
    stop();
    }
    else if(n==11){
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(4000);
    //End
    }
    n=n+1;
    }

for(int j=0; j<8 ;j++){
      int maxi=0;
      int mini=0;
      for(int i=0; i<12; i++){
        if(caliberation_data[j][i]>maxi){
          maxi = caliberation_data[j][i];
        }
        else if(caliberation_data[j][i]<mini){
          mini=caliberation_data[j][i];
        }
      }
     int avg = (maxi+mini)/2;
     caliberation_threshold[j] = avg;
    }
}
///////////////////////////////////////////////////////


void loop() {
  if(start==0){
    caliberation();
    start += 1;
  }
    read_IR();

    if (checkpoint(IR_D_val)){
      delay(200);
      //read_IR();  
      stop();
      }
      else{
  /*  if (IR_D_val[0]==0 && IR_D_val[1]==0 && IR_D_val[2]==0 && IR_D_val[3]==0 && IR_D_val[4]==0 && IR_D_val[5]==0 && IR_D_val[6]==0 && IR_D_val[7]==0 ){
        stop();
        while(1){}
            } */
    int hasDeviate = deviator(IR_D_val);
    if(hasDeviate==1){
      //direction
      int turningTo = turnTo(Temp_IR_D_val);
      if(turningTo == 0){
        ////turnleft
        ///when black detected
        delay(250);
        while (hasDeviate==1){
        TurnLeft();  
        read_IR();
        hasDeviate = deviator(IR_D_val);
        }
        
      }
      else{
        //turn right
        ///when black detected
        delay(250);
        while (hasDeviate==1){
        TurnRight();  
        read_IR();
        hasDeviate = deviator(IR_D_val);
      }
      
    }
    }
    else{
    set_forward();
    PID_control();
    set_speed();
    for(int i=0; i<8; i++){
            Temp_IR_D_val[i] = IR_D_val[i];
          }
    }
  }
}
