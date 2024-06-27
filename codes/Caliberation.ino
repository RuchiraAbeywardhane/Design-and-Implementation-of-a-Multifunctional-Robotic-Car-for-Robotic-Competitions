
//Calibeartion Arrays
int caliberation_threshold[8];
int caliberation_data[8][10];

void setup() {
  // put your setup code here, to run once:
  const int caliberation_pin = 6;
}


//caliberation Function
void caliberation(){
    int caliberationData[8];
    for (int i = 0; i < 9; i++) {
      for(int j = 0; j < 7; j++){
      caliberationData[j].append(analogRead(tcrtPins[j]));
      }
      //Rotate the Car for callibeartion
      
    }
}

void loop() {
  // put your main code here, to run repeatedly:

}
