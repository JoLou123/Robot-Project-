#include<SoftwareSerial.h>
#include <AFMotor.h>

int pulses;                              //Output pulses.
int deg = 0;
int encoderA = 18;
int encoderB = 19;
int pulsesChanged = 0;
int total = 1450;                        //x1 pulses per rotation.
int finishTurn = 0;

AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // create motor #4, 1KHz pwm

void setup(){
  Serial.begin(9600);
  
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT); 
  attachInterrupt(digitalPinToInterrupt(encoderA), A_CHANGE, CHANGE);


}//setup

void turningAngle(int degree) {
  int numPulse = degree*1450;
  if(degree > 0) {
    while(pulses < numPulse) {
      motor3.run(FORWARD);
      motor3.setSpeed(255);
      motor2.run(FORWARD);
      motor2.setSpeed(255);

      motor1.run(FORWARD);
      motor1.setSpeed(255);
      motor4.run(FORWARD);
      motor4.setSpeed(255);
    }
    pulses = 0;
    finishTurn = 1;
  }
}

void loop(){
  
//    if(pulses >= total) {
//      motor3.setSpeed(0);
//    } else {
//      motor3.run(FORWARD);
//      motor3.setSpeed(255);
//      Serial.println(pulses);
//    }
    if(finishTurn == 0){
      turningAngle(180);
    } else {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }
    
}

void A_CHANGE(){                                  //Function that to read the pulses in x1.
  if( digitalRead(encoderB) == 0 ) {
    if ( digitalRead(encoderA) == 0 ) {
      // A fell, B is low
      pulses--; // moving reverse
    } else {
      // A rose, B is low
      pulses++; // moving forward
    }
  }
  pulsesChanged = 1;
}
