#include <AFMotor.h>  
AF_DCMotor motor1(1, MOTOR12_64KHZ); // back left
AF_DCMotor motor2(2, MOTOR12_64KHZ); // back right
AF_DCMotor motor3(3, MOTOR34_1KHZ); // front right
AF_DCMotor motor4(4, MOTOR34_1KHZ); // front left

int tiltPin = 26;         // the number of the input pin
 
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
 
// the following variables are long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounceTime = 500;   // the debounce time, increase if the output flickers

int leftLimit = 25;
int left = 0; //left limit switch read value
int bottomReached = 0;

void setup()
{
  pinMode(tiltPin, INPUT);
  digitalWrite(tiltPin, HIGH);
  pinMode(leftLimit, INPUT);
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);
  Serial.begin(9600);
}

void traverseWall() {
    int switchstate;
 
  reading = digitalRead(tiltPin);
 
  // If the switch changed, due to bounce or pressing...
  if (reading != previous) {
    // reset the debouncing timer
    time = millis();
  } 
 
  if ((millis() - time) > debounceTime) {
     // whatever the switch is at, its been there for a long time
     // so lets settle on it!
     switchstate = reading;
 
  }
  
  left = digitalRead(leftLimit);
  if(left == 1) {
    bottomReached = 1;
  }
    
  Serial.println(switchstate);
  if(switchstate == 0 && bottomReached == 0) {
    motor1.run(FORWARD);
    motor1.setSpeed(255);  

    motor2.run(FORWARD);
    motor2.setSpeed(255);

 
    motor3.run(FORWARD);
    motor3.setSpeed(255);

    motor4.run(FORWARD);
    motor4.setSpeed(255);
  } else if(switchstate == 1 && bottomReached == 0) {
    motor1.run(FORWARD);
    motor1.setSpeed(255);  

    motor2.run(FORWARD);
    motor2.setSpeed(255);
 
    motor3.run(FORWARD);
    motor3.setSpeed(255);

    motor4.run(FORWARD);
    motor4.setSpeed(255);
  } else { 
      motor1.run(FORWARD);
      motor1.setSpeed(255);  
  
      motor2.run(FORWARD);
      motor2.setSpeed(255);
  
   
      motor3.run(FORWARD);
      motor3.setSpeed(255);
  
      motor4.run(FORWARD);
      motor4.setSpeed(255);
  }
  
  // Save the last reading so we keep a running tally
  previous = reading;
  delay(100);
}
 
void loop()
{
  //left = digitalRead(leftLimit);
  //Serial.println(left);
  traverseWall();
//      motor1.run(FORWARD);
//      motor1.setSpeed(255);  
//  
//      motor2.run(FORWARD);
//      motor2.setSpeed(255);
//  
//   
//      motor3.run(FORWARD);
//      motor3.setSpeed(255);
//  
//      motor4.run(FORWARD);
//      motor4.setSpeed(255);
//  
}
