// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
AF_DCMotor motor1(1, MOTOR12_64KHZ); // backright
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // create motor #4, 1KHz pwm
//
int leftLimit = 25;
int rightLimit = 24;
int left = 0; //left limit switch read value
int right = 0; //right limit switch read value


const int trigPin = 22;
const int echoPin = 23;
long duration;
float distance;

void setup()
{
  //Limit Switch + Power
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  pinMode(50, OUTPUT);
  digitalWrite(50, HIGH);

  //Ultrasonic Pins + Power
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);

  //Serial Output read
  Serial.begin(9600);
}

void loop()
{
  //Read limit switch values
  left = digitalRead(leftLimit);
  right = digitalRead(rightLimit);
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin to LOW,HIGH, then LOW state for clean signal
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance = duration*0.034/2;
  
  //demo code
 // if(left == 0 && right == 0 && distance > 5.0){
    motor1.run(FORWARD);
    motor1.setSpeed(255);  

        motor2.run(FORWARD);
    motor2.setSpeed(255); 

        motor3.run(FORWARD);
    motor3.setSpeed(255); 

        motor4.run(FORWARD);
    motor4.setSpeed(255); 
 /* } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }*/
}
//
//void setup() {
//   Serial.begin (9600);
//   
//  pinMode(limitSwitchLeft, INPUT);
//  // turn on motor
// // motor1.setSpeed(255);
//  //motor2.setSpeed(255);
//  //motor3.setSpeed(255);
//  //motor4.setSpeed(255);
//}
//
//void loop() {
//      left = digitalRead(limitSwitchLeft);
//
//    //  while(left == 0) { //1 if pressed, 0 otherwise
//      //    motor1.run(FORWARD);
//        //  motor1.setSpeed(200);  
//    
//         // motor2.run(FORWARD);
//         // motor2.setSpeed(200);
//    
//       
//          //motor3.run(FORWARD);
//          //motor3.setSpeed(200);
//    
//          //motor4.run(FORWARD);
//          //motor4.setSpeed(200);
//     // }
//
//    //  motor1.run(RELEASE);
//      //motor2.run(RELEASE);
//      //motor3.run(RELEASE);
//      //motor4.run(RELEASE);
//
//       Serial.println(left);
//     
//}