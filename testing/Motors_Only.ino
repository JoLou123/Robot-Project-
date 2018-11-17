// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // create motor #4, 1KHz pwm

void setup() {
   Serial.begin (9600);

  // turn on motor
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
}

void loop() {
     
      motor1.run(FORWARD);
      motor1.setSpeed(255);  

      motor2.run(FORWARD);
      motor2.setSpeed(255);

   
      motor3.run(FORWARD);
      motor3.setSpeed(255);

      motor4.run(FORWARD);
      motor4.setSpeed(255);
}