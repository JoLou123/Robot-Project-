#include "AFMotor.h"

#define MAX_MOTOR_SPEED (255)

int inPin = 26;         // the number of the input pin

int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the following variables are long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 50;   // the debounce time, increase if the output flickers

AF_DCMotor motor1(1, MOTOR12_64KHZ); // Create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // Create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // Create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Create motor #4, 1KHz pwm

void setup()
{
  pinMode(inPin, INPUT);
  digitalWrite(inPin, HIGH); // Internal pull-up
  Serial.begin(9600);
}
 
void loop()
{
  int switchstate;
 
  reading = digitalRead(inPin);
 
  // If the switch changed, due to bounce or pressing...
  if (reading != previous) {
    // reset the debouncing timer
    time = millis();
  } 
 
  if ((millis() - time) > debounce) {
     // whatever the switch is at, its been there for a long time
     // so lets settle on it!
     switchstate = reading;
 
     // Now invert the output on the pin13 LED
    if (switchstate == LOW)
    {
      motor1.run(FORWARD);
      motor1.setSpeed((int)(MAX_MOTOR_SPEED));
      motor4.run(FORWARD);
      motor4.setSpeed((int)(MAX_MOTOR_SPEED));
      motor2.run(FORWARD);
      motor2.setSpeed((int)(MAX_MOTOR_SPEED));
      motor3.run(FORWARD);
      motor3.setSpeed((int)(MAX_MOTOR_SPEED));
    }
    else
    {
      motor1.run(FORWARD);
      motor1.setSpeed((int)(0.5 * MAX_MOTOR_SPEED));
      motor4.run(FORWARD);
      motor4.setSpeed((int)(0.5 * MAX_MOTOR_SPEED));
      motor2.run(FORWARD);
      motor2.setSpeed((int)(0.5 * MAX_MOTOR_SPEED));
      motor3.run(FORWARD);
      motor3.setSpeed((int)(0.5 * MAX_MOTOR_SPEED));
    }
  }
  
  Serial.println(reading);
 
  // Save the last reading so we keep a running tally
  previous = reading;
  delay(500);
}
