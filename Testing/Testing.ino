/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <AFMotor.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability


int motor3A = 18;
int motor3B = 19; 



Encoder myEnc(motor3A, motor3B);  
//   avoid using pins with LEDs attached
AF_DCMotor motor3(3, MOTOR34_1KHZ); // create motor #3, 1KHz pwm

void setup() {
  Serial.begin(9600);
  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);

  digitalWrite(motor3A, HIGH);
  digitalWrite(motor3B, HIGH);
  attachInterrupt(digitalPinToInterrupt(motor3A), increment, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(motor3B), increment, CHANGE);

  
  Serial.println("Basic Encoder Test:");
}

//long oldPosition  = -999;
void increment() {
//    long newPosition = myEnc.read();
//  if (newPosition != oldPosition) {
//    oldPosition = newPosition;
//    Serial.println(newPosition);
//  } 
   
}

void loop() {

  motor3.run(FORWARD);
  motor3.setSpeed(180);  
      
}
