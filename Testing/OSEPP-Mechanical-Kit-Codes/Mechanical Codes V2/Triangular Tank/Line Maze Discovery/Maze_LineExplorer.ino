/*
 Maze_LineExplorer

 Created 25 Dec. 2016
 Modified 7 Jan. 2017
 by kevin@ossep.com

*/

#include <Arduino.h>

//Note that REGULAR power status ONLY, these values on speed with delay.
//Make a selection on two speed modes with delay.
const int SPEED = 150; //fast speed mode
const int SPEED_TURN = 200;
const int LINE_DIFF_DELAY = 30;
const int TURN_LEFT_DELAY = 120;
const int TURN_RIGHT_DELAY = 220;

/*
const int SPEED = 100; //slow speed mode
const int SPEED_TURN = 200;
const int LINE_DIFF_DELAY = 30;
const int TURN_LEFT_DELAY = 220;
const int TURN_RIGHT_DELAY = 400;
*/

//DO NOT change these configuration until you really understand them.
const int leftMotorDirPin = 12;
const int leftMotorPwmPin = 11;
const int rightMotorDirPin = 8;
const int rightMotorPwmPin = 3;
const int TARGETNODE_DELAY = 90;
const int PLACE_CENTER_DELAY = 120;
const int SPEED_MAX = 500;
const int BLACKDIFF = 400;
const byte LINENODE = 0;
const byte DIFFLINENODE = 1;
const byte LEFTLINENODE = 2;
const byte RIGHTLINENODE = 3;
const byte LEAFNODE = 4;
const byte TARGETNODE = 5;
volatile int nState0;
volatile int nState1;
volatile int nState2;
volatile int nState3;
volatile int nState4;
volatile byte nodeState;
volatile byte tempState;
volatile bool pendingState;

void setup() {
	pinMode(A0, INPUT_PULLUP);
	pinMode(A1, INPUT_PULLUP);
	pinMode(A2, INPUT_PULLUP);
	pinMode(A3, INPUT_PULLUP);
	pinMode(A4, INPUT_PULLUP);
	pinMode(leftMotorDirPin, OUTPUT);
	pinMode(rightMotorDirPin, OUTPUT);
	digitalWrite(leftMotorDirPin, HIGH);
	digitalWrite(rightMotorDirPin, LOW);
	digitalWrite(leftMotorPwmPin, LOW);
	digitalWrite(rightMotorPwmPin, LOW);
	delay(100);
	detectLineNode();
	//PCMSK1 = PCMSK1 | bit (PCINT8) | bit (PCINT9) | bit (PCINT10) | bit (PCINT11) | bit (PCINT12);
	PCMSK1 = 0b00011111;
	PCIFR |= bit (PCIF1); // clear pin interrupts
	PCICR |= bit (PCIE1); // enable pin interrupts
	pendingState = false;
	forward();
}

void loop() {
	if(nodeState == TARGETNODE && detectTarget())
		hitTarget();
	else if(pendingState){
		while(nodeState == DIFFLINENODE){
			adjustDiff();
			delay(LINE_DIFF_DELAY);
			nodeState = detectNode();
		}
		if(nodeState == LEAFNODE)
			turnAround(false, false);
		if(nodeState == LEFTLINENODE)
			turnAround(true, true);
		if(nodeState == RIGHTLINENODE)
			turnAround(false, true);
		if(nodeState == LINENODE)
			forward();
		pendingState = false;
	}
}
void adjustDiff(){
	if(nState1 < BLACKDIFF){
		analogWrite(leftMotorPwmPin, SPEED_MAX);
		analogWrite(rightMotorPwmPin, SPEED);
	}
	if(nState3 < BLACKDIFF){
		analogWrite(leftMotorPwmPin, SPEED);
		analogWrite(rightMotorPwmPin, SPEED_MAX);		
	}
}
void forward(){
	digitalWrite(leftMotorDirPin, HIGH);
	digitalWrite(rightMotorDirPin, LOW);
	analogWrite(leftMotorPwmPin, SPEED);
	analogWrite(rightMotorPwmPin, SPEED);
}
void turnAround(bool hasLeft, bool hasDelay){
	//begin to position center
	forward();
	
	if(hasLeft){
		if(hasDelay)
			delay(TURN_LEFT_DELAY);
		digitalWrite(leftMotorDirPin, LOW);
		digitalWrite(rightMotorDirPin, LOW);
	}else{
		if(hasDelay)
			delay(TURN_RIGHT_DELAY);
		digitalWrite(leftMotorDirPin, HIGH);
		digitalWrite(rightMotorDirPin, HIGH);		
	}
	analogWrite(leftMotorPwmPin, SPEED_TURN);
	analogWrite(rightMotorPwmPin, SPEED_TURN);
	if(hasDelay)
		delay(PLACE_CENTER_DELAY);
	detectLineNode();
}
void hitTarget(){
	analogWrite(leftMotorPwmPin, LOW);
	analogWrite(rightMotorPwmPin, LOW);
	while(true)
		delay(1000);
}
void detectLineNode(){
	while(true){
		detectLine();
		if(nState1 > BLACKDIFF && nState2 < BLACKDIFF && nState3 > BLACKDIFF){
			nodeState = LINENODE;
			break;			
		}
	}
	delay(LINE_DIFF_DELAY);
}
bool detectTarget(){
	delay(TARGETNODE_DELAY);
	detectLine();
	if(nState1 < BLACKDIFF && nState2 < BLACKDIFF && nState3 < BLACKDIFF)
		return true;
	nodeState = LEFTLINENODE;
	pendingState = true;
	forward();
	return false;
}
void detectLine(){
	nState1 = analogRead(A1);
	nState2 = analogRead(A2);
	nState3 = analogRead(A3);	
}
void detectFullLine(){
	nState0 = analogRead(A0);
	detectLine();
	nState4 = analogRead(A4);	
}
byte detectNode(){
	detectFullLine();
	if(nState1 < BLACKDIFF && nState2 < BLACKDIFF && nState3 < BLACKDIFF && (nState0 < BLACKDIFF || nState4 < BLACKDIFF))
		return TARGETNODE;
	if(nState0 > BLACKDIFF && nState1 > BLACKDIFF && nState2 > BLACKDIFF && nState3 > BLACKDIFF && nState4 > BLACKDIFF)
		return LEAFNODE;
	if(nState4 < BLACKDIFF)
		return LEFTLINENODE;
	if(nState0 > BLACKDIFF && nState1 > BLACKDIFF && nState2 < BLACKDIFF && nState3 > BLACKDIFF && nState4 > BLACKDIFF)
		return LINENODE;
	if(nState0 < BLACKDIFF && nState1 > BLACKDIFF && nState2 > BLACKDIFF && nState3 > BLACKDIFF && nState4 > BLACKDIFF)
		return RIGHTLINENODE;
	if((nState1 > BLACKDIFF && nState2 < BLACKDIFF && nState3 < BLACKDIFF) || (nState1 < BLACKDIFF && nState2 < BLACKDIFF && nState3 > BLACKDIFF))
		return DIFFLINENODE;
	return nodeState;
}
ISR (PCINT1_vect){
	if(pendingState)
		return;
	tempState = detectNode();
	if(tempState == TARGETNODE){
		nodeState = TARGETNODE;
		pendingState = true;
		return;
	}
	if(tempState != nodeState){
		nodeState = tempState;
		pendingState = true;
		return;
	}
}
