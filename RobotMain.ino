

// uncomment to show debug messages in Serial Monitor
//#define __DEBUG 

#include "Motor.h"
#include <Servo.h>


// our pins
const int TRIG_PIN = A4;
const int ECHO_PIN = A5; 
const int IR_PIN = 3;
const int SERVO_PIN = 9;

// MOTOR CONFIGURATION
// The Rover robot has a 2:1 gear ratio and the right motor is 
// mounted backwars and needs to be inverted
Motor leftMotor = Motor(4, 5, 6, 7, false, 2.f);
Motor rightMotor = Motor(A0, A1, A2, A3, true, 2.f);
const int BASESPEED = 450;

// OUR SERVO
Servo sensorServo;
int FRONTANGLE = 90;
int LEFTANGLE = 150;
int RIGHTANGLE = 30;

// various states
enum State { START, MOVING, CHECK_TURN, TURN_LEFT, TURN_RIGHT, STUCK, REMOTE_CONTROLLED, FINISH, EXIT};
volatile State state = START;

// remote buttons
enum remote {
  Power, VOLup, STOP,

  Rewind = 4, PlayOrPause, Fastforward,

  Down = 8, VOLdown, Up,

  zero = 12, EQ, ST,

  one = 16, two, three,

  four = 20, five, six,

  seven = 24, eight, nine};
    


// function declarations for movements
void moveForward(float distance = 1.f);
void followLeftWall(long distLeft);
void followRightWall(long distRight);
void leftTurn();
void rightTurn();
void checkForTurn();
void stopAll();

// logic functions to proceed
float readDistance();
float readLeftUltrasonic();
float readFrontUltrasonic();
float readRightUltrasonic();

// set up timer function delclaration
uint32_t last_control = 0;
const uint32_t CONTROL_INTERVAL = 500; // in milliseconds
void noBlockWait(uint32_t duration = 250);


void setup() {

#ifdef __DEBUG
  Serial.begin(115200);
#endif

  sensorServo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(1000);

}

void loop() {

  if (millis() - last_control >= CONTROL_INTERVAL) {
    last_control = millis();
    controlLoop();
  }
  // updating the motors every cycle
  leftMotor.runSpeedToPosition();
  rightMotor.runSpeedToPosition();

}


// control loop
void controlLoop(){
  
  
  long distFront = readFrontUltrasonic();

  
  switch(state) {

    case START:
        moveForward();
        state = MOVING;
        break;

    case MOVING:
        if (distFront < 40) {
          state = CHECK_TURN;
        }
        else{
        float distLeft = readLeftUltrasonic();
          if (distLeft < 30) {
            followLeftWall(distLeft);
          }
          else {float distRight = readRightUltrasonic(); 
            if (distRight < 30) {
                followRightWall(distRight);
            }
          }
            //implement some method to create a scale to get it to the right distance. and not crash into wall.
            moveForward(.2f);
        }
        break;

    case CHECK_TURN:
        if (readLeftUltrasonic() > 60) {
            state = TURN_LEFT;
            break;
        }
        else if (readRightUltrasonic() > 60) {
            state = TURN_RIGHT;
            break;
        }
        else if (distFront == 0)
        {
            stopAll();
            state = FINISH;
            break;

        }
        else state = STUCK;
        break;

    case TURN_LEFT:
        leftTurn();
        moveForward();
        state = MOVING;
        break;

    case TURN_RIGHT:
        rightTurn();
        moveForward();
        state = MOVING;
        break;
    
    case STUCK:
        // could implement some escape maneuvers here
        stopAll();
        break;
    case FINISH:
        victoryDance();
        state = EXIT;
        break;

    case REMOTE_CONTROLLED:
        // to be implemented
        break;


    case EXIT:
        stopAll();
  }
}


// moving forward
void moveForward(float distance = 1.f) {

  leftMotor.forward(BASESPEED, distance);
  rightMotor.forward(BASESPEED, distance);
}

// turning logic 
// the left turn
void leftTurn(){
  leftMotor.reverse(250, 0.12);
  rightMotor.forward(250, 0.16);

  // waiting for the turn to complete
  while (leftMotor.distanceToGo() != 0 || rightMotor.distanceToGo() != 0) {
    leftMotor.runSpeedToPosition();
    rightMotor.runSpeedToPosition();
  } 
}
// the right turn
void rightTurn(){
  leftMotor.forward(250, 0.16);
  rightMotor.reverse(250, 0.12);

// waiting for the turn to complete
  while (leftMotor.distanceToGo() != 0 || rightMotor.distanceToGo() != 0) {
    leftMotor.runSpeedToPosition();
    rightMotor.runSpeedToPosition();
  } 
}
// centering with left wall.
void followLeftWall(long distLeft){

  const int distFromWall = 35;
  signed int error = distFromWall - distLeft;

  signed int turn = error*100;

  leftMotor.setSpeed(BASESPEED - turn);
  rightMotor.setSpeed(BASESPEED + turn);
}
// centering with right wall.
void followRightWall(long distRight){

  const int distFromWall = 35;
  signed int error = distFromWall - distRight;

  signed int turn = error*100;

  leftMotor.setSpeed(BASESPEED + turn);
  rightMotor.setSpeed(BASESPEED - turn);
}

// stopping motors
void stopAll() {

  leftMotor.stop();
  rightMotor.stop();
}

// ultrasonic read
float readDistance(){
  digitalWrite(TRIG_PIN, LOW); // clear Trig pin
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); // send signal
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW); // clear trig pin
  long duration = pulseIn(ECHO_PIN, HIGH, 400*29.1*2); // read echo pin
  float distance = duration * 0.034 / 2; // calculate distance in cm

  return distance;
}

// scanning front, right, & left functions
float readLeftUltrasonic() {
    sensorServo.write(LEFTANGLE);
    noBlockWait(); // non-blocking/ allow motors to run while waiting
   int distance = readDistance();
    sensorServo.write(FRONTANGLE); // return to front
    return distance;
}
float readFrontUltrasonic() {
    sensorServo.write(FRONTANGLE);
    noBlockWait(); // non-blocking/ allow motors to run while waiting
    return readDistance();
}
float readRightUltrasonic() {
    sensorServo.write(RIGHTANGLE);
    noBlockWait(); // non-blocking/ allow motors to run while waiting
    int distance = readDistance();
    sensorServo.write(FRONTANGLE); // return to front
    return distance;
    }
    
// non-blocking wait function to allow motors to run while waiting
void noBlockWait(uint32_t duration) {
    uint32_t start = millis();
    while (millis() - start < duration) {
        leftMotor.runSpeedToPosition();
        rightMotor.runSpeedToPosition();
    }
}

void victoryDance(){
  int dance = 10;
  leftMotor.forward(500, 2.f);
  rightMotor.reverse(500, 2.f);
  while (dance) {
    noBlockWait();
    readLeftUltrasonic();
    readRightUltrasonic();
    dance--;
  }
}


// IR Remote Use