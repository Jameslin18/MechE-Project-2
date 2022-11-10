//This code is based on starter code provided by Elegoo
//www.elegoo.com

//instructions on how to run correctly: 1. connect wire to audrino 2.choose your port (audrino uno) 3.click the upload arrow button
//note: click the check mark to compile your code and see if it works

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

//Define all of the pins used.  These are NOT up to us, but rather what Elegoo decided.  Don't change.
int Echo = 12;  
int Trig = 13; 

#define ENA 5
#define ENB 6
#define DIRA 8
#define DIRB 7
#define EN 3
//Line follower pins
#define LR A2
#define LM A1
#define LL A0

int stage = 1;
int rspeed = 80;
int lspeed = 80;
bool dir = false; //true: fowards, false: backwards
int counter = 0;
int baseSpeed = 175;
int L;
int R;
int M;
int corrA = 1.1;
int corrS;

//Docking and speed control
int dockDist = 3; //Distance to dock at
int speed;
int speedMulti;
int wallDist = 8.7; //Target distance from wall
bool sensorLeft_prev=false;
bool sensorRight_prev=false;
bool sensorMiddle_prev=false;

/******************************Helper functions*********************************************/
//Begin helper functions.  You should CALL these functions, but do not change them.  You DO NOT need to worry about the details inside the functions.

//The functions below set the left and right motor speeds and directions.
//Speed is an integer from 0 - 255 that determines the motor speed.
//Direction is 1 for forward and 0 for backwards.
void leftMotor(int lspeed, bool ldirection){
  analogWrite(ENA, lspeed);
  if (ldirection)
  {
    digitalWrite(DIRA, HIGH);
  }
  else
  {
    digitalWrite(DIRA, LOW);
  }
}
void rightMotor(int rspeed, bool rdirection){
  analogWrite(ENB, rspeed);
  if (rdirection)
  {
    digitalWrite(DIRB, HIGH);
  }
  else
  {
    digitalWrite(DIRB, LOW);
  }
}
//  The function below stops.  This is the same as just setting motor speeds to zero - not really necessary
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
} 

//Ultrasonic distance measurement helper function - returns a float with distance in cm
float Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58.0;       
  return Fdistance;
}  

//checks if the motor speed is greater than 255, if it is, it would return 255
float checkMax(float speed)
{
  if (speed>255)
  {
    return (255.0);
  }
  return (speed);  
  }

/*************************Setup*************************************************/
//You shouldn't need to touch this - it is merely setting up pins and stopping the motors at the start
void setup() { 
  myservo.attach(10,500,2400);  // attach servo on pin 10 to servo object
  Serial.begin(9600);     //For debugging
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,HIGH);
  stop();
} 


//-------------------------------Stage 1--------------------------------------------------------------------------------------
void distanceRead(){
  float value = Distance_test();
  Serial.print("Distance = ");
  Serial.println(value);
  delay(1000);
}

void sensorRead(){
  int onTapeLeft = analogRead(LR);        //read from sensors
  int onTapeMiddle = analogRead(LM);
  int onTapeRight = analogRead(LL);

  Serial.print("Left Line Sensor: ");        //print sensor readings
  Serial.println(onTapeLeft);
  Serial.print("Middle Line Sensor: ");
  Serial.println(onTapeMiddle);
  Serial.print("Right Line Sensor: ");
  Serial.println(onTapeRight);
  delay(0);
}

bool sensorCondition(int sensorInp, int sensorNum){ // This outputs a bool based on whether the sensor is reading the line. It makes the controller code easier to write and understand
                                                    //sensorNum: 1 = Left, 2 = Middle, 3 = Right : orgininally anticipated each sensor having different thresholds, but turns out 400 works for all three
  switch (sensorNum){
    case(1):
    if (sensorInp >= 400){       //values aquired from testing
      bool condition = true;
      return condition;
    }
    else{
      bool condition = false;
      return condition;
    }
    case(2):
    if (sensorInp >= 400){       //values aquired from testing
      bool condition = true;
      return condition;
    }
    else{
      bool condition = false;
      return condition;
    }
    case(3):
    if (sensorInp >= 400){       //values aquired from testing
      bool condition = true;
      return condition;
    }
    else{
      bool condition = false;
      return condition;
    }
  }
}

void lineFollowExecution2(){
  float dist = Distance_test();
  
  L = analogRead(LL);
  R = analogRead(LR);
  M = analogRead(LM);
  corrS = (counter + baseSpeed) * corrA;
  if (corrS >= 255)
    corrS = 255;
  if (L > 500) {
    rightMotor(corrS, 1);
    leftMotor(0, 1);
    counter++;
  } else if (R > 500) {
    leftMotor(corrS, 1);
    rightMotor(0, 1);
    counter++;
  } else if (M > 500) {
    counter = 0;
    leftMotor(baseSpeed, 1);
    rightMotor(baseSpeed, 1);
  }
  Serial.print("Dist = ");
  Serial.println(dist); 
  if (dist <= 15) {                       //goes to wall follow after getting train
    rightMotor(25, 1);                              
    leftMotor(25, 1);                              
    delay(500);
    stage = 3;
  }
}                           
//-------------------------------Stage 2--------------------------------------------------------------------------------------

void dockSpeedController() { //assuming speed slow at 25cm
    while (Distance_test() > dockDist) {
      speed = Distance_test() / 25 * 200;
      leftMotor(speed, dir);
      rightMotor(speed, dir);
    }
    dir = !dir;
    stage = 3;
}

//-------------------------------Stage 3--------------------------------------------------------------------------------------


float wallSpeedController() {
  
  float wallDistance3 = 15.0;                                              //wall following distance
  float speedMulti = (Distance_test() - wallDistance3) / wallDistance3;
  return speedMulti;
}


void wallFollowController(){                                            //robot is hard coded to turn a little bit to straighten itself then go backwards until it is in the endzone
  Serial.println("stage 3 running");
  leftMotor(100,1);
  rightMotor(100,0);
  delay(100);
  leftMotor(250,0);
  rightMotor(250,0);
  delay(1000);
  stage=0;
}

/********************************Loop - yours to edit!****************************************************************************/
//Below is some skeleton code that calls functions.  Your primary task is to edit this code to accomplish your goals.
void loop() {
    switch (stage) {
      case(0):
      leftMotor(0,1);
      rightMotor(0,1);
      break;
      case(1):
//      backDrive();
//      spinAround();
//      sensorRead();
//      distanceRead();
      lineFollowExecution2();
      break;
      case(2):
      dockSpeedController();
      break;
      case(3):
      myservo.write(180);
      delay(750);
//      distanceRead();
      wallFollowController();
      //steer method
      break;
      default:
      exit(2);
    }
}


//-------------------------------Tools--------------------------------------------------------------------------------------
// //Here is how you set the servo angle
// myservo.write(90);  //setservo position to angle; 90 is nominally straight in front
// delay(500); //Each time you change the servo, you should give it some time to reach the destination

// //Here is how you get the distance in cm
// float distance = Distance_test();  

// //Here is how you drive your car - this sample drives the car forward
// int rspeed = 200;
// int lspeed = 200;
// leftMotor(lspeed, dir);  //Replace 1 with zero to reverse the direction for either motor
// rightMotor(rspeed, dir);

// //Here is how you tell if the line following sensor is seeing the black tape
// //Check the values you read when on tape and off to see how to interpret these readings!
// int onTapeLeft = analogRead(LL);
// int onTapeRight = analogRead(LR);
// int onTapeMiddle = analogRead(LM);


// //Report variables back for your sanity
// Serial.print("Distance Reading: ");
// Serial.print(distance);
// Serial.println(" cm");
// Serial.print("Left Line Sensor: ");
// Serial.println(onTapeLeft);
// Serial.print("Right Line Sensor: ");
// Serial.println(onTapeRight);
// Serial.print("Right Line Middle: ");
// Serial.println(onTapeMiddle);