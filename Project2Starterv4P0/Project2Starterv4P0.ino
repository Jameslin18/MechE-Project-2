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
int rspeed = 50;
int lspeed = 50;
bool dir = false; //true: fowards, false: backwards

//Docking and speed control
int dockDist = 3; //Distance to dock at
int speed;
int speedMulti;
int wallDist = 8.7; //Target distance from wall

//wall following
float wallDistance3=17; //don't know yet

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


/********************************Loop - yours to edit!****************************************************************************/
//Below is some skeleton code that calls functions.  Your primary task is to edit this code to accomplish your goals.
void loop() {
    switch (stage) {
      case(1):
      lineFollowExecution();
      break;
      case(2):
      dockSpeedController();
      break;
      case(3):
      myservo.write(180);
      wallFollowController();
      //steer method
      break;
      default:
      exit(2);
    }
}

//-------------------------------Stage 1--------------------------------------------------------------------------------------




float lineFollowController(){
  float w_old_dir = 0;
  float w_old = abs(w_old_dir);        //magnitude of old angular velocity

  int t = 5;         //time between adjustments
  
  int k1 = 3;        //multiplier values
  int k2 = 6;
  int k3 = 10;

  int TapeOn = 0;        //value when senor reads tape
  int TapeOff = 1;       //value when senor doesn't read tape
  
  int onTapeLeft = analogRead(LL);        //read from sensors
  int onTapeMiddle = analogRead(LM);
  int onTapeRight = analogRead(LR);

  /*Serial.print("Left Line Sensor: ");        //print sensor readings
  Serial.println(onTapeLeft);
  Serial.print("Right Line Middle: ");
  Serial.println(onTapeMiddle);
  Serial.print("Right Line Sensor: ");
  Serial.println(onTapeRight);*/


//  if(onTapeLeft == TapeOn, onTapeMiddle == TapeOn, onTapeRight == TapeOn){                    //all three sensors detect line -> don't turn
//    float w = 0;
//    return w;
//  }
  if(onTapeLeft == TapeOff, onTapeMiddle == TapeOn, onTapeRight == TapeOff){                    //middle sensor detect line -> don't turn
    float w = 0;
    float w_old_dir = 1;          //save previous angular velocity
    delay(t);
    return w;
  }
  if(onTapeLeft == TapeOn, onTapeMiddle == TapeOn, onTapeRight == TapeOff){                     //left + middle sensors detect line -> turn left with k1 multiplier
    float w = k1*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
    
  }
  if(onTapeLeft == TapeOn, onTapeMiddle == TapeOff, onTapeRight == TapeOff){                    //left sensor detect line -> turn left with k2 multiplier
    float w = k2*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
  }
  if(onTapeLeft == TapeOff, onTapeMiddle == TapeOn, onTapeRight == TapeOn){                     //right + middle sensors detect line -> turn left with k1 multiplier
    float w = -k1*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
  }
  if(onTapeLeft == TapeOff, onTapeMiddle == TapeOff, onTapeRight == TapeOn){                    //right sensor detect line -> turn right with k2 multiplier
    float w = -k2*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
  }
  if(onTapeLeft == TapeOff, onTapeMiddle == TapeOff, onTapeRight == TapeOff, w_old_dir < 0){        //no sensor detect line while turning right -> turn left with k3 multiplier
    float w = k3*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
  }
  if(onTapeLeft == TapeOff, onTapeMiddle == TapeOff, onTapeRight == TapeOff, w_old_dir > 0){        //no sensor detect line while turning left -> turn right with k3 multiplier
    float w = -k3*w_old;
    float w_old_dir = w;          //save previous angular velocity
    delay(t);
    return w;
  }
}

void lineFollowExecution(){
  float w = lineFollowController();       //angular velocity from controller function
  
  float Vc = 150;                                 //base speed of robot
  float L = 10;                                   //distance between left and right wheels
  
  float Vr = Vc + 0.5*L*w;                        //speed of right wheels calculated using inverse kinematics
  float Vl = Vc - 0.5*L*w;                        //speed of left wheels calculated using inverse kinematics

  rightMotor(Vr, 1);                              //drive motors
  leftMotor(Vl, 1);
}


//-------------------------------Stage 2--------------------------------------------------------------------------------------

void dockSpeedController() { //assuming speed slow at 20cm
    while (Distance_test() > dockDist) {
      speed = Distance_test() / 20 * 200;
      leftMotor(speed, dir);
      rightMotor(speed, dir);
    }
    dir = !dir;
    stage = 3;
}

//-------------------------------Stage 3--------------------------------------------------------------------------------------


float wallSpeedController() {
    speedMulti = abs(Distance_test() - wallDist) / wallDist;
    return speedMulti;
}

void wallFollowController()
{
  float error=(Distance_test() - wallDistance3);
  Serial.println(error);
  if (error>0)
  {
    leftMotor(lspeed*2.15,dir);
    rightMotor(rspeed,dir);
  }
  if (error<0)
  {
    leftMotor(lspeed,dir);
    rightMotor(rspeed*2.15,dir);
  }
  }
  //leftMotor(50,dir);
  //rightMotor(50,dir);


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
