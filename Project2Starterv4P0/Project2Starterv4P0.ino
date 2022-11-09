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

class dualOut{          //allows me to have a method return multiple values
  public:
    float main;
    bool dir;
    //int num //1 for right, 0 for left
};

class dualOut lineFollowController(){
  dualOut output;

  int t = 0;         //time between adjustments
  
  float k1 = 0.5;        //multiplier values
  float k2 = 0.6;
  
  int onTapeLeft = analogRead(LL);        //read from sensors
  int onTapeMiddle = analogRead(LM);
  int onTapeRight = analogRead(LR);
  
  bool sensorLeft = sensorCondition(onTapeLeft, 1);         //turns sensor reading into bool depending on whether it detects tape
  bool sensorMiddle = sensorCondition(onTapeMiddle, 2);
  bool sensorRight = sensorCondition(onTapeRight, 3);

  Serial.print("Left Line Sensor: ");        //print sensor readings and their corresponding bool values
  Serial.print(onTapeLeft);
  Serial.print(" = ");
  Serial.println(sensorLeft);
  Serial.print("Middle Line Sensor: ");
  Serial.print(onTapeMiddle);
  Serial.print(" = ");
  Serial.println(sensorMiddle);
  Serial.print("Right Line Sensor: ");
  Serial.print(onTapeRight);
  Serial.print(" = ");
  Serial.println(sensorRight);

//if(sensorLeft == true, sensorMiddle == true, sensorRight == true){                    //all sensor detect line -> don't turn
//    output.main = 0.0;
//    output.dir = true;
//    delay(t);
//    return output;
//  }  
  if(sensorLeft == false, sensorMiddle == true, sensorRight == false){                    //middle sensor detect line -> don't turn
    output.main = 0.0;
    output.dir = true;
    delay(t);
    return output;
  }
  if(sensorLeft == true and sensorMiddle == true and sensorRight == false){                     //left + middle sensors detect line -> turn left with k1 multiplier
    output.main = k1;
    output.dir = true;
    delay(t);
    return output;
    
  }
  if(sensorLeft == true and sensorMiddle == false and sensorRight == false){                    //left sensor detect line -> turn left with k2 multiplier
    output.main = k2;
    output.dir = true;
    delay(t);
    return output;
  }
  if(sensorLeft == false and sensorMiddle == true and sensorRight == true){                     //right + middle sensors detect line -> turn left with k1 multiplier
    output.main = -k1;
    output.dir = true;
    delay(t);
    return output;
  }
  if(sensorLeft == false and sensorMiddle == false and sensorRight == true){                    //right sensor detect line -> turn right with k2 multiplier
    output.main = -k2;
    output.dir = 1;
    delay(t);
    return output;
  }
  if(sensorLeft == false and sensorMiddle == false and sensorRight == false){        //no sensor detect line while turning right -> turn left with k3 multiplier
    output.main = 0.0;
    output.dir = false;
    delay(t);
    return output;
  }
/*  else{                                                                               //middle sensor detect line -> don't turn
    while(sensorLeft==false and sensorMiddle==true and sensorLeft==false)
    {
      leftMotor(150,1);
      rightMotor(150,1);
      onTapeLeft = analogRead(LL);        //read from sensors
      onTapeMiddle = analogRead(LM);
      onTapeRight = analogRead(LR);
  
      sensorLeft = sensorCondition(onTapeLeft, 1);         //turns sensor reading into true or false depending on whether it detects tape
      sensorMiddle = sensorCondition(onTapeMiddle, 2);
      sensorRight = sensorCondition(onTapeRight, 3);
    }
    
    output.main = 0.0;
    output.dir = true;
    delay(t);
    return output;

  }

  sensorLeft_prev=sensorLeft;
  sensorRight_prev=sensorRight;
  sensorMiddle_prev=sensorMiddle;
  */
}

void lineFollowExecution(){
  dualOut exec;
  exec = lineFollowController();       //outputs from controller function

  float w = exec.main;                //angular velocity
  bool dir = exec.dir;                //dirction of travel
  Serial.print("w = ");                    //prints angular velocity
  Serial.println(w);
  Serial.print("dir = ");                    //prints direction of travel
  Serial.println(dir);

  float dist = Distance_test();
  
  float Vc = 150.0;                                 //base speed of robot
  float L = 300.0;                                  //distance between left and right wheels, verified by tuning value until it spun at 2pi rad/s
  
  float Vr = Vc + 0.5*L*w;                        //speed of right wheels calculated using inverse kinematics
  float Vl = Vc - 0.5*L*w;                        //speed of left wheels calculated using inverse kinematics

  if (Vr < 0){                                    //limits motor output between 0-255
    Vr = 0;
  }
  if (Vr > 255){
    Vl = 255;
  }
  if (Vl < 0){
    Vl = 0;
  }
  if (Vl > 255){
    Vl = 255;
  }
  if (dir == false){                          //dampens reverse speed
    Vr = 0.6*Vr;
    Vl = 0.6*Vl;
  }
  while (w == 0 and dir == true){                 //drive motors continuously when going straight forward
    rightMotor(Vr, dir);                              
    leftMotor(Vl, dir);
  }
  
  if(dist <= 25.0 and dist > 16){              //slows down when approach train
    float mult = dist / 100;
    Vc = mult  * Vc;
  }
  if(dist <= 16){                             //goes to wall follow after getting train
  Serial.print(dist);
  Serial.print("Distance = ");
  Serial.println(dist);
  rightMotor(25, 1);                              
  leftMotor(25, 1);   
                                
  delay(500);
  stage = 3;
  }

  rightMotor(Vr, dir);                              //drive motors
  leftMotor(Vl, dir);
  
  delay(120);

  rightMotor(0, dir);                              //stop motors
  leftMotor(0, dir);

  Serial.print("Vr = ");                          //prints power given to motors
  Serial.println(Vr);
  Serial.print("Vl = ");
  Serial.println(Vl);
}

void spinAround(){
  float w = 6.28;       //angular velocity from controller function
  
  Serial.print("w = ");                    //prints angular velocity
  Serial.println(w);
  
  float L = 300;                                   //distance between left and right wheels
  
  float Vc = 100; 
  
  float Vr = 0.5*L*w;                        //speed of right wheels calculated using inverse kinematics
  float Vl = 0.5*L*w;                        //speed of left wheels calculated using inverse kinematics

  rightMotor(Vr, 1);                              //drive motors
  leftMotor(Vl, 0);
}

void backDrive(){
  int Vc = 150;

  rightMotor(Vc, 0);                              //drive motors
//  leftMotor(Vc, 0);
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
  /*
  float sensorDist = Distance_test();
  float speedMulti = wallSpeedController();
  float speedDecrease = 1-speedMulti;
  float errorMargin = 0.5/15.0;
  float k = 2.0;
  int netSpeed = 100;

  float rightspeed = netSpeed;
  float leftspeed = 1.5*netSpeed;
  
  if (speedMulti <= errorMargin && speedMulti >= -errorMargin){
    Serial.println("mode 1");
    leftMotor(netSpeed,0);
    rightMotor(netSpeed,0);
  }
  if(speedMulti > errorMargin){
    Serial.println("mode 2");
    float rightspeed = checkMax(rightspeed * k *(1-speedMulti));
    float leftspeed = checkMax(netSpeed * k *(1+speedMulti));
  }
  if(speedMulti < errorMargin){
    Serial.println("mode 3");
    float rightspeed = checkMax(rightspeed * k *(1-speedMulti));
    float leftspeed = checkMax(leftspeed * k *(1+speedMulti));
  }
  if (sensorDist >= 25){
    myservo.write(90);
//    leftMotor(50,1);
//    rightMotor(50,1);
//    delay(2000);
    stage = 0;
  }  
  
  leftMotor(leftspeed,0);
  rightMotor(rightspeed,0);
    
  Serial.print("leftspeed = ");
  Serial.println(leftspeed);
  Serial.print("rightspeed = ");
  Serial.println(rightspeed);

  Serial.print("control = ");
  Serial.println(speedMulti);
  
  Serial.print("Wall Distance = ");
  Serial.println(sensorDist);
  delay(50);
  */
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
      lineFollowExecution();
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
