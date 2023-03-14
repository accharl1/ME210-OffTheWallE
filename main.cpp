#include <Arduino.h>
#define USE_TIMER_2       true
#include <TimerInterrupt.h>
#include <Servo.h>


/*--------------------------Constants--------------------------*/


#define LINE_THRESHOLD     330
#define BEACON_THRESHOLD   900
#define ORIENT_DUTY        190
#define LINE_DUTY          208
#define SLIGHT_LINE_DUTY   190
#define TURN_180_DUTY      220
#define TIMER_INTERVAL_MS  130000   // 2 min 10 sec


/*-----------------------Pin Definitions-----------------------*/


#define L_IR_sensor        A0    //output from far left IR sensor
#define CL_IR_sensor       A1    //output from center left IR sensor
#define CTR_IR_sensor      A2    //output from center IR sensor
#define CR_IR_sensor       A3    //output from center right IR sensor
#define R_IR_sensor        A4    //output from far right IR sensor




#define R_motor_dir1       2    //right motor direction output 1
#define L_motor_dir1       8    //left motor direction output 1
#define R_motor_dir2       4    //right motor direction output 2
#define L_motor_dir2       7    //left motor direction output 2
#define R_motor_en         5    //right motor enable output
#define L_motor_en         6    //left motor enable output


#define Bad_Button_Pin     11   //
#define Good_Button_Pin    12   //




#define payload_pin        10    //payload delivery servo pin
#define celebration_pin    9    //microservo celebration pin






#define beacon_pin         A5   //information from beacon phototransistor




/*--------------------------Function Declarations--------------------------*/
















void TurnOnIR(int);
void TurnOffIR(int);
int ReadTapeSensor(int);
















int CheckL();
int CheckCL();
int CheckCTR();
int CheckCR();
int CheckR();
int ReadTriggers();
void CheckButtons();
int ReadAll();








bool CheckGameTimer();
void HandleGameOver();
void Celebrate();


void OrientBot();
void GoodLineFollow();
void BadLineFollow();
void GoodHomeLineFollow();
void BadHomeLineFollow();
void OperatePayload();
void OperateMotor();
void Turn180();


void DriveFwd(void);
void DriveRev(void);
void TurnSharpRight(void);
void TurnSharpRight180(void);
void TurnSharpLeft(void);
void TurnOrient(void);
void TurnSlightRight(void);
void TurnSlightLeft(void);
void Stop(void);
void DriveEnterStudio(void);
void DriveSlowFwd(void);
void GoodToBad();


/*-------------------------State Definitions--------------------------*/


typedef enum {
  ORIENT_TURN,
  FORWARD,
  BACKWARD,
  STOP,
  SLIGHT_LEFT,
  SLIGHT_RIGHT,
  SHARP_LEFT,
  SHARP_RIGHT,
  SHARP_RIGHT_180,
  SLOW_FWD
} MotorStates;


typedef enum {
  OPEN,
  CLOSE,
  NOTHING
} PayloadStates;


typedef enum {
  ORIENTING,
  GOING_GOOD,
  GOING_BAD,
  GOING_HOME_GOOD,
  GOING_HOME_BAD,
  AT_PAYLOAD,
  TURNING_180,
  LOADING,
  ENTERING_STUDIO,
  GAME_OVER,
  GOOD_TO_BAD
} DirectionStates;


/*-------------------------Module Variables--------------------------*/


MotorStates motorState;
PayloadStates payloadState;
DirectionStates directionState;


/*bool RMotor;
bool LMotor;
bool RStop;
bool LStop;*/


Servo payloadServo;
Servo celebrateServo;


bool facingBeacon;
unsigned long orientTime;
unsigned long payloadTime;
unsigned long StudioEnterTime;
unsigned long junctionTime;
bool passedJunction;
unsigned long turn180Time;
bool GoodButtonPressed;
bool BadButtonPressed;
unsigned long justGotHere;
bool doneBalling;


/*--------------------------Main Functions--------------------------*/


void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello, world!");


  ITimer2.init();


  // Interval in unsigned long millisecs
  if (ITimer2.attachInterruptInterval(TIMER_INTERVAL_MS, HandleGameOver)){
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  } else {
    Serial.println("Can't set ITimer. Select another freq. or timer");
  }


  motorState = STOP;
  directionState = LOADING; // setup state will eventually have to be LOADING
  payloadState = CLOSE;


  facingBeacon = false;
  orientTime = 0;
  StudioEnterTime = 0;
  payloadTime = 0;
  junctionTime = 0;
  passedJunction = false;


  turn180Time = 0;
  justGotHere = 0;


  GoodButtonPressed = false; // TRUE FOR TESTING PURPOSES ONLY
  BadButtonPressed = false;


  doneBalling = false;


  // IR PIN MODES
  pinMode(L_IR_sensor, INPUT);
  pinMode(CL_IR_sensor, INPUT);
  pinMode(CTR_IR_sensor, INPUT);
  pinMode(CR_IR_sensor, INPUT);
  pinMode(R_IR_sensor, INPUT);


  // MOTOR PIN MODES
  pinMode(R_motor_dir1, OUTPUT);
  pinMode(L_motor_dir1, OUTPUT);
  pinMode(R_motor_dir2, OUTPUT);
  pinMode(L_motor_dir2, OUTPUT);
  pinMode(R_motor_en, OUTPUT);
  pinMode(L_motor_en, OUTPUT);


  // BEACON PIN MODES
  pinMode(beacon_pin, INPUT);


  // PAYLOAD PIN MODES
  pinMode(payload_pin, OUTPUT);
  payloadServo.attach(payload_pin);


  // CELEBRATION PIN MODES
  pinMode(celebration_pin, OUTPUT);
  celebrateServo.attach(celebration_pin);
  payloadServo.write(180);


  // BUTTON PIN MODES
  pinMode(Good_Button_Pin, INPUT);
  pinMode(Bad_Button_Pin, INPUT);


  // celebrate at beginning
  Celebrate();


  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);


}


void loop() {
  // put your main code here, to run repeatedly
 
  // CheckL();  //used for testing one single sensor
  // CheckCL();
  // CheckCTR();
  // CheckCR();
  // CheckR();
  // ReadAll();
  // ReadTriggers();


  // OrientBot();
  // GoodLineFollow();
  // BadLineFollow();


  //OperatePayload();


  // /*
  switch (directionState) {
    case ORIENTING:
      Serial.println("Direction State: ORIENTING");
      OrientBot();
      break;
    case GOING_GOOD:
      Serial.println("Direction State: GOOD");
      GoodLineFollow();
      break;
    case GOING_BAD:
      Serial.println("Direction State: BAD");
      BadLineFollow();
    break;
    case GOING_HOME_GOOD:
      Serial.println("Direction State: HOME GOOD");
      GoodHomeLineFollow();
      break;
    case GOING_HOME_BAD:
      Serial.println("Direction State: HOME BAD");
      BadHomeLineFollow();
      break;
    case AT_PAYLOAD:
      Serial.println("Direction State: PAYLOAD");
      OperatePayload();
      break;
    case TURNING_180:
      Serial.println("Direction State: TURNING 180");
      Turn180();
      break;
    case ENTERING_STUDIO:
      Serial.println("Direction State: ENTERING STUDIO");
      DriveEnterStudio();
      break;
    case LOADING:
      Serial.println("Direction State: LOADING");
      CheckButtons();
      break;
    case GAME_OVER:
      Serial.println("Good Game!");
      break;
    case GOOD_TO_BAD:
      Serial.println("Direction State: GOOD TO BAD");
      GoodToBad();
      break;
  }
  // */


}




/*--------------------------Module Functions--------------------------*/


/*----------Line-Following Functions----------*/


void OrientBot() {
   //Serial.println(analogRead(beacon_pin));
 
  if (analogRead(beacon_pin) > BEACON_THRESHOLD) {
    facingBeacon = true;
  }
 
  if (!facingBeacon)
    motorState = ORIENT_TURN;
  else {
    motorState = SLOW_FWD;


    /*if ((CheckL() || CheckCL()) && !(CheckR() || CheckCR())) {
      motorState = SLIGHT_LEFT;
    } else if ((CheckR() || CheckCR()) && !(CheckL() || CheckCL())) {
      motorState = SLIGHT_RIGHT;
    } else */
    if (CheckCL() && CheckCR()) {
      motorState = STOP;
      OperateMotor();
      delay(250);


      directionState = GOING_GOOD;


    }
   
    //OperateMotor();
  }


  OperateMotor();


  passedJunction = false;
  orientTime = millis();
}


void GoodLineFollow() {
 
  if (CheckCR() || CheckR()) { //!(CheckCL()) &&
    motorState = SHARP_RIGHT;
    Serial.println("SHARP RIGHT");
  } else if (CheckCL() || CheckL()) {
    motorState = SHARP_LEFT;
    Serial.println("SHARP LEFT");
  } else {
    motorState = FORWARD;
    Serial.println("FORWARD");
  } // note that while directionState = GOING_GOOD, motorState should NEVER be BACKWARD, SHARP_RIGHT, or SHARP_LEFT


  if (passedJunction) {
    if (CheckR() && CheckCL()) {
      justGotHere = millis();
      motorState = BACKWARD;
      //OperateMotor();
      payloadState = OPEN;
      directionState = AT_PAYLOAD; // eventually will be AT_PAYLOAD
      Serial.println("AT PAYLOAD");
    }
  } else {
      if ((CheckL() && CheckCL() && !CheckR()) && ((millis() - orientTime) > 1500)) {  //junction case
        Serial.println("ARRIVED AT JUNCTION");
        motorState = FORWARD;
        passedJunction = true;
        junctionTime = millis();
      }
  }


  if (millis() - orientTime < 250) {  // for leaving the studio
    Serial.println("LEAVING STUDIO");
    motorState = FORWARD;
  }


  OperateMotor();
}


void BadLineFollow() {
  Serial.print("passedJunction: "); Serial.println(passedJunction);
 
  if (CheckL() || CheckCL()) {
    Serial.println("SHARP LEFT");
    motorState = SHARP_LEFT;
  } else if (CheckR() || CheckCR()) {
    Serial.println("SHARP RIGHT");
    motorState = SHARP_RIGHT;
  } else {
    Serial.println("FORWARD");
    motorState = FORWARD;
  } // note that while directionState = GOING_BAD, motorState should NEVER be BACKWARD or SHARP RIGHT


  if (passedJunction) {
    if ((millis() - junctionTime < 500)) {
      Serial.println("JUNCTION LEFT");
      motorState = SHARP_LEFT;
    } else if ((millis() - junctionTime < 1000) && !(CheckCTR())) {
      Serial.println("JUNCTION STILL TURNING");
      motorState = SHARP_LEFT;
    } else if (CheckCR() && CheckCL() && ((millis() - orientTime) > 4000)) {
      justGotHere = millis();
      motorState = BACKWARD;
      payloadState = OPEN;
      directionState = AT_PAYLOAD; // eventually will be AT_PAYLOAD
      Serial.println("AT PAYLOAD");
    }
  } else {
    if ((CheckL() && CheckCL() && !CheckR()) && ((millis() - orientTime) > 1500)) {  // junction case
      Serial.println("JUNCTION ARRIVED");
      motorState = FORWARD;
      passedJunction = true;
      junctionTime = millis();
    }  
  }


  if (millis() - orientTime < 250) {  // for leaving the studio
    Serial.println("LEAVING STUDIO");
    motorState = FORWARD;
  }


  OperateMotor();


  }




void GoodHomeLineFollow() {
  if (CheckCL() || CheckL()) {
    motorState = SHARP_LEFT;
    Serial.println("SHARP LEFT");
  } else if (CheckCR() || CheckR()) {
    motorState = SHARP_RIGHT;
    Serial.println("SHARP RIGHT");
  } else {
    motorState = FORWARD;
    Serial.println("FORWARD");
  }


  if (passedJunction) {
    if (CheckCL() && CheckCR() && (millis() - payloadTime > 6000)) {
      Serial.println("ENTER STUDIO");
      directionState = ENTERING_STUDIO;
      motorState = STOP;
      StudioEnterTime = millis();
      GoodButtonPressed = false;
      BadButtonPressed = false;
    }
  } else {
      if (CheckR() && CheckCR() && !CheckL() && ((millis() - payloadTime) > 1000)) {  //junction case
        Serial.println("FORWARD");
        motorState = FORWARD;
        junctionTime = millis();
        passedJunction = true;
      }
  }


  Serial.println();


  OperateMotor();
}




void BadHomeLineFollow() {
  if (CheckCR() || CheckR()) {
    motorState = SHARP_RIGHT;
    Serial.println("SHARP RIGHT");
  } else if (CheckCL() || CheckL()) {
    motorState = SHARP_LEFT;
    Serial.println("SHARP LEFT");
  } else {
    motorState = FORWARD;
    Serial.println("FORWARD");
  }


  if (passedJunction) {
    if ((millis() - junctionTime < 500)) {
      Serial.println("JUNCTION RIGHT");
      motorState = SHARP_RIGHT;
    } else if ((millis() - junctionTime < 1500) && !(CheckCTR())) {
      Serial.println("JUNCTION STILL TURNING");
      motorState = SHARP_RIGHT;
    } else if (CheckCL() && CheckCR() && (millis() - junctionTime > 2000)) {
      Serial.println("ENTER STUDIO");
      directionState = ENTERING_STUDIO;
      motorState = STOP;
      StudioEnterTime = millis();
      GoodButtonPressed = false;
      BadButtonPressed = false;
    }
  } else {
    if (CheckR() && CheckCL()) {  //junction case
      Serial.println("SHARP RIGHT");
      motorState = SHARP_RIGHT;
      junctionTime = millis();
      passedJunction = true;
    }
  }


  Serial.println();


  OperateMotor();
}


void GoodToBad() {
    if (CheckCL() || CheckL()) {
    motorState = SHARP_LEFT;
    Serial.println("SHARP LEFT");
  } else if (CheckCR() || CheckR()) {
    motorState = SHARP_RIGHT;
    Serial.println("SHARP RIGHT");
  } else {
    motorState = FORWARD;
    Serial.println("FORWARD");
  }


  if (passedJunction) {
    if ((millis() - junctionTime < 500)) {
      Serial.println("JUNCTION RIGHT");
      motorState = SHARP_RIGHT;
    } else if ((millis() - junctionTime < 1500) && !(CheckCTR())) {
      Serial.println("JUNCTION STILL TURNING");
      motorState = SHARP_RIGHT;
    } else if (CheckCL() && CheckCR() && ((millis() - junctionTime) > 3000)) {
      Serial.println("AT PAYLOAD");
      directionState = AT_PAYLOAD;
      motorState = BACKWARD;
      payloadState = OPEN;
    }
  } else {
    if ((CheckR() && CheckCR() && !CheckL()) && ((millis() - payloadTime) > 4000)) {  //junction case
      Serial.println("FORWARD");
      motorState = FORWARD;
      junctionTime = millis();
      passedJunction = true;
    }
  }


  Serial.println();


  OperateMotor();
}


/*----------Motor Functions----------*/


void OperateMotor() {
  switch (motorState) {
    case ORIENT_TURN:
    TurnOrient();
    break;


    case FORWARD:
    DriveFwd();
    break;


    case BACKWARD:
    DriveRev();
    break;


    case STOP:
    Stop();
    break;


    case SLIGHT_LEFT:
    TurnSlightLeft();
    break;


    case SLIGHT_RIGHT:
    TurnSlightRight();
    break;


    case SHARP_LEFT:
    TurnSharpLeft();
    break;


    case SHARP_RIGHT:
    TurnSharpRight();
    break;


    case SHARP_RIGHT_180:
    TurnSharpRight180();
    break;


    case SLOW_FWD:
    DriveSlowFwd();
    break;
  }
}


void DriveFwd(void) {
  // Serial.println("Fwd");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, HIGH);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, HIGH);


  analogWrite(R_motor_en, LINE_DUTY);
  analogWrite(L_motor_en, LINE_DUTY);
}


void DriveSlowFwd(void) {
  // Serial.println("Slow Fwd");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, HIGH);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, HIGH);


  analogWrite(R_motor_en, ORIENT_DUTY);
  analogWrite(L_motor_en, ORIENT_DUTY);
}


void DriveRev(void) {
  // Serial.println("Rev");
  digitalWrite(R_motor_dir1, HIGH);
  digitalWrite(R_motor_dir2, LOW);
  digitalWrite(L_motor_dir1, HIGH);
  digitalWrite(L_motor_dir2, LOW);


  analogWrite(R_motor_en, 207);
  analogWrite(L_motor_en, LINE_DUTY);
}




void TurnSharpRight(void) {
  // Serial.println("Sharp R");
  digitalWrite(R_motor_dir1, HIGH);
  digitalWrite(R_motor_dir2, LOW);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, HIGH);




  analogWrite(R_motor_en, SLIGHT_LINE_DUTY);
  analogWrite(L_motor_en, SLIGHT_LINE_DUTY);
}


void TurnSharpRight180(void) {
  // Serial.println("Sharp R");
  digitalWrite(R_motor_dir1, HIGH);
  digitalWrite(R_motor_dir2, LOW);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, HIGH);


  analogWrite(R_motor_en, TURN_180_DUTY);
  analogWrite(L_motor_en, TURN_180_DUTY);
}


void TurnSharpLeft(void) {
  // Serial.println("Sharp L");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, HIGH);
  digitalWrite(L_motor_dir1, HIGH);
  digitalWrite(L_motor_dir2, LOW);


  analogWrite(R_motor_en, SLIGHT_LINE_DUTY);
  analogWrite(L_motor_en, SLIGHT_LINE_DUTY);
}




void TurnOrient(void) {
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, HIGH);
  digitalWrite(L_motor_dir1, HIGH);
  digitalWrite(L_motor_dir2, LOW);


  analogWrite(R_motor_en, ORIENT_DUTY);
  analogWrite(L_motor_en, ORIENT_DUTY);
}


void TurnSlightRight(void) {
  // Serial.println("Slight R");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, LOW);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, HIGH);


  analogWrite(R_motor_en, LINE_DUTY);
  analogWrite(L_motor_en, LINE_DUTY);
}


void TurnSlightLeft(void) {
  // Serial.println("Slight L");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, HIGH);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, LOW);


  analogWrite(R_motor_en, LINE_DUTY);
  analogWrite(L_motor_en, LINE_DUTY);
}


void Stop(void) {
  //Serial.println("Stop");
  digitalWrite(R_motor_dir1, LOW);
  digitalWrite(R_motor_dir2, LOW);
  digitalWrite(L_motor_dir1, LOW);
  digitalWrite(L_motor_dir2, LOW);


  analogWrite(R_motor_en, 0);
  analogWrite(L_motor_en, 0);
}


void DriveEnterStudio(void) {
  if (millis() - StudioEnterTime < 250){
    Serial.println("FIRST STOP");
    motorState = BACKWARD;
  } else if (millis() - StudioEnterTime < 1000) {
    motorState = FORWARD;
  } else {
    Serial.println("STUDIO STOP");
    motorState = STOP;
    directionState = LOADING;
  }


  OperateMotor();
}


void Turn180(void) {
  if ((millis() - turn180Time) < 1000) {
    if ((millis() - turn180Time) < 350) {
      motorState = BACKWARD;
    } else if (CheckCL()) {
      motorState = SHARP_RIGHT;
    } else if (CheckCR()) {
      motorState = SHARP_LEFT;
    } else {
      motorState = BACKWARD;
    }
  } else if ((millis() - turn180Time) < 2000) {
    Serial.println("TURNING AROUND");
    motorState = SHARP_RIGHT_180;
  } else if (!CheckCTR() && (((millis() - turn180Time) > 2000)) && ((millis() - turn180Time) < 3000)) {
    Serial.println("where's the tape? STILL TURNING");
    motorState = SHARP_RIGHT_180;
  } else if (CheckCTR()) {
    Serial.println("FINISHED TURNING");
    motorState = STOP;


    if (doneBalling) {
      directionState = GOING_BAD;
    } else {
      directionState = GOOD_TO_BAD;
      doneBalling = true;
    }


  } else {
    Serial.println("STILL TURNING");
    motorState = SHARP_RIGHT;
  }


  passedJunction = false;
  OperateMotor();
}


/*----------Payload Functions----------*/


void OperatePayload() {
 
  if (millis() - justGotHere < 250) {
    motorState = BACKWARD;
    OperateMotor();
  } else {
    facingBeacon = false;


    motorState = STOP;
    OperateMotor();


    payloadServo.write(90);
    delay(250);
    payloadServo.write(180);
    delay(500);


    directionState = TURNING_180;
    turn180Time = millis();
    payloadTime = millis();
  }


}
















/*----------Check Buttons Functions-----------*/
















void CheckButtons(void){
  if (digitalRead(Good_Button_Pin)) {
    GoodButtonPressed = true;
  } else if (digitalRead(Bad_Button_Pin)) {
    BadButtonPressed = true;
  }
















  if (GoodButtonPressed || BadButtonPressed) {
    directionState = ORIENTING;
  }
}
















/*----------Check Tape Sensor Functions----------*/
















// if TRUE, there IS tape
int CheckL() {
















  int read = ReadTapeSensor(L_IR_sensor);
















  //Serial.print("L: "); Serial.println(read);
















  if (read >= LINE_THRESHOLD) {
    //Serial.println("L");
    return true;
  } else {
    //Serial.println("blank");
    return false;
  }
}
















int CheckCL() {
  int read = ReadTapeSensor(CL_IR_sensor);
















  //Serial.print("CL: "); Serial.println(read);
















  if (read >= LINE_THRESHOLD) {
    //Serial.println("CTR-LEFT");
    return true;
  } else {
    //Serial.println("blank");
    return false;
  }
}
















int CheckCTR() {
  int read = ReadTapeSensor(CTR_IR_sensor);
















  //Serial.print("CTR: "); Serial.println(read);
















  if (read >= LINE_THRESHOLD) {  
    //Serial.println("CTR");
    return true;
  } else {
    //Serial.println("blank");
    return false;
  }
}
















int CheckCR() {
  int read = ReadTapeSensor(CR_IR_sensor);
















  //Serial.print("CR: "); Serial.println(read);
















  if (read >= LINE_THRESHOLD) {
    //Serial.println("CTR-RIGHT");
    return true;
  } else {
    //Serial.println("blank");
    return false;
  }
}




int CheckR() {
  int read = ReadTapeSensor(R_IR_sensor);


  Serial.print("R: "); Serial.println(read);




  if (read >= 390) {
    //Serial.println("R");
    return true;
  } else {
    //Serial.println("blank");
    return false;
  }
}


// if FALSE, NONE of the sensors see tape
int ReadTriggers() {
  if (CheckL() || CheckCL() || CheckCTR() || CheckCR() || CheckR()){
    Serial.println("Triggered");
    return true;
  } else {
    Serial.println("Blank");
    return false;
  }
}


int ReadAll() {
  if (CheckL() && CheckCL() && CheckCTR() && CheckCR()){ //&& CheckR()
    Serial.println("All Tape");
    return true;
  } else {
    Serial.println("Blank");
    return false;
  }
}


int ReadTapeSensor(int sensor_pin){
  int read = 0;
  read = analogRead(sensor_pin);




  //Serial.println(read);
  return read;
}






void TurnOnIR(int LED_pin){
  digitalWrite(LED_pin, HIGH);
}




void TurnOffIR(int LED_pin){
  digitalWrite(LED_pin, LOW);
}




void Celebrate() {
  celebrateServo.write(120);
  delay(500);
  celebrateServo.write(60);
  delay(500);
  celebrateServo.write(90);
  delay(1000);
}






void HandleGameOver(){
  directionState = GAME_OVER;
  motorState = STOP;
  OperateMotor();
  Celebrate();
}



