
/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/    
 * www.osoyoo.com IR remote control smart car
 * program tutorial http://osoyoo.com/2018/12/14/osoyoo-smart-car-lesson-2-control-robot-car-through-infrared-remote/
 *  Copyright John Yu
 */
#include <IRremote.h>  
#define IR_PIN    10 //IR receiver Signal pin connect to Arduino pin D2 
 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;   
#define speedPinR 46 // RIGHT PWM pin connect MODEL-X ENA, was 9
#define RightDirectPin1 48
#define RightDirectPin2 50
#define speedPinL 47 // Left PWM pin connect MODEL-X ENB, was 6
#define LeftDirectPin1 49
//Right Motor direction pin 1 to MODEL-X IN1, was 12
//Right Motor direction pin 2 to MODEL-X IN2, was 11
//Left Motor direction pin 1 to MODEL-X IN3, was 7
#define LeftDirectPin2 51 //Left Motor direction pin 1 to MODEL-X IN4, was 8

 #define IR_ADVANCE       0x00FF18E7       //code from IR controller "▲" button
 #define IR_BACK          0x00FF4AB5       //code from IR controller "▼" button
 #define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
 #define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
 #define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
 #define IR_turnsmallleft 0x00FFB04F       //code from IR controller "#" button
 #define speedPinR_2 44 // RIGHT PWM pin connect MODEL-X ENA, was 9
#define RightDirectPin1_2 40 //Right Motor direction pin 1 to MODEL-X IN1, was 12
#define RightDirectPin2_2 42 //Right Motor direction pin 2 to MODEL-X IN2, was 11
#define speedPinL_2 45 // Left PWM pin connect MODEL-X ENB, was 6
#define LeftDirectPin1_2 41 //Left Motor direction pin 1 to MODEL-X IN3, was 7
#define LeftDirectPin2_2 43 //Left Motor direction pin 1 to MODEL-X IN4, was 8

enum DN
{ 
  GO_ADVANCE, //go forward
  GO_LEFT, //left turn
  GO_RIGHT,//right turn
  GO_BACK,//backward
  STOP_STOP, 
  DEF
}Drive_Num=DEF;

bool stopFlag = true;//set stop flag
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;
uint8_t motor_update_flag = 0;
/***************motor control***************/


void stop_Idle() //Stop but coast
{
Serial.println ("stop_Idle");
analogWrite(speedPinL,0); // front controller
analogWrite(speedPinR,0);
analogWrite(speedPinL_2,0);//back controller
analogWrite(speedPinR_2,0);
}

void go_Advance_R(void) //Forward, rear controller
{
Serial.println ("go_Advance rear");
digitalWrite(RightDirectPin1_2, HIGH);
digitalWrite(RightDirectPin2_2,LOW);
digitalWrite(LeftDirectPin1_2,HIGH);
digitalWrite(LeftDirectPin2_2,LOW);
analogWrite(speedPinL_2,255);
analogWrite(speedPinR_2,255);
}
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,255);
  analogWrite(speedPinR,255);
}

void go_Left_R(int t=0)  //Turn left
{
  digitalWrite(RightDirectPin1_2, HIGH);
  digitalWrite(RightDirectPin2_2,LOW);
  digitalWrite(LeftDirectPin1_2,LOW);
  digitalWrite(LeftDirectPin2_2,HIGH);
  analogWrite(speedPinL_2,200);
  analogWrite(speedPinR_2,200);
  delay(t);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void go_Right_R(int t=0)  //Turn right
{
  digitalWrite(RightDirectPin1_2, LOW);
  digitalWrite(RightDirectPin2_2,HIGH);
  digitalWrite(LeftDirectPin1_2,HIGH);
  digitalWrite(LeftDirectPin2_2,LOW);
  analogWrite(speedPinL_2,200);
  analogWrite(speedPinR_2,200);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void go_Back_R(int t=0)  //Reverse
{
  digitalWrite(RightDirectPin1_2, LOW);
  digitalWrite(RightDirectPin2_2,HIGH);
  digitalWrite(LeftDirectPin1_2,LOW);
  digitalWrite(LeftDirectPin2_2,HIGH);
  analogWrite(speedPinL_2,255);
  analogWrite(speedPinR_2,255);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,255);
  analogWrite(speedPinR,255);
  delay(t);
}

void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
}

/**************detect IR code***************/
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_ADVANCE)
    {
      Drive_Num=GO_ADVANCE;
    }
    else if(IRresults.value==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
    }
    else if(IRresults.value==IR_LEFT)
    {
       Drive_Num=GO_LEFT;
    }
    else if(IRresults.value==IR_BACK)
    {
        Drive_Num=GO_BACK;
    }
    else if(IRresults.value==IR_STOP)
    {
        Drive_Num=STOP_STOP; 
    }
    IRresults.value = 0;
    IR.resume();
  }
}

/**************car control**************/
void do_Drive_Tick()
{
    switch (Drive_Num) 
    {
      case GO_ADVANCE:go_Advance();go_Advance_R();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_ADVANCE code is detected, then go advance
      case GO_LEFT: go_Left();go_Left_R();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_LEFT code is detected, then turn left
      case GO_RIGHT:  go_Right();go_Right_R();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_RIGHT code is detected, then turn right
      case GO_BACK: go_Back();go_Back_R();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_BACK code is detected, then backward
      case STOP_STOP: stop_Idle();JogTime = 0;break;//stop coast
      default:break;
    }
    Drive_Num=DEF;
   //keep current moving mode for  200 millis seconds
    if(millis()-JogTime>=200)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        //stop_Stop();
        stop_Idle(); // coast
      }
    }
}

void setup()
{
  Serial.begin(9600);
while(!Serial)
{
}
;
Serial.println ("Testing IR receiver driving");
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();

  pinMode(IR_PIN, INPUT); 
  digitalWrite(IR_PIN, HIGH);  
  IR.enableIRIn();       
}


void loop()
{
  do_IR_Tick();
  do_Drive_Tick();
}
