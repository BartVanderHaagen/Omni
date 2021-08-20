#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <stdlib.h>

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


#define ENCA_FLW 18 // YELLOW 
#define ENCB_FLW 31 // WHITE
#define PWM_FLW 12 //left_front_wheel
#define IN2_FLW 4  //a or positive
#define IN1_FLW 5  //b or negative

#define ENCA_FRW 19 
#define ENCB_FRW 33
#define PWM_FRW 13 //right_front_wheel
#define IN2_FRW 2  //a or positive
#define IN1_FRW 3  //b or negative

#define ENCA_BLW 20
#define ENCB_BLW 35
#define PWM_BLW 11 //left_back_wheel
#define IN2_BLW 8  //a or positive
#define IN1_BLW 9  //b or negative

#define ENCA_BRW 21
#define ENCB_BRW 37
#define PWM_BRW 10 //right_back_wheel
#define IN2_BRW 6  //a or positive
#define IN1_BRW 7  //b or negative

volatile int posiCFL = 0; 
volatile int posiCFR = 0;
volatile int posiCBR = 0;
volatile int posiCBL = 0;

long prevT_FR = 0;
float eprev_FR = 0;
float eintegral_FR = 0;

long prevT_FL = 0;
float eprev_FL = 0;
float eintegral_FL = 0;

long prevT_BL = 0;
float eprev_BL = 0;
float eintegral_BL = 0;

long prevT_BR = 0;
float eprev_BR = 0;
float eintegral_BR = 0;

int Vx=0;
int Vy=0;
int Vz=0;
int V=80;

int FRW = (Vx + Vy);
int FLW = (Vx - Vy);
int BLW = (Vx + Vy);
int BRW = (Vx - Vy);

void callback(const geometry_msgs::Twist& cmd_vel)
{
  Vx = cmd_vel.linear.x;
  Vy = cmd_vel.linear.y;
  Vz = cmd_vel.angular.z;


  //forwards
  if (Vx < -0.1 and Vy > -0.1 && Vy < 0.1 and Vz > -0.1 && Vz < 0.1) {
  Vx=(V*Vx);
  Vy=0;
  Vz=0; 
  
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
  //backwards
  if (Vx > 0.1 and Vy > -0.1 && Vy < 0.1 && Vz > -0.1 and Vz < 0.1) {
  Vx=(V*Vx);
  Vy=0;
  Vz=0; 
 
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }

  //strive_left 
  if (Vy < -0.1 && Vx > -0.1 and Vx < 0.1 && Vz > -0.1 and Vz < 0.1) {
  Vx=0;
  Vy=(V*Vy);
  Vz=0;
     
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);  
  mover();
  }
  
  // strive_right
  if (Vy > 0.1 && Vx > -0.1 and Vx < 0.1 && Vy > -0.1 and Vy < 0.1) {
  Vx=0;
  Vy=(V*Vy);
  Vz=0;     
  
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
  //turn_left
  if (Vz < -0.1 && Vx > -0.1 and Vx < 0.1 && Vy > -0.1 and Vy < 0.1) {
  Vx = (Vx * V); 
  Vy = (Vy * V); 

  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
  //turn_right
  if (Vz > 0.1 && Vx > -0.1 and Vx < 0.1 && Vy > -0.1 and Vy < 0.1) {
  Vx = (Vx * V); 
  Vy = (Vy * V);  

  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);  
  mover();
  }

//strive_left_backward
  if (Vx > 0.1 && Vy < -0.1 && Vz > -0.1 and Vz < 0.1){
  Vx=(V*Vx);
  Vy=(V*Vy);
  Vz=0; 
 
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
//strive_left_forward
  if (Vx < -0.1 && Vy > 0.1 && Vz > -0.1 and Vz < 0.1){
  Vx=(V*Vx);
  Vy=(V*Vy);
  Vz=0; 
  
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
//strive_right_forward
  if (Vx > 0.1 && Vy > 0.1 && Vz > -0.1 and Vz < 0.1){
  Vx=(V*Vx);
  Vy=(V*Vy);
  Vz=0; 
 
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);
  mover();
  }
  //strive_right_backward
  if (Vx < -0.1 && Vy < -0.1 && Vz > -0.1 and Vz < 0.1){
  Vx=(V*Vx);
  Vy=(V*Vy);
  Vz=0;
     
  FRW = (Vx + Vy);
  FLW = (Vx - Vy);
  BLW = (Vx + Vy);
  BRW = (Vx - Vy);  
  mover();
  }

  
  
}

  ros::NodeHandle nh;
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback);



void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(ENCA_FLW, INPUT);
  pinMode(ENCB_FLW, INPUT);
  pinMode(ENCA_FRW, INPUT);
  pinMode(ENCB_FRW, INPUT);
  pinMode(ENCA_BLW, INPUT);
  pinMode(ENCB_BLW, INPUT);
  pinMode(ENCA_BRW, INPUT);
  pinMode(ENCB_BRW, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCA_FLW),readEncoder_LF,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_FRW),readEncoder_RF,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_BLW),readEncoder_LB,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_BRW),readEncoder_RB,RISING);
  
  pinMode(PWM_FRW,OUTPUT);
  pinMode(IN1_FRW,OUTPUT);
  pinMode(IN2_FRW,OUTPUT);

  pinMode(PWM_FLW,OUTPUT);
  pinMode(IN1_FLW,OUTPUT);
  pinMode(IN2_FLW,OUTPUT);

  pinMode(PWM_BRW,OUTPUT);
  pinMode(IN1_BRW,OUTPUT);
  pinMode(IN2_BRW,OUTPUT);

  pinMode(PWM_BLW,OUTPUT);
  pinMode(IN1_BLW,OUTPUT);
  pinMode(IN2_BLW,OUTPUT);
  

}


void loop() {
 nh.spinOnce();
 delay(1);

}
void mover(){

  // set target position
  //target = 4000;
  //target = 4000*sin(3.14));
  //target = 4000*cos(3.14)); 
  int target_RF = FRW; //200*cos(prevT_FR/1e6);
  int target_LF = FLW; //200*cos(prevT_FL/1e6);
  int target_RB = BRW; //200*cos(prevT_BR/1e6);
  int target_LB = BLW; //200*cos(prevT_BL/1e6);

  // PID constants
  float kp_RF = 1;
  float kd_RF = 0.2;
  float ki_RF = 0;

  float kp_LF = 1;
  float kd_LF = 0.2;
  float ki_LF = 0;

  float kp_RB = 1;
  float kd_RB = 0.2;
  float ki_RB = 0;

  float kp_LB = 1;
  float kd_LB = 0.2;
  float ki_LB = 0;  

  // time difference
  long currT_RF = micros();
  float deltaT_RF = ((float) (currT_RF - prevT_FR))/( 1.0e6 );
  prevT_FR = currT_RF;

    long currT_FL = micros();
  float deltaT_FL = ((float) (currT_FL - prevT_FL))/( 1.0e6 );
  prevT_FL = currT_FL;

    long currT_RB = micros();
  float deltaT_RB = ((float) (currT_RB - prevT_BR))/( 1.0e6 );
  prevT_BR = currT_RB;

    long currT_LB = micros();
  float deltaT_LB = ((float) (currT_LB - prevT_BL))/( 1.0e6 );
  prevT_BL = currT_LB;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

  int pos_LF = 0; 
  int pos_RF = 0;
  int pos_LB = 0;
  int pos_RB = 0;
   
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_LF = posiCFL;
    pos_RF = posiCFR;
    pos_LB = posiCBL;
    pos_RB = posiCBR;
  }
  
  // error
  int e_LF = pos_LF - target_LF;
  int e_RF = pos_RF - target_RF;
  int e_LB = pos_LB - target_LB;
  int e_RB = pos_RB - target_RB;

  // derivative
  float dedt_LF = (e_LF-eprev_FL)/(deltaT_FL);
  float dedt_RF = (e_RF-eprev_FR)/(deltaT_RF);
  float dedt_LB = (e_LB-eprev_BL)/(deltaT_LB);
  float dedt_RB = (e_RB-eprev_BR)/(deltaT_RB);

  // integral
  eintegral_FL = eintegral_FL + e_LF*deltaT_FL;
  eintegral_FR = eintegral_FR + e_RF*deltaT_RF;
  eintegral_BL = eintegral_BL + e_LB*deltaT_LB;
  eintegral_BR = eintegral_BR + e_RB*deltaT_RB;
  
  // control signal
  float u_LF = kp_LF*e_LF + kd_LF*dedt_LF + ki_LF*eintegral_FL;
  float u_LB = kp_LB*e_LB + kd_LB*dedt_LB + ki_LB*eintegral_BL;
  float u_RF = kp_RF*e_RF + kd_RF*dedt_RF + ki_RF*eintegral_FR;
  float u_RB = kp_RB*e_RB + kd_RB*dedt_RB + ki_RB*eintegral_BR;
  

  // motor power
  float pwr_LF = fabs(u_LF);
  if( pwr_LF > 255 ){
    pwr_LF = 255;
    if (pwr_LF < 70){
      pwr_LF = 0;
    }
  }

  float pwr_RF = fabs(u_RF);
  if( pwr_RF > 255 ){
    pwr_RF = 255;
    if (pwr_RF < 70){
      pwr_RF = 0;
    }
  }

  float pwr_LB = fabs(u_LB);
  if( pwr_LB > 255 ){
    pwr_LB = 255;
  if (pwr_LB < 70){
      pwr_LB = 0;
    }
  }

  float pwr_RB = fabs(u_RB);
  if( pwr_RB > 255 ){
    pwr_RB = 255;
  if (pwr_RB < 70){
      pwr_RB = 0;
    }  
  }

  // motor direction
  int dir_LF = 1;
  if(u_LF<0){
    dir_LF = -1;
  }

  int dir_RF = -1;
  if(u_RF<0){
    dir_RF = 1;
  }

  int dir_LB = -1;
  if(u_LB<0){
    dir_LB = 1;
  }

  int dir_RB = 1;
  if(u_RB<0){
    dir_RB = -1;
  }

  // signal the motor
  setMotor_LF(dir_LF,pwr_LF,PWM_FLW,IN1_FLW,IN2_FLW);
  setMotor_RF(dir_RF,pwr_RF,PWM_FRW,IN1_FRW,IN2_FRW);
  setMotor_LB(dir_LB,pwr_LB,PWM_BLW,IN1_BLW,IN2_BLW);
  setMotor_RB(dir_RB,pwr_RB,PWM_BRW,IN1_BRW,IN2_BRW);

  // store previous error
  eprev_FL= e_LF;
  eprev_FR= e_RF;
  eprev_BL= e_LB;
  eprev_BR= e_RB;

  
}

void setMotor_LF(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void setMotor_RF(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void setMotor_LB(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void setMotor_RB(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}


void readEncoder_LF(){
  int b_LF = digitalRead(ENCB_FLW);
  if(b_LF > 0){
    posiCFL++;
  }
  else{
    posiCFL--;
  }
}

void readEncoder_RF(){
  int b_RF = digitalRead(ENCB_FRW);
  if(b_RF > 0){
    posiCFR++;
  }
  else{
    posiCFR--;
  }
}

void readEncoder_LB(){
  int b_LB = digitalRead(ENCB_BLW);
  if(b_LB > 0){
    posiCBL++;
  }
  else{
    posiCBL--;
  }
}

void readEncoder_RB(){
  int b_RB = digitalRead(ENCB_BRW);
  if(b_RB > 0){
    posiCBR++;
  }
  else{
    posiCBR--;
  }
}
