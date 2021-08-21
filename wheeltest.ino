
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


const int rightforwa = 2;
const int rightforwb = 3;
const int leftforwa = 4;
const int leftforwb = 5;
const int rightbackwa = 6;
const int rightbackwb = 7;
const int leftbackwa = 8;
const int leftbackwb = 9;
const int enRF = 13;
const int enLF = 12;
const int enRB = 10;
const int enLB = 11;

float move1;
float move2;
float move3;

int speed_x = 0;
int speed_y = 0;
int speed_z = 0;

void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.linear.y;
  move3 = cmd_vel.angular.z;

 

  if (move1 > 0.2 && move2 > -0.2 && move2 < 0.2 && move3 > -0.2 && move3 < 0.2)
  {
  forward();
  }
  
  else if (move1 < -0.2 && move2 > -0.2 && move2 < 0.2 && move3 > -0.2 && move3 < 0.2)
  {
  back();
  }
  
  
  else if (move2 < -0.2 and move1 > -0.2 && move1 < 0.2 && move3 > -0.2 && move3 < 0.2) 
  {
  striveLeft();
  }
   
  else if (move2 > 0.2 and move1 > -0.2 && move1 < 0.2 && move3 > -0.2 && move3 < 0.2)
  {
  striveRight();  
  }
  
  
  else if (move1 > 0.2 && move2 < -0.2 && move3 > -0.2 && move3 < 0.2)
  {
  backstrLeft();
  }

  else if (move1 > 0.2 && move2 > 0.2 && move3 > -0.2 && move3 < 0.2)
  {
  forstrRight();
  }
  
  else if (move1 < -0.2 && move2 > 0.2 && move3 > -0.2 && move3 < 0.2)
  {
  backstrRight();
  }
 
  else if (move1 < -0.2 && move2 < -0.2 && move3 > -0.2 && move3 < 0.2)
  {
  forstrLeft();
  }  

  
  if (move1 > 0.2 && move2 > -0.2 && move2 < 0.2 && move3 > 0.2)
  {
  forwardturnRight();
  }

  if (move1 > 0.2 && move2 > -0.2 && move2 < 0.2 && move3 < -0.2)
  {
  forwardturnLeft();
  }

  
  else if (move1 < -0.2 && move2 > -0.2 && move2 < 0.2 && move3 > 0.2)
  {
  backturnRight();
  }
  
  else if (move1 < -0.2 && move2 > -0.2 && move2 < 0.2 && move3 < -0.2)
  {
  backturnLeft();
  }
  else if (move2 < -0.2 and move1 > -0.2 && move1 < 0.2 && move3 > 0.2) 
  {
  striveLeftturnRight();
  }

  else if (move2 < -0.2 and move1 > -0.2 && move1 < 0.2 && move3 < -0.2) 
  {
  striveLeftturnLeft();
  }  
   
  else if (move2 > 0.2 and move1 > -0.2 && move1 < 0.2 && move3 > 0.2)
  {
  striveRightturnRight();  
  }
  else if (move2 > 0.2 and move1 > -0.2 && move1 < 0.2 && move3 < -0.2)
  {
  striveRightturnLeft();
  }
  else if (move1 > 0.2 && move2 < -0.2 && move3 > 0.2)
  {
  backstrLeftturnRight();
  }
  else if (move1 > 0.2 && move2 < -0.2 && move3 < -0.2)
  {
  backstrLeftturnLeft();
  }
  else if (move1 > 0.2 && move2 > 0.2 && move3 > 0.2)
  {
  forstrRightturnRight();
  }
  else if (move1 > 0.2 && move2 > 0.2 && move3 < -0.2)
  {
  forstrRightturnLeft();
  }
  else if (move1 < -0.2 && move2 > 0.2 && move3 > 0.2)
  {
  backstrRightturnRight();
  }
  else if (move1 < -0.2 && move2 > 0.2 && move3 < -0.2)
  {
  backstrRightturnLeft();
  }
  else if (move1 < -0.2 && move2 < -0.2 && move3 > 0.2)
  {
  forstrLeftturnRight();
  }  
  else if (move1 < -0.2 && move2 < -0.2 && move3 < -0.2)
  {
  forstrLeftturnLeft();
  }  

  else if (move3 > 0.2 && move2 > -0.2 && move2 < 0.2 && move1 > -0.2 && move1 < 0.2)
  {
    turnRight();
  }

  else if (move3 < -0.2 && move2 > -0.2 && move2 < 0.2 && move1 -0.2 && move1 < 0.2)
  {
    turnLeft();
  }
  
    

  
  else
  {die();
  }
}





ros::NodeHandle nh;
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback);

void setup() {
pinMode(rightforwa, OUTPUT);
pinMode(rightforwb, OUTPUT);
pinMode(leftforwa, OUTPUT);
pinMode(leftforwb, OUTPUT);
pinMode(rightbackwa, OUTPUT);
pinMode(rightbackwb, OUTPUT);
pinMode(leftbackwa, OUTPUT);
pinMode(leftbackwb, OUTPUT);
pinMode(enRF, OUTPUT);
pinMode(enLF, OUTPUT);
pinMode(enRB, OUTPUT);
pinMode(enLB, OUTPUT); 

nh.initNode();
nh.subscribe(sub);
}


void loop() 
{

nh.spinOnce();
delay(1);

}


void back()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
    
}

void forstrLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, 160);
    analogWrite(enLF, LOW);
    analogWrite(enRB, LOW);
    analogWrite(enLB, 160);
    delay(130);
    die();     
}

void forstrRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);   
    analogWrite(enRF, 160);
    analogWrite(enLF, LOW);
    analogWrite(enRB, LOW);
    analogWrite(enLB, 160);
    delay(130);
    die();     
}

void backstrRight()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, LOW);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, LOW);
    delay(130);
    die();     
}

void backstrLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, LOW);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, LOW);
    delay(130);
    die();     
}

void forward()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH); 
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
}
void striveRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);  
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
}

void striveLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);  
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
}

void turnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);  
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
}
void turnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);  
    analogWrite(enRF, 140);
    analogWrite(enLF, 140);
    analogWrite(enRB, 140);
    analogWrite(enLB, 140);
    delay(130);
    die(); 
}

void die()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, LOW); 
    digitalWrite(enRF, LOW);
    digitalWrite(enLF, LOW);
    digitalWrite(enRB, LOW);
    digitalWrite(enLB, LOW);   
    delay(1);
    
}

void forwardturnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH); 
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160);   
    delay(130);
    die();
    
}

void forwardturnLeft()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH); 
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);   
    delay(130);
    die();
}  
void backturnRight()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160);   
    delay(130);
    die();
}  

void backturnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);   
    delay(130);
    die();
}  

void striveLeftturnRight()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);  
    analogWrite(enRF, 160);
    analogWrite(enLF, 80);
    analogWrite(enRB, 160);
    analogWrite(enLB, 160);
    delay(130);
    die();
}  

void striveLeftturnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);  
    analogWrite(enRF, 80);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 160);   
    delay(130);
    die();
    
}  


void striveRightturnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);  
    analogWrite(enRF, 80);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 160);
    delay(130);
    die();
    
}  

void striveRightturnLeft()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);  
    analogWrite(enRF, 160);
    analogWrite(enLF, 80);
    analogWrite(enRB, 160);
    analogWrite(enLB, 160);
    delay(130);
    die();
    
}  


void forstrRightturnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);   
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160);   
    delay(130);
    die();
    
}  

void forstrRightturnLeft()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);   
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160); 
    delay(130);  
    die();
    
}  

void backstrRightturnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);  
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);
    delay(130);
    die();     
    
}
void backstrRightturnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);   
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);
    delay(130);
    die();     
    
}  

 

void backstrLeftturnRight()
{
    digitalWrite(rightforwa, HIGH);
    digitalWrite(rightforwb, LOW);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);
    delay(130);
    die();
    
}  
void backstrLeftturnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, LOW);
    digitalWrite(leftbackwb, HIGH);     
    analogWrite(enRF, 100);
    analogWrite(enLF, 160);
    analogWrite(enRB, 160);
    analogWrite(enLB, 100);
    delay(130);
    die();
    
}  




void forstrLeftturnRight()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, LOW);
    digitalWrite(leftforwb, HIGH);
    digitalWrite(rightbackwa, LOW);
    digitalWrite(rightbackwb, HIGH);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160);
    delay(130);
    die();     
    
}  
void forstrLeftturnLeft()
{
    digitalWrite(rightforwa, LOW);
    digitalWrite(rightforwb, HIGH);
    digitalWrite(leftforwa, HIGH);
    digitalWrite(leftforwb, LOW);
    digitalWrite(rightbackwa, HIGH);
    digitalWrite(rightbackwb, LOW);
    digitalWrite(leftbackwa, HIGH);
    digitalWrite(leftbackwb, LOW);   
    analogWrite(enRF, 160);
    analogWrite(enLF, 100);
    analogWrite(enRB, 100);
    analogWrite(enLB, 160);
    delay(130);
    die();     
    
}  
