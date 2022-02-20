
/*

 */

#include "foxbot_core_config.h"
#include <Wire.h>


// Real Time debug GPIO with oscilloscop
#define RT_PIN0 8
#define RT_PIN1 9



geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;



void setup()
{
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
  
  // serial
  nh.getHardware()->setBaud(115200);
  analogWriteResolution(12);

  
  // ROS node initialization
  nh.initNode();
  broadcaster.init(nh);
  
  // Subsriber
  nh.subscribe(cmd_vel_sub);
  //nh.subscribe(sub_shoulder);
  //nh.subscribe(sub_elbow);
  

  joints.name_length = 4; //pan and tilt with it ? length is 6
  joints.position_length = 4; //pan and tilt with it ? length is 6
  joints.velocity_length = 4; //pan and tilt with it ? length is 6
  joints.effort_length = 4; //pan and tilt with it ? length is 6
  joints.name[4]; //pan and tilt with it ? length is 6
  joints.position[4]; //pan and tilt with it ? length is 6
  joints.name = as;
  
  positions_tab[0] = 0.0;//right_rate_est.data;
  positions_tab[1] = 0.0;//left_rate_est.data;
  positions_tab[2] = 0.0;//back_right_rate_est.data;
  positions_tab[3] = 0.0;//back_left_rate_est.data;
  //positions_tab[4] = 0.0;//pan_rate_est.data;
  //positions_tab[5] = 0.0;//tilt_rate_est.data;
  velocities_tab[0] = 0.0;
  velocities_tab[1] = 0.0;
  velocities_tab[2] = 0.0;
  velocities_tab[3] = 0.0;
  //velocities_tab[4] = 0.0;
  //velocities_tab[5] = 0.0;
  efforts_tab[0] = 0.0;
  efforts_tab[1] = 0.0;
  efforts_tab[2] = 0.0;
  efforts_tab[3] = 0.0;
  //efforts_tab[4] = 0.0;
  //efforts_tab[5] = 0.0;
 
  // Builtin LED
  pinMode(LED_BUILTIN, OUTPUT);

  // blink LED
  digitalWrite(LED_BUILTIN, LOW);

  // define pin mode motor Right
  pinMode(motorRightEncoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(motorRightEncoderPinA),
          motorRightIsrCounterDirection, RISING);
  pinMode(motorRightEncoderPinB, INPUT);
  pinMode(enMotorRight, OUTPUT);
  pinMode(in1MotorRight, OUTPUT);
  pinMode(in2MotorRight, OUTPUT);

  // define pin mode motor Left
  pinMode(motorLeftEncoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(motorLeftEncoderPinA),
          motorLeftIsrCounterDirection, RISING);
  pinMode(motorLeftEncoderPinB, INPUT);
  pinMode(enMotorLeft, OUTPUT);
  pinMode(in1MotorLeft, OUTPUT);
  pinMode(in2MotorLeft, OUTPUT);

    // define pin mode motor Back Right
  pinMode(motorBackRightEncoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(motorBackRightEncoderPinA),
          motorBackRightIsrCounterDirection, RISING);
  pinMode(motorBackRightEncoderPinB, INPUT);
  pinMode(enMotorBackRight, OUTPUT);
  pinMode(in1MotorBackRight, OUTPUT);
  pinMode(in2MotorBackRight, OUTPUT);

  // define pin mode motor Back Left
  pinMode(motorBackLeftEncoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(motorBackLeftEncoderPinA),
          motorBackLeftIsrCounterDirection, RISING);
  pinMode(motorBackLeftEncoderPinB, INPUT);
  pinMode(enMotorBackLeft, OUTPUT);
  pinMode(in1MotorBackLeft, OUTPUT);
  pinMode(in2MotorBackLeft, OUTPUT);

  // define tasks frequency
  _frequency_rate.start((unsigned long) millis());
  _frequency_odometry.start((unsigned long) millis());
  _frequency_controller.start((unsigned long) millis());
  _frequency_jsp.start((unsigned long) millis());
  
  //Odometry publisher
  nh.advertise(odom_publisher);

  //Joint state publisher
  nh.advertise(joint_state_publisher);
  
  // Wheels 
  nh.advertise(wheel_quat_pub_rf);
  nh.advertise(wheel_quat_pub_lf);
  nh.advertise(wheel_quat_pub_rb);
  nh.advertise(wheel_quat_pub_lb);

  //pan tilt functions 
  //nh.advertise(pan_rate_pub);
  //nh.advertise(tilt_rate_pub);
  
  // Debugger
  nh.advertise(debug_publisher_1);
  
  }
  



void loop() {
  // rate computation
  if(_frequency_rate.delay(millis())) {
    digitalWrite(RT_PIN0, HIGH);

    float dt;

    // MOTOR RIGHT
    
    // direction
    motor_right_direction_median_filter.in(motor_right_direction);
    motor_right_filtered_direction = motor_right_direction_median_filter.out();

    // filter increment per second
    dt = (millis() - motor_right_prev_time);
    motor_right_prev_time = millis();
    motor_right_filtered_inc_per_second = runningAverage(motor_right_filtered_inc_per_second,
        (float)motor_right_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);

    // estimated rate
    motor_right_rate_est = (float)motor_right_filtered_direction
               * motor_right_filtered_inc_per_second * RATE_CONV;
    motor_right_inc = 0;

    if (abs(motor_right_rate_est) < 0.1f)
      motor_right_rate_est = 0.0f;

    motor_right_check_dir = 1;
    motor_right_direction = 0;

    // MOTOR LEFT
    // direction
    motor_left_direction_median_filter.in(motor_left_direction);
    motor_left_filtered_direction = motor_left_direction_median_filter.out();

    // filter increment per second
    dt = (millis() - motor_left_prev_time);
    motor_left_prev_time = millis();
    motor_left_filtered_inc_per_second = runningAverage(motor_left_filtered_inc_per_second,
        (float)motor_left_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);


    // estimated rate
    motor_left_rate_est = (float)motor_left_filtered_direction
              * motor_left_filtered_inc_per_second * RATE_CONV;
    motor_left_inc = 0;

    if (abs(motor_left_rate_est) < 0.1f)
      motor_left_rate_est = 0.0f;
    motor_left_check_dir = 1;
    motor_left_direction = 0;


   // MOTOR BACK RIGHT
    // direction
    motor_back_right_direction_median_filter.in(motor_back_right_direction);
    motor_back_right_filtered_direction = motor_back_right_direction_median_filter.out();

    // filter increment per second
    dt = (millis() - motor_back_right_prev_time);
    motor_back_right_prev_time = millis();
    motor_back_right_filtered_inc_per_second = runningAverage(motor_back_right_filtered_inc_per_second,
        (float)motor_back_right_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);

    // estimated rate
    motor_back_right_rate_est = (float)motor_back_right_filtered_direction
               * motor_back_right_filtered_inc_per_second * RATE_CONV;
    motor_back_right_inc = 0;

    if (abs(motor_back_right_rate_est) < 0.1f)
      motor_back_right_rate_est = 0.0f;

    motor_back_right_check_dir = 1;
    motor_back_right_direction = 0;

    // MOTOR BACK LEFT
    // direction
    motor_back_left_direction_median_filter.in(motor_back_left_direction);
    motor_back_left_filtered_direction = motor_back_left_direction_median_filter.out();

    // filter increment per second
    dt = (millis() - motor_back_left_prev_time);
    motor_back_left_prev_time = millis();
    motor_back_left_filtered_inc_per_second = runningAverage(motor_back_left_filtered_inc_per_second,
        (float)motor_back_left_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);


    // estimated rate
    motor_back_left_rate_est = (float)motor_back_left_filtered_direction
              * motor_back_left_filtered_inc_per_second * RATE_CONV;
    motor_back_left_inc = 0;

    if (abs(motor_back_left_rate_est) < 0.1f)
      motor_back_left_rate_est = 0.0f;
      
    motor_back_left_check_dir = 1;
    motor_back_left_direction = 0;

    

    // Motor_comand

        
    // to stop
    if ((linear_velocity_ref_x < 0.1 && linear_velocity_ref_x > -0.1) and (linear_velocity_ref_y > -0.1 && linear_velocity_ref_y < 0.1) and (angular_velocity_ref_z > -0.1 && angular_velocity_ref_z < 0.1)) {
    motor_right_rate_ref = 0;//(linear_velocity_ref_x);
    motor_left_rate_ref  = 0;//(linear_velocity_ref_x);
    motor_back_right_rate_ref = 0;//(linear_velocity_ref_x);
    motor_back_left_rate_ref  = 0;//(linear_velocity_ref_x);
    
    //delay(130);
    //die(); 
    }
    
       // this is where the magic happens
    else {
    motor_right_rate_ref = (linear_velocity_ref_x + linear_velocity_ref_y + angular_velocity_ref_z) *4;
    motor_left_rate_ref  = (linear_velocity_ref_y - linear_velocity_ref_x + angular_velocity_ref_z) *4;
    motor_back_right_rate_ref = (linear_velocity_ref_x - linear_velocity_ref_y + angular_velocity_ref_z) *4;
    motor_back_left_rate_ref  = (-linear_velocity_ref_y - linear_velocity_ref_x + angular_velocity_ref_z) *4;
    
      
    }

    
    digitalWrite(RT_PIN0, LOW);
  }

  // rate controler
  if(_frequency_controller.delay(millis())) {
    digitalWrite(RT_PIN1, HIGH);

    // MOTOR RIGHT
    rateControler(motor_right_rate_ref, motor_right_rate_est, motor_right_pwm_rate,
            controler_motor_right_prev_time, controler_motor_right_prev_epsilon,
            controler_motor_right_int);
    pwmMotorRight = pwmMotorRight + motor_right_pwm_rate;
    pwmMotorRight = constrain(pwmMotorRight, 0, PWM_MAX);
    setMotorRateAndDirection(pwmMotorRight, motor_right_rate_ref, enMotorRight,
                 in1MotorRight, in2MotorRight);

    // MOTOR LEFT
    rateControler(motor_left_rate_ref, motor_left_rate_est, motor_left_pwm_rate,
            controler_motor_left_prev_time, controler_motor_left_prev_epsilon,
            controler_motor_left_int);
    pwmMotorLeft = pwmMotorLeft + motor_left_pwm_rate;
    pwmMotorLeft = constrain(pwmMotorLeft, 0, PWM_MAX);
    setMotorRateAndDirection(pwmMotorLeft, motor_left_rate_ref, enMotorLeft,
                 in1MotorLeft, in2MotorLeft);

    // MOTOR back RIGHT
    rateControler(motor_back_right_rate_ref, motor_back_right_rate_est, motor_back_right_pwm_rate,
            controler_motor_back_right_prev_time, controler_motor_back_right_prev_epsilon,
            controler_motor_back_right_int);
    pwmMotorBackRight = pwmMotorBackRight + motor_back_right_pwm_rate;
    pwmMotorBackRight = constrain(pwmMotorBackRight, 0, PWM_MAX);
    setMotorRateAndDirection(pwmMotorBackRight, motor_back_right_rate_ref, enMotorBackRight,
                 in1MotorBackRight, in2MotorBackRight);

    // MOTOR back LEFT
    rateControler(motor_back_left_rate_ref, motor_back_left_rate_est, motor_back_left_pwm_rate,
            controler_motor_back_left_prev_time, controler_motor_back_left_prev_epsilon,
            controler_motor_back_left_int);
    pwmMotorBackLeft = pwmMotorBackLeft + motor_back_left_pwm_rate;
    pwmMotorBackLeft = constrain(pwmMotorBackLeft, 0, PWM_MAX);
    setMotorRateAndDirection(pwmMotorBackLeft, motor_back_left_rate_ref, enMotorBackLeft,
                 in1MotorBackLeft, in2MotorBackLeft);

    //DEBUG
    debug_1.x = 0;
    debug_1.y = 0;
    debug_1.z = yaw_est;
    debug_publisher_1.publish(&debug_1);

 

  }

  // update odometry
  if(_frequency_odometry.delay(millis())) {
    float dt, dx, dy;
    float qw, qx, qy, qz;

    dt = (float)(millis() - odom_prev_time) * 0.001f;
    odom_prev_time = millis();

   // compute linear and angular estimated velocity

   // standing still  
        
    if ((motor_right_rate_est > -0.1 && motor_left_rate_est < 0.1) and (motor_back_right_rate_est < 0.1 && motor_back_left_rate_est > -0.1)){
       linear_velocity_est_x = 0;
       linear_velocity_est_y = 0;
       angular_velocity_est_z = 0;

       
    }
    
    else {   //we have movement..

   
   if ((linear_velocity_ref_x > 0.01) or (linear_velocity_ref_x < -0.01))  {  
          linear_velocity_est_x = linear_velocity_ref_x * ((motor_right_rate_est + motor_back_right_rate_est) - (motor_left_rate_est + motor_back_left_rate_est)) /4 *(WHEEL_RADIUS);
          linear_velocity_est_y = linear_velocity_ref_x * ((motor_left_rate_est + motor_right_rate_est) - (motor_back_left_rate_est + motor_back_right_rate_est)) /4 *(WHEEL_RADIUS);
          angular_velocity_est_z = (WHEEL_RADIUS / BASE_LENGTH) * (motor_right_rate_est + motor_back_right_rate_est + motor_left_rate_est + motor_back_left_rate_est) ;
       }

   
    }
    
    // compute translation and rotation
    // Quaternion data
    yaw_est += angular_velocity_est_z * dt;// (orientationData.orientation.x * 3.1415 / 180);
    dx =  linear_velocity_est_x * dt;
    dy =  linear_velocity_est_y * dt;



    // compute quaternion
    //qw = cos (yaw_est);
    qw = cos(abs(yaw_est) / 4.0f ); 
    qx = 0.0f;
    qy = 0.0f;
    qz = sign(yaw_est) * sin(abs(yaw_est) / 4.0f ); 
       
   
    
    // feed odom message
    wheel_odom.header.stamp = nh.now();
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_footprint";
    wheel_odom.pose.pose.position.x += dx;
    wheel_odom.pose.pose.position.y += dy;
    wheel_odom.pose.pose.position.z = 0.0;
    wheel_odom.pose.pose.orientation.w = qw;
    wheel_odom.pose.pose.orientation.x = qx;
    wheel_odom.pose.pose.orientation.y = qy;
    wheel_odom.pose.pose.orientation.z = qz;
        
        // Velocity expressed in base_link frame
    wheel_odom.twist.twist.linear.x = linear_velocity_est_x;
    wheel_odom.twist.twist.linear.y = linear_velocity_est_y;
    wheel_odom.twist.twist.angular.z = angular_velocity_est_z;

    odom_publisher.publish(&wheel_odom);
  
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x += dx; 
    t.transform.translation.y += dy;
    t.transform.translation.z = 0.0;  
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy; 
    t.transform.rotation.z = qz; 
    t.transform.rotation.w = qw;  
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);

   //sensorValuePosShoulder = 0; //
   //sensorValuePosElbow = 0; //

   // sensorValuePosShoulder = analogRead(pan_rate_est_pin);
   // sensorValuePosElbow = (analogRead(tilt_rate_est_pin)  - ELBOW_ZER0) / ELBOW_RATIO;

  // pan 
 // pan_est.data =  convert_adc(sensorValuePosShoulder);
 // pan_rate_pub.publish(&pan_est);
   
  // tilt 
  //tilt_est.data = convert_adc(sensorValuePosElbow);
  //tilt_rate_pub.publish(&tilt_est);

  // wheel_quat in ticks 
  right_rate_est.data += motor_right_rate_est *dt;
  wheel_quat_pub_rf.publish(&right_rate_est);
    
  left_rate_est.data += (motor_left_rate_est *dt) *-1;    
  wheel_quat_pub_lf.publish(&left_rate_est);
    
  back_right_rate_est.data += motor_back_right_rate_est *dt;
  wheel_quat_pub_rb.publish(&back_right_rate_est);

  back_left_rate_est.data += (motor_back_left_rate_est *dt) *-1;
  wheel_quat_pub_lb.publish(&back_left_rate_est);
  
     
  // feed joint_states message    
  joints.header.stamp = nh.now();
  //positions_tab[0] = pan_est.data;
  //positions_tab[1] = tilt_est.data;
  positions_tab[0] = right_rate_est.data;
  positions_tab[1] = left_rate_est.data;
  positions_tab[2] = back_right_rate_est.data;
  positions_tab[3] = back_left_rate_est.data;
  velocities_tab[0] = motor_right_rate_est;
  velocities_tab[1] = motor_left_rate_est;
  velocities_tab[2] = motor_back_right_rate_est;
  velocities_tab[3] = motor_back_left_rate_est;
  
  
  
  
  
  joints.position = positions_tab;
  joints.velocity = velocities_tab;
  joints.effort = efforts_tab;
  
  joint_state_publisher.publish( &joints );

 
   digitalWrite(RT_PIN1, LOW);

  }

     
  // update subscribers values
  if(_frequency_rospinonce.delay(millis())) {
    nh.spinOnce();
  }

}


void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  linear_velocity_ref_x  = cmd_vel_msg.linear.x;
  linear_velocity_ref_y  = cmd_vel_msg.linear.y;
  angular_velocity_ref_z = cmd_vel_msg.angular.z;
}


 
double convert_adc(int value)
{
  return -(value - 450) * ADCVALUE;
}



void motorRightIsrCounterDirection() {
  motor_right_inc ++;
  if ( motor_right_check_dir == 1) {
    if (digitalRead(motorRightEncoderPinB) && digitalRead(motorRightEncoderPinA)){
      motor_right_direction = -1;
    } else {
      motor_right_direction = 1;
    }
    motor_right_check_dir = 0;
  }
}

void motorLeftIsrCounterDirection() {
  motor_left_inc ++;
  if ( motor_left_check_dir == 1) {
    if ( digitalRead(motorLeftEncoderPinB) && digitalRead(motorLeftEncoderPinA)){
      motor_left_direction = 1;
    } else {
      motor_left_direction = -1;
    }
    motor_left_check_dir = 0;
  }
}

void motorBackRightIsrCounterDirection() {
  motor_back_right_inc ++;
  if ( motor_back_right_check_dir == 1) {
    if (digitalRead(motorBackRightEncoderPinB) && digitalRead(motorBackRightEncoderPinA)){
      motor_back_right_direction = -1;
    } else {
      motor_back_right_direction = 1;
    }
    motor_back_right_check_dir = 0;
  }
}

void motorBackLeftIsrCounterDirection() {
  motor_back_left_inc ++;
  if ( motor_back_left_check_dir == 1) {
    if ( digitalRead(motorBackLeftEncoderPinB) && digitalRead(motorBackLeftEncoderPinA)){
      motor_back_left_direction = 1;
    } else {
      motor_back_left_direction = -1;
    }
    motor_back_left_check_dir = 0;
  }
}

void rateControler(const float rate_ref, const float rate_est, int & pwm_rate,
           unsigned long & prev_time, float & previous_epsilon, float & i_epsilon) {

  float epsilon = abs(rate_ref) - abs(rate_est);
  float d_epsilon = (epsilon - previous_epsilon) / (prev_time - millis());

  // reset and clamp integral (todo : add anti windup)
  if (rate_ref == 0.0) {
    i_epsilon = 0.0;
  } else {
    i_epsilon += epsilon * (prev_time - millis()) * RATE_CONTROLLER_KI;
  }
  i_epsilon = constrain(i_epsilon, -RATE_INTEGRAL_FREEZE, RATE_INTEGRAL_FREEZE);

  prev_time = millis();
  previous_epsilon = epsilon;

  //debug_left.z = i_epsilon * RATE_CONTROLLER_KI;

  pwm_rate = epsilon * RATE_CONTROLLER_KP
       + d_epsilon * RATE_CONTROLLER_KD
       + i_epsilon * RATE_CONTROLLER_KI;

  // saturate output
  pwm_rate = constrain(pwm_rate, RATE_CONTROLLER_MIN_PWM, RATE_CONTROLLER_MAX_PWM);
}

float runningAverage(float prev_avg, const float val, const int n) {

  return (prev_avg * (n - 1) + val) / n;
}



void die()
{
    digitalWrite(in1MotorRight, LOW);
    digitalWrite(in2MotorRight, LOW);
    digitalWrite(in1MotorLeft, LOW);
    digitalWrite(in2MotorLeft, LOW);
    digitalWrite(in1MotorBackRight, LOW);
    digitalWrite(in2MotorBackRight, LOW);
    digitalWrite(in1MotorBackLeft, LOW);
    digitalWrite(in2MotorBackLeft, LOW); 
    digitalWrite(enMotorRight, LOW);
    digitalWrite(enMotorLeft, LOW);
    digitalWrite(enMotorBackRight, LOW);
    digitalWrite(enMotorBackLeft, LOW);   
    delay(1);

}

void setMotorRateAndDirection(int pwm_ref, const float rate_ref,
                const byte enMotor, const byte in1Motor, const byte in2Motor) {

    // avoid noisy pwm range
    if (abs(rate_ref) < 0.1)
      pwm_ref = 0;

    // write direction
    if (rate_ref > 0) {
      digitalWrite(in1Motor, LOW);
      digitalWrite(in2Motor, HIGH);
    }
    else if (rate_ref < 0) {
      digitalWrite(in1Motor, HIGH);
      digitalWrite(in2Motor, LOW);
    }

    // write pwm
    analogWrite(enMotor, pwm_ref);
}
