#include <ros.h>
#include <rover_msgs/ArmCmd.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

// Stepper pins (STEP, DIR, EN)
const uint8_t STEP_PIN[5] = {2,3,4,5,6};
const uint8_t DIR_PIN [5] = {7,8,9,10,11};
const uint8_t EN_PIN  [5] = {12,13,22,23,24};

// Limit switches (NO -> GND, INPUT_PULLUP)
const uint8_t LIM_PIN[5]  = {25,26,27,28,29};

// Steps per degree per joint (gearboxes etc.)
float STEPS_PER_DEG[5] = {10.0, 10.0, 10.0, 8.0, 8.0}; // tune each axis

volatile long step_pos[5] = {0,0,0,0,0}; // in steps
long target_steps[5] = {0,0,0,0,0};
float speed_scale = 0.5;

sensor_msgs::JointState js;
ros::Publisher js_pub("/arm/joint_states", &js);

void armCmdCb(const rover_msgs::ArmCmd& cmd){
  if(cmd.joint_deg_length < 5) return;
  speed_scale = cmd.speed_scale;
  speed_scale = constrain(speed_scale, 0.05, 1.0);

  for(int i=0;i<5;i++){
    target_steps[i] = (long)(cmd.joint_deg[i]*STEPS_PER_DEG[i]);
  }
}
ros::Subscriber<rover_msgs::ArmCmd> arm_sub("/arm/joint_cmd", &armCmdCb);

void setup(){
  for(int i=0;i<5;i++){
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(EN_PIN[i], OUTPUT);
    digitalWrite(EN_PIN[i], LOW); // enable
    pinMode(LIM_PIN[i], INPUT_PULLUP);
  }

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.advertise(js_pub);

  js.name_length = 5;
  js.position_length = 5;
  static char *names[] = {
    (char*)"joint1", (char*)"joint2", (char*)"joint3",
    (char*)"joint4", (char*)"joint5"
  };
  js.name = names;
}

void stepOnce(int i, bool dir){
  digitalWrite(DIR_PIN[i], dir ? HIGH : LOW);
  digitalWrite(STEP_PIN[i], HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN[i], LOW);
  delayMicroseconds(2);
  step_pos[i] += dir ? 1 : -1;
}

void loop(){
  nh.spinOnce();

  // simple multi-axis stepping (round-robin)
  int active = 0;
  for(int i=0;i<5;i++){
    long diff = target_steps[i] - step_pos[i];
    if(diff != 0){
      bool dir = (diff > 0);

      // stop if homing switch hit while moving negative
      if(!dir && digitalRead(LIM_PIN[i])==LOW){
        step_pos[i]=0; target_steps[i]=0; continue;
      }

      stepOnce(i, dir);
      active++;
    }
  }

  // speed control (global)
  int us = (int)(2000.0 / speed_scale); // tune base speed
  delayMicroseconds(us);

  // publish joint states at ~20 Hz
  static unsigned long last_pub=0;
  if(millis()-last_pub > 50){
    static float pos_rad[5];
    for(int i=0;i<5;i++){
      float deg = step_pos[i]/STEPS_PER_DEG[i];
      pos_rad[i] = deg * M_PI/180.0;
    }
    js.header.stamp = nh.now();
    js.position = pos_rad;
    js_pub.publish(&js);
    last_pub=millis();
  }
}
