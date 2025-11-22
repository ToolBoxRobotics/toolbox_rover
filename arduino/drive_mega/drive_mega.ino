#include <ros.h>
#include <rover_msgs/WheelCmd.h>
#include <rover_msgs/SteerCmd.h>
#include <sensor_msgs/JointState.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

ros::NodeHandle nh;

// ---------- PCA9685 ----------
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo channels on PCA9685:
const uint8_t SERVO_CH[4] = {0,1,2,3}; // FL, FR, RL, RR

// Servo pulse mapping (adjust!)
int servo_min = 120;  // ~0 deg
int servo_max = 520;  // ~180 deg
int servo_center = 320;

// ---------- Motors (DRI0002 style PWM+DIR) ----------
const uint8_t PWM_PIN[6] = {5,6,7,8,9,10};
const uint8_t DIR_PIN[6] = {22,23,24,25,26,27};

// ---------- Encoders ----------
const uint8_t ENC_A[6] = {2,3,18,19,20,21};   // interrupt-capable pins
const uint8_t ENC_B[6] = {28,29,30,31,32,33}; // regular pins
volatile long enc_counts[6] = {0,0,0,0,0,0};

// ticks per rev (set!)
const float TICKS_PER_REV = 1024.0;

// For velocity estimation
long last_counts[6] = {0};
unsigned long last_vel_ms = 0;
float wheel_pos_rad[6] = {0};
float wheel_vel_rad_s[6] = {0};

sensor_msgs::JointState wheel_state_msg;
ros::Publisher wheel_state_pub("/rover/wheel_states", &wheel_state_msg);

// ---------- Command storage ----------
float target_rpm[6] = {0};
float target_steer_deg[4] = {0};

// ---------- PID ----------
float kp=0.8, ki=0.1, kd=0.0;
float integ[6]={0};
float last_err[6]={0};

// ---------- Encoder ISRs ----------
void isrEnc0(){ handleEnc(0); }
void isrEnc1(){ handleEnc(1); }
void isrEnc2(){ handleEnc(2); }
void isrEnc3(){ handleEnc(3); }
void isrEnc4(){ handleEnc(4); }
void isrEnc5(){ handleEnc(5); }

inline void handleEnc(uint8_t i){
  bool A = digitalRead(ENC_A[i]);
  bool B = digitalRead(ENC_B[i]);
  enc_counts[i] += (A==B) ? 1 : -1; // standard quadrature
}

// ---------- ROS callbacks ----------
void wheelCmdCb(const rover_msgs::WheelCmd& msg){
  if(msg.wheel_rpm_length < 6) return;
  for(int i=0;i<6;i++) target_rpm[i]=msg.wheel_rpm[i];
}

void steerCmdCb(const rover_msgs::SteerCmd& msg){
  if(msg.steer_deg_length < 4) return;
  for(int i=0;i<4;i++) target_steer_deg[i]=msg.steer_deg[i];
}

ros::Subscriber<rover_msgs::WheelCmd> wheel_sub("/rover/wheel_cmd", &wheelCmdCb);
ros::Subscriber<rover_msgs::SteerCmd> steer_sub("/rover/steer_cmd", &steerCmdCb);

// ---------- Helpers ----------
int degToPulse(float deg){
  // deg around center; clamp to +/-35 typical
  float abs_deg = deg + 90.0; // map -90..+90 to 0..180
  abs_deg = constrain(abs_deg, 0.0, 180.0);
  int pulse = map((int)abs_deg, 0, 180, servo_min, servo_max);
  return pulse;
}

void setServo(uint8_t idx, float deg){
  int pulse = degToPulse(deg);
  pca.setPWM(SERVO_CH[idx], 0, pulse);
}

void setMotor(uint8_t i, float rpm_cmd){
  // rpm_cmd signed
  bool dir = (rpm_cmd >= 0);
  digitalWrite(DIR_PIN[i], dir ? HIGH : LOW);

  // naive feedforward PWM scale:
  float pwm = fabs(rpm_cmd) / 120.0 * 255.0; // 120rpm -> full
  pwm = constrain(pwm, 0, 255);
  analogWrite(PWM_PIN[i], (int)pwm);
}

void setup(){
  // Motors
  for(int i=0;i<6;i++){
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
  }

  // Encoders
  for(int i=0;i<6;i++){
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrEnc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrEnc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrEnc3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[4]), isrEnc4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[5]), isrEnc5, CHANGE);

  // PCA9685
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  nh.initNode();
  nh.subscribe(wheel_sub);
  nh.subscribe(steer_sub);
  nh.advertise(wheel_state_pub);

  wheel_state_msg.name_length = 6;
  wheel_state_msg.position_length = 6;
  wheel_state_msg.velocity_length = 6;
  static char *names[] = {
    (char*)"wheel_fl", (char*)"wheel_fr", (char*)"wheel_ml",
    (char*)"wheel_mr", (char*)"wheel_rl", (char*)"wheel_rr"
  };
  wheel_state_msg.name = names;

  last_vel_ms = millis();
}

void loop(){
  nh.spinOnce();

  // --- Steering update (open-loop) ---
  for(int i=0;i<4;i++){
    setServo(i, target_steer_deg[i]);
  }

  // --- Velocity estimation ---
  unsigned long now = millis();
  float dt = (now - last_vel_ms)/1000.0;
  if(dt >= 0.05){ // 20 Hz
    noInterrupts();
    long counts[6];
    for(int i=0;i<6;i++) counts[i]=enc_counts[i];
    interrupts();

    for(int i=0;i<6;i++){
      long dc = counts[i] - last_counts[i];
      last_counts[i] = counts[i];

      float rev = dc / TICKS_PER_REV;
      float rad = rev * 2.0 * M_PI;
      wheel_pos_rad[i] += rad;
      wheel_vel_rad_s[i] = rad/dt;
    }
    last_vel_ms = now;

    wheel_state_msg.header.stamp = nh.now();
    wheel_state_msg.position = wheel_pos_rad;
    wheel_state_msg.velocity = wheel_vel_rad_s;
    wheel_state_pub.publish(&wheel_state_msg);
  }

  // --- PID motor control ---
  // Convert vel rad/s to rpm:
  for(int i=0;i<6;i++){
    float rpm_meas = wheel_vel_rad_s[i] * 60.0/(2.0*M_PI);
    float err = target_rpm[i] - rpm_meas;
    integ[i] += err * dt;
    float deriv = (err - last_err[i]) / dt;
    last_err[i] = err;

    float rpm_cmd = kp*err + ki*integ[i] + kd*deriv + target_rpm[i]; // PI around target
    setMotor(i, rpm_cmd);
  }

  delay(5);
}
