#include <OpenCM904.h>
#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

 
// Dynamixel Setup
#if defined(ARDUINO_OpenCM904)
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22;
#else
  #error "Unsupported board. Please use OpenCM904 or modify the code for your board."
#endif
 
const float DXL_PROTOCOL_VERSION = 2.0;
 
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
 
const uint8_t MOTOR1_ID = 1;
const uint8_t MOTOR2_ID = 2;
const uint8_t MOTOR3_ID = 3;
 
ros::NodeHandle nh;
 
std_msgs::Float64 motor_pos1_msg; 
std_msgs::Float64 motor_pos2_msg; 
std_msgs::Float64 motor_pos3_msg;
 
std_msgs::Float64 motor_vel1_msg; 
std_msgs::Float64 motor_vel2_msg; 
std_msgs::Float64 motor_vel3_msg;
 
void pwm1(const std_msgs::Float64 &msg);
void pwm2(const std_msgs::Float64 &msg);
void pwm3(const std_msgs::Float64 &msg);
 
ros::Subscriber<std_msgs::Float64> sub_pwm1("/pwm1", pwm1);
ros::Subscriber<std_msgs::Float64> sub_pwm2("/pwm2", pwm2);
ros::Subscriber<std_msgs::Float64> sub_pwm3("/pwm3", pwm3);
 
ros::Publisher motor_pos1_pub("/q1", &motor_pos1_msg);
ros::Publisher motor_pos2_pub("/q2", &motor_pos2_msg);
ros::Publisher motor_pos3_pub("/q3", &motor_pos3_msg);
 
ros::Publisher motor_vel1_pub("/qdot1", &motor_vel1_msg);
ros::Publisher motor_vel2_pub("/qdot2", &motor_vel2_msg);
ros::Publisher motor_vel3_pub("/qdot3", &motor_vel3_msg);
 
int32_t initial_position1 = 0.0;
int32_t initial_position2 = 0.0;
int32_t initial_position3 = 0.0;
 
void setup() {
  DEBUG_SERIAL.begin(115200);
  DXL_SERIAL.begin(57600);
 
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(MOTOR1_ID);
  dxl.ping(MOTOR2_ID);
  dxl.ping(MOTOR3_ID);
 
  dxl.torqueOff(MOTOR1_ID);
  dxl.torqueOff(MOTOR2_ID);
  dxl.torqueOff(MOTOR3_ID);
 
  dxl.setOperatingMode(MOTOR1_ID, OP_PWM);
  dxl.setOperatingMode(MOTOR2_ID, OP_PWM);
  dxl.setOperatingMode(MOTOR3_ID, OP_PWM);
 
  dxl.torqueOn(MOTOR1_ID);
  dxl.torqueOn(MOTOR2_ID);
  dxl.torqueOn(MOTOR3_ID);
 
  initial_position1 = dxl.getPresentPosition(MOTOR1_ID, UNIT_DEGREE);
  initial_position2 = dxl.getPresentPosition(MOTOR2_ID, UNIT_DEGREE);
  initial_position3 = dxl.getPresentPosition(MOTOR3_ID, UNIT_DEGREE);
 
  nh.initNode();
  nh.subscribe(sub_pwm1);
  nh.subscribe(sub_pwm2);
  nh.subscribe(sub_pwm3);
  nh.advertise(motor_pos1_pub);
  nh.advertise(motor_pos2_pub);
  nh.advertise(motor_pos3_pub);
  nh.advertise(motor_vel1_pub);
  nh.advertise(motor_vel2_pub);
  nh.advertise(motor_vel3_pub);
}
 
void loop() {
  nh.spinOnce();
  int32_t pose11 = dxl.getPresentPosition(MOTOR1_ID, UNIT_DEGREE) - initial_position1;
  int32_t pose22 = dxl.getPresentPosition(MOTOR2_ID, UNIT_DEGREE) - initial_position2;
  int32_t pose33 = dxl.getPresentPosition(MOTOR3_ID, UNIT_DEGREE) - initial_position3;
  int32_t vel11 = dxl.getPresentVelocity(MOTOR1_ID, UNIT_RPM);
  int32_t vel22 = dxl.getPresentVelocity(MOTOR2_ID, UNIT_RPM);
  int32_t vel33 = dxl.getPresentVelocity(MOTOR3_ID, UNIT_RPM);

  float pose1 = static_cast<float>(pose11);
  float pose2 = static_cast<float>(pose22);
  float pose3 = static_cast<float>(pose33);

  float vel1 = static_cast<float>(vel11);
  float vel2 = static_cast<float>(vel22);
  float vel3 = static_cast<float>(vel33);
  
  motor_pos1_msg.data = (pose1 * 3.141592 / 180.0);
  motor_pos2_msg.data = (pose2 * 3.141592 / 180.0);
  motor_pos3_msg.data = (pose3 * 3.141592 / 180.0);
  motor_pos1_pub.publish(&motor_pos1_msg);
  motor_pos2_pub.publish(&motor_pos2_msg);
  motor_pos3_pub.publish(&motor_pos3_msg);
 
  motor_vel1_msg.data = (vel1 * 2 * 3.141592 / 60);
  motor_vel2_msg.data = (vel2 * 2 * 3.141592 / 60);
  motor_vel3_msg.data = (vel3 * 2 * 3.141592 / 60);
  motor_vel1_pub.publish(&motor_vel1_msg);
  motor_vel2_pub.publish(&motor_vel2_msg);
  motor_vel3_pub.publish(&motor_vel3_msg);
 
  delay(10); 
}
 
void pwm1(const std_msgs::Float64 &msg) {
  int32_t pwm_value1 = static_cast<int32_t>(msg.data);
  dxl.setGoalPWM(MOTOR1_ID, pwm_value1);
}
 
void pwm2(const std_msgs::Float64 &msg) {
  int32_t pwm_value2 = static_cast<int32_t>(msg.data);
  dxl.setGoalPWM(MOTOR2_ID, pwm_value2);
}
 
void pwm3(const std_msgs::Float64 &msg) {
  int32_t pwm_value3 = static_cast<int32_t>(msg.data);
  dxl.setGoalPWM(MOTOR3_ID, pwm_value3);
} 
