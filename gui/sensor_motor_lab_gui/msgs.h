#ifndef MSGS_H
#define MSGS_H

#include <std_msgs/Float32.h>

struct MotorFbMsg {
  float motor_fb;
  MotorFbMsg(): motor_fb(0.0) {}
  void motorFbCallback(const std_msgs::Float32::ConstPtr& motor_fb_msg) {
    motor_fb = motor_fb_msg->data;
  }
};

struct SensorFbMsg {
  float sensor_fb;
  SensorFbMsg(): sensor_fb(0.0) {}
  void sensorFbCallback(const std_msgs::Float32::ConstPtr& sensor_fb_msg) {
    sensor_fb = sensor_fb_msg->data;
  }
};

#endif // MSGS_H
