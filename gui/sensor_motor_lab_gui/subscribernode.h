#ifndef SUBSCRIBERNODE_H
#define SUBSCRIBERNODE_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "msgs.h"

class SubscriberNode : public QObject
{
  Q_OBJECT
public:
  SubscriberNode() {}
  SubscriberNode(ros::NodeHandle *p_nh): p_nh_(p_nh) {}
  ~SubscriberNode();

public slots:
  void slotStartSubs();

signals:
  void finished();
  void sigStopped();
  void sigMotor1Fb(float);
  void sigSensor1Fb(float);
  void sigDcControlFb(bool);

private:
  ros::NodeHandle *p_nh_;
  ros::Subscriber *motor1_fb_sub_;
  ros::Subscriber *sensor1_fb_sub_;
  ros::Subscriber *dc_position_control_sub_;

  MotorFbMsg motor1_fb_msg_;
  SensorFbMsg sensor1_fb_msg_;
  DcControlFbMsg dc_position_control_msg;

  void setupSubscriber(ros::Subscriber *motor_fb_sub, ros::Subscriber *sensor_fb_sub,
                       std::string motor_fb_topic, std::string sensor_fb_topic,
                       MotorFbMsg &motorFbMsg, SensorFbMsg &sensorFbMsg);
};

#endif // SUBSCRIBERNODE_H
