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
  void sigMotorFb(float);
  void sigSensorFb(float);

private:
  ros::NodeHandle *p_nh_;
  ros::Subscriber *motor_fb_sub_;
  ros::Subscriber *sensor_fb_sub_;

  MotorFbMsg motor_fb_msg_;
  SensorFbMsg sensor_fb_msg_;
};

#endif // SUBSCRIBERNODE_H
