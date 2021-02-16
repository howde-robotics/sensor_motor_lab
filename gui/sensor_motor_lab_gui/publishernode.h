#ifndef PUBLISHERNODE_H
#define PUBLISHERNODE_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

class PublisherNode : public QObject
{
  Q_OBJECT
public:
  PublisherNode(ros::NodeHandle *p_nh);
  ~PublisherNode();

signals:
  void finished();

public slots:
  void slotPubCmd1(float);
  void slotPubMotorSelection(int);

private:
  ros::NodeHandle *p_nh_;
  ros::Publisher *p_cmd1_pub_, *p_motor_selection_pub_;

  std_msgs::Float32 cmd_msg_;
  std_msgs::Int8 motor_selection_msg_;
};

#endif // PUBLISHERNODE_H
