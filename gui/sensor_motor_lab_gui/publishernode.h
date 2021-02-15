#ifndef PUBLISHERNODE_H
#define PUBLISHERNODE_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

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
  void slotPubCmd2(float);
  void slotPubCmd3(float);

private:
  ros::NodeHandle *p_nh_;
  ros::Publisher *p_cmd1_pub_, *p_cmd2_pub_, *p_cmd3_pub_;

  std_msgs::Float32 cmd_msg_;
};

#endif // PUBLISHERNODE_H
