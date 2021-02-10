#include "publishernode.h"

PublisherNode::PublisherNode(ros::NodeHandle *p_nh): p_nh_(p_nh) {
  p_cmd_pub_ = new ros::Publisher();
  *p_cmd_pub_ = p_nh_->advertise<std_msgs::Float32>("/motor_cmd", 10, true);
}

void PublisherNode::slotPubCmd(float cmd) {
  cmd_msg_.data = cmd;
  p_cmd_pub_->publish(cmd_msg_);
}

PublisherNode::~PublisherNode() {
  ros::shutdown();
  delete p_nh_;
  delete p_cmd_pub_;
  emit finished();
}
