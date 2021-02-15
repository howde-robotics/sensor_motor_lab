#include "publishernode.h"

PublisherNode::PublisherNode(ros::NodeHandle *p_nh): p_nh_(p_nh) {
  p_cmd1_pub_ = new ros::Publisher();
  p_cmd2_pub_ = new ros::Publisher();
  p_cmd3_pub_ = new ros::Publisher();
  *p_cmd1_pub_ = p_nh_->advertise<std_msgs::Float32>("/motor1_cmd", 10, true);
  *p_cmd2_pub_ = p_nh_->advertise<std_msgs::Float32>("/motor2_cmd", 10, true);
  *p_cmd3_pub_ = p_nh_->advertise<std_msgs::Float32>("/motor3_cmd", 10, true);
}

void PublisherNode::slotPubCmd1(float cmd) {
  cmd_msg_.data = cmd;
  p_cmd1_pub_->publish(cmd_msg_);
}

void PublisherNode::slotPubCmd2(float cmd) {
  cmd_msg_.data = cmd;
  p_cmd2_pub_->publish(cmd_msg_);
}

void PublisherNode::slotPubCmd3(float cmd) {
  cmd_msg_.data = cmd;
  p_cmd3_pub_->publish(cmd_msg_);
}

PublisherNode::~PublisherNode() {
  ros::shutdown();
  delete p_nh_;
  delete p_cmd1_pub_;
  delete p_cmd2_pub_;
  delete p_cmd3_pub_;
  emit finished();
}
