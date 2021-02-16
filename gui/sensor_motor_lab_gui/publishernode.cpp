#include "publishernode.h"

PublisherNode::PublisherNode(ros::NodeHandle *p_nh): p_nh_(p_nh) {
  p_cmd1_pub_ = new ros::Publisher();
  p_motor_selection_pub_ = new ros::Publisher();

  *p_cmd1_pub_ = p_nh_->advertise<std_msgs::Float32>("/motor_cmd", 10, true);
  *p_motor_selection_pub_ = p_nh_->advertise<std_msgs::Int8>("/motor_selection", 1, true);
}

void PublisherNode::slotPubCmd1(float cmd) {
  cmd_msg_.data = cmd;
  p_cmd1_pub_->publish(cmd_msg_);
}

void PublisherNode::slotPubMotorSelection(int msg) {
  std::cout << "hello";
  motor_selection_msg_.data = msg;
  p_motor_selection_pub_->publish(motor_selection_msg_);
}

PublisherNode::~PublisherNode() {
  ros::shutdown();
  delete p_nh_;
  delete p_cmd1_pub_;
  delete p_motor_selection_pub_;
  emit finished();
}
