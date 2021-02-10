#include "subscribernode.h"

void SubscriberNode::slotStartSubs()
{
  int argc = 0; char **argv = nullptr;
  ros::init(argc, argv, "subscriber_node");

  // initialise subscriber
  motor_fb_sub_ = new ros::Subscriber();
  *motor_fb_sub_ = p_nh_->subscribe("motor_fb", 10, &MotorFbMsg::motorFbCallback, &motor_fb_msg_);
  sensor_fb_sub_ = new ros::Subscriber();
  *sensor_fb_sub_ = p_nh_->subscribe("sensor_fb", 10, &SensorFbMsg::sensorFbCallback, &sensor_fb_msg_);

  // run ROS
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    emit sigMotorFb(motor_fb_msg_.motor_fb);
    emit sigSensorFb(sensor_fb_msg_.sensor_fb);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

SubscriberNode::~SubscriberNode() {
  ros::shutdown();
  delete p_nh_;
  delete motor_fb_sub_;
  delete sensor_fb_sub_;
  emit finished();
}
