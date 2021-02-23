#include "subscribernode.h"

void SubscriberNode::slotStartSubs()
{
  int argc = 0; char **argv = nullptr;
  ros::init(argc, argv, "subscriber_node");

  // initialise subscriber
  setupSubscriber(motor1_fb_sub_, sensor1_fb_sub_, "motor_fb", "sensor_fb", motor1_fb_msg_, sensor1_fb_msg_);
  dc_position_control_sub_ = new ros::Subscriber();
  *dc_position_control_sub_ = p_nh_->subscribe("dc_position_control", 1, &DcControlFbMsg::FbCallback, &dc_position_control_msg);

  // run ROS
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    emit sigMotor1Fb(motor1_fb_msg_.motor_fb);
    emit sigSensor1Fb(sensor1_fb_msg_.sensor_fb);
    emit sigDcControlFb(dc_position_control_msg.msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SubscriberNode::setupSubscriber(ros::Subscriber *motor_fb_sub, ros::Subscriber *sensor_fb_sub,
                     std::string motor_fb_topic, std::string sensor_fb_topic,
                     MotorFbMsg &motorFbMsg, SensorFbMsg &sensorFbMsg) {
  motor_fb_sub = new ros::Subscriber();
  *motor_fb_sub = p_nh_->subscribe(motor_fb_topic, 1, &MotorFbMsg::motorFbCallback, &motorFbMsg);
  sensor_fb_sub = new ros::Subscriber();
  *sensor_fb_sub = p_nh_->subscribe(sensor_fb_topic, 1, &SensorFbMsg::sensorFbCallback, &sensorFbMsg);
}

SubscriberNode::~SubscriberNode() {
  ros::shutdown();
  delete p_nh_;
  delete motor1_fb_sub_;
  delete sensor1_fb_sub_;
  emit finished();
}
