#include "subscribernode.h"

void SubscriberNode::slotStartSubs()
{
  int argc = 0; char **argv = nullptr;
  ros::init(argc, argv, "subscriber_node");

  // initialise subscriber
  setupSubscriber(motor1_fb_sub_, sensor1_fb_sub_, "motor_fb", "sensor_fb", motor1_fb_msg_, sensor1_fb_msg_);

  // run ROS
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    emit sigMotor1Fb(motor1_fb_msg_.motor_fb);
    emit sigSensor1Fb(sensor1_fb_msg_.sensor_fb);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SubscriberNode::setupSubscriber(ros::Subscriber *motor_fb_sub, ros::Subscriber *sensor_fb_sub,
                     std::string motor_fb_topic, std::string sensor_fb_topic,
                     MotorFbMsg &motorFbMsg, SensorFbMsg &sensorFbMsg) {
  motor_fb_sub = new ros::Subscriber();
  *motor_fb_sub = p_nh_->subscribe(motor_fb_topic, 100, &MotorFbMsg::motorFbCallback, &motorFbMsg);
  sensor_fb_sub = new ros::Subscriber();
  *sensor_fb_sub = p_nh_->subscribe(sensor_fb_topic, 100, &SensorFbMsg::sensorFbCallback, &sensorFbMsg);
}

SubscriberNode::~SubscriberNode() {
  ros::shutdown();
  delete p_nh_;
  delete motor1_fb_sub_;
  delete sensor1_fb_sub_;
  emit finished();
}
