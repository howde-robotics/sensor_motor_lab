#include "subscribernode.h"

void SubscriberNode::slotStartSubs()
{
  int argc = 0; char **argv = nullptr;
  ros::init(argc, argv, "subscriber_node");

  // initialise subscriber
  setupSubscriber(motor1_fb_sub_, sensor1_fb_sub_, "motor1_fb", "sensor1_fb", motor1_fb_msg_, sensor1_fb_msg_);
  setupSubscriber(motor2_fb_sub_, sensor2_fb_sub_, "motor2_fb", "sensor2_fb", motor2_fb_msg_, sensor2_fb_msg_);
  setupSubscriber(motor3_fb_sub_, sensor3_fb_sub_, "motor3_fb", "sensor3_fb", motor3_fb_msg_, sensor3_fb_msg_);

  // run ROS
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    emit sigMotor1Fb(motor1_fb_msg_.motor_fb);
    emit sigSensor1Fb(sensor1_fb_msg_.sensor_fb);
    emit sigMotor2Fb(motor2_fb_msg_.motor_fb);
    emit sigSensor2Fb(sensor2_fb_msg_.sensor_fb);
    emit sigMotor3Fb(motor3_fb_msg_.motor_fb);
    emit sigSensor3Fb(sensor3_fb_msg_.sensor_fb);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SubscriberNode::setupSubscriber(ros::Subscriber *motor_fb_sub, ros::Subscriber *sensor_fb_sub,
                     std::string motor_fb_topic, std::string sensor_fb_topic,
                     MotorFbMsg &motorFbMsg, SensorFbMsg &sensorFbMsg) {
  motor_fb_sub = new ros::Subscriber();
  *motor_fb_sub = p_nh_->subscribe(motor_fb_topic, 10, &MotorFbMsg::motorFbCallback, &motorFbMsg);
  sensor_fb_sub = new ros::Subscriber();
  *sensor_fb_sub = p_nh_->subscribe(sensor_fb_topic, 10, &SensorFbMsg::sensorFbCallback, &sensorFbMsg);
}

SubscriberNode::~SubscriberNode() {
  ros::shutdown();
  delete p_nh_;
  delete motor1_fb_sub_;
  delete sensor1_fb_sub_;
  delete motor2_fb_sub_;
  delete sensor2_fb_sub_;
  delete motor3_fb_sub_;
  delete sensor3_fb_sub_;
  emit finished();
}
