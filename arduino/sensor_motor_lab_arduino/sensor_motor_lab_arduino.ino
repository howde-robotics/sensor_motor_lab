#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

class SensorMotorLab {
public:
  SensorMotorLab() :
  motor1_cmd_sub("motor1_cmd", &SensorMotorLab::motor1_cmd_cb, this),
  motor2_cmd_sub("motor2_cmd", &SensorMotorLab::motor2_cmd_cb, this),
  motor3_cmd_sub("motor3_cmd", &SensorMotorLab::motor3_cmd_cb, this),
  motor1_fb_pub("motor1_fb", &motor1_fb_msg),
  sensor1_fb_pub("sensor1_fb", &sensor1_fb_msg),
  motor2_fb_pub("motor2_fb", &motor2_fb_msg),
  sensor2_fb_pub("sensor2_fb", &sensor2_fb_msg),
  motor3_fb_pub("motor3_fb", &motor3_fb_msg),
  sensor3_fb_pub("sensor3_fb", &sensor3_fb_msg)
  {}

  void init(ros::NodeHandle& nh) {
    nh.advertise(motor1_fb_pub);
    nh.advertise(sensor1_fb_pub);
    nh.advertise(motor2_fb_pub);
    nh.advertise(sensor2_fb_pub);
    nh.advertise(motor3_fb_pub);
    nh.advertise(sensor3_fb_pub);
  
    nh.subscribe(motor1_cmd_sub);
    nh.subscribe(motor1_cmd_sub);
    nh.subscribe(motor1_cmd_sub);
  }

  void run() {
    
  }

  void motor1_cmd_cb(const std_msgs::Float32& msg) {
  
  }
  
  void motor2_cmd_cb(const std_msgs::Float32& msg) {
    
  }
  
  void motor3_cmd_cb(const std_msgs::Float32& msg) {
    
  }
private:
  std_msgs::Float32 motor1_fb_msg, sensor1_fb_msg,
                    motor2_fb_msg, sensor2_fb_msg,
                    motor3_fb_msg, sensor3_fb_msg;

  ros::Publisher motor1_fb_pub;
  ros::Publisher sensor1_fb_pub;
  ros::Publisher motor2_fb_pub;
  ros::Publisher sensor2_fb_pub;
  ros::Publisher motor3_fb_pub;
  ros::Publisher sensor3_fb_pub;

  ros::Subscriber<std_msgs::Float32, SensorMotorLab> motor1_cmd_sub;
  ros::Subscriber<std_msgs::Float32, SensorMotorLab> motor2_cmd_sub;
  ros::Subscriber<std_msgs::Float32, SensorMotorLab> motor3_cmd_sub;
};

SensorMotorLab node;

void setup() {
  Serial.begin(9600);
  nh.initNode();
  node.init(nh);
}

void loop() {
  node.run();
  nh.spinOnce();
}
