#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

struct abstractMotorSensorPair {
  virtual void setup() = 0;
  virtual void run() = 0;//single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;//returns motor state 0 to 1
  virtual float sensorFeedback() = 0;//return sensor state 0 to 1

  virtual void processGuiCommand(float cmd) = 0;
};

class SensorMotorLab {
public:
  SensorMotorLab() :
  motor_cmd_sub("motor_cmd", &SensorMotorLab::motor1_cmd_cb, this),
  motor_selection_sub("motor_selection", &SensorMotorLab::motor_selection_cb, this),
  motor_fb_pub("motor_fb", &motor_fb_msg),
  sensor_fb_pub("sensor_fb", &sensor_fb_msg)
  {}

  void init(ros::NodeHandle& nh) {
    flexStepperObj.setup();
    
    nh.advertise(motor_fb_pub);
    nh.advertise(sensor_fb_pub);
  
    nh.subscribe(motor_cmd_sub);
  }

  void run() {
    switch (curr_motor) {
      case motorSelection::motor1:
        flexStepperObj.run();
        motor_fb_msg.data = flexStepperObj.motorFeedback();
        sensor_fb_msg.data = flexStepperObj.sensorFeedback();
        motor_fb_pub.publish(&motor_fb_msg);
        sensor_fb_pub.publish(&sensor_fb_msg);
        break;
      case motorSelection::motor2:
        // do smth
        break;
      case motorSelection::motor3:
        // do smth
        break;
    }
  }

  void motor1_cmd_cb(const std_msgs::Float32& msg) {
    flexStepperObj.processGuiCommand(msg.data);
  }
  
  void motor2_cmd_cb(const std_msgs::Float32& msg) {
    
  }
  
  void motor3_cmd_cb(const std_msgs::Float32& msg) {
    
  }

  void motor_selection_cb(const std_msgs::Int8& msg) {
    curr_motor = msg.data;
  }


private:
  std_msgs::Float32 motor_fb_msg, sensor_fb_msg;

  enum motorSelection {
    motor1,
    motor2,
    motor3
  };

  short int curr_motor = motorSelection::motor1;

  ros::Publisher motor_fb_pub;
  ros::Publisher sensor_fb_pub;

  ros::Subscriber<std_msgs::Float32, SensorMotorLab> motor_cmd_sub;
  ros::Subscriber<std_msgs::Int8, SensorMotorLab> motor_selection_sub;

  flexStepperPair flexStepperObj;
};


SensorMotorLab node;

void setup() {
  nh.initNode();
  node.init(nh);
}

void loop() {
  node.run();
  nh.spinOnce();
}
