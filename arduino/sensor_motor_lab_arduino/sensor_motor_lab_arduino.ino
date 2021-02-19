#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <Servo.h> 
#include <math.h>

#define SERVOPIN 11       //Servo (PWM) writing
#define LIGHTSENSORPIN A1 //Ambient light sensor reading
#define FLEXSENSORPIN A0 //Pin in for flex meter signal
#define STEPPERPIN 3  //pin out for stepper step command
#define STEPPERDIRPIN 2 //pin out for stepper direction command
#define BUTTONPIN 4  // pin in for button

ros::NodeHandle nh;
 
struct abstractMotorSensorPair {
  virtual void setup() = 0; // single setup function when switching between motors
  virtual void run() = 0; // single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;  // returns motor state
  virtual float sensorFeedback() = 0; // return sensor state

  virtual void processGuiCommand(float cmd) = 0; // takes an input command from the gui
};


struct lightServoPair : abstractMotorSensorPair {
  Servo myservo; 
  bool inc = 1; //  If inc == 1, servo is going from 0 to 180 degrees; reverse if inc == 0
  unsigned long timer;
  const float max_vel = 315.78948;  //  max velocity of servo is 315.79 degrees per second
  float pos_float = 0.0;  //  position of servo from 0 to 180
  float filter_read = 0.0;  //  filtered ambient light value
  const float filter_percent = 0.1; //  percent of prveious value used in filtered ambient light value
  float gain = 1.0;
  
  void setup() {
    pinMode(LIGHTSENSORPIN,  INPUT);  
    myservo.attach(SERVOPIN);
    myservo.write(0); //  Set servo position to 0 degrees
    timer = millis();
  }

  void processGuiCommand(float command){
    gain = command;
  }

  float motorFeedback() {
    //  Returns angle of the servo, from 0 to 180 degrees
    return myservo.read();
  }

  float sensorFeedback() {
    //  Returns sensor illuminance in lux(lx)
    return analogRead(LIGHTSENSORPIN);
  }

  void run(){
    float reading = analogRead(LIGHTSENSORPIN); //  Read light level

    filter_read = filter_percent*filter_read+(1-filter_percent)*reading;
    float ratio = gain * filter_read / 1023.0; // Get percent of maximum value (1023)
    float vel = ratio * max_vel; // Get servo velocity based on ratio
  
    if (pos_float <= 0) {
      inc = 1;
    } else if (pos_float >= 180) {
      inc = 0;
    }
    
    if (inc == 1) { 
        float delta = vel*(millis()-timer)/1000;
        pos_float += delta;
        myservo.write((int)pos_float);
    } else {
        float delta = vel*(millis()-timer)/1000;
        pos_float -= delta;
        myservo.write(pos_float);
    }
  
    timer = millis();
  }
};

struct flexStepperPair : abstractMotorSensorPair {
  const float vcc = 5.05; //  voltage output from the arduino  
  const float rDiv = 47500; // Measured resistance on the voltage devider resistor
  const float rStraight = 22540;  //  resistance of the flex meter when straight
  const float rBend = 45000;  //  resistance of the flex meter when bent at 90degrees
  
  const float motorStep = 0.9;  //  degrees
  
  float motorState = 0; //  starting motor state value (degrees)
  float sensorState = 0;  //  starting flex state value (will be immediately overwritten

  const float emaAlpha = 2.0/(1 + 40);  //  exponential moving average weighting term
  float avFlexAngle = 0;  // filtered flex angle
  float gain = 1.0;
  
  void setup() {
    //set pin IO
    pinMode(FLEXSENSORPIN, INPUT);
    pinMode(STEPPERPIN, OUTPUT);
    pinMode(STEPPERDIRPIN, OUTPUT);
  }

  void processGuiCommand(float command){
    gain = command;
  }

  float motorFeedback() {
    return motorState;
  }

  float sensorFeedback() {
    return sensorState;
  }

  void run(){
    //read the flex sensor
    int flexIn = analogRead(FLEXSENSORPIN);
    float flexV = flexIn * vcc / 1023.0;
    float flexR = rDiv * (vcc / flexV - 1.0);
    sensorState = map(flexR, rStraight, rBend, 0, 90.0);

    avFlexAngle = (emaAlpha * sensorState) + (1.0 - emaAlpha) * avFlexAngle;

    //update the stepper motor
    if (abs(motorState - sensorState) > 2 * motorStep) {
      if (sensorState < motorState) {
        digitalWrite(STEPPERDIRPIN, HIGH);
        digitalWrite(STEPPERPIN, HIGH);
        delayMicroseconds(4000);
        digitalWrite(STEPPERPIN, LOW);
        motorState -= motorStep;
      } else {
        digitalWrite(STEPPERDIRPIN, LOW);
        digitalWrite(STEPPERPIN, HIGH);
        delayMicroseconds(4000);
        digitalWrite(STEPPERPIN, LOW);
        motorState += motorStep;
      }
    }
    delayMicroseconds(4000);    
  }
};

class SensorMotorLab {
public:
  SensorMotorLab() :
  motor_cmd_sub("motor_cmd", &SensorMotorLab::motor_cmd_cb, this),
  motor_selection_sub("motor_selection", &SensorMotorLab::motor_selection_cb, this),
  motor_fb_pub("motor_fb", &motor_fb_msg),
  sensor_fb_pub("sensor_fb", &sensor_fb_msg)
  {}

  void init(ros::NodeHandle& nh) {
    flexStepperObj.setup();
    
    nh.advertise(motor_fb_pub);
    nh.advertise(sensor_fb_pub);
  
    nh.subscribe(motor_cmd_sub);
    nh.subscribe(motor_selection_sub);
  }

  void run() {
    switch (curr_motor) {
      case motorSelection::motor1:
        sensorMotorPairRun(flexStepperObj);
        break;
      case motorSelection::motor2:
        sensorMotorPairRun(lightServoObj);
        break;
      case motorSelection::motor3:
        // do smth
        break;
    }
    motor_fb_pub.publish(&motor_fb_msg);
    sensor_fb_pub.publish(&sensor_fb_msg);

    if (readButtonWithDebounce(BUTTONPIN, &buttonState, &prevButtonState, &lastDebounceTime)) {
      // do smth
    }
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
  lightServoPair lightServoObj;

  int buttonState, prevButtonState = LOW;
  unsigned long lastDebounceTime = 0;
  
  bool readButtonWithDebounce(const int BUTTON_PIN, int *buttonState, int *lastButtonState, unsigned long *lastDebounceTime) {
    bool output = false;
    
    int reading = digitalRead(BUTTON_PIN);
  
    if (reading != *lastButtonState)
    {
      *lastDebounceTime = millis();
    }
  
    if ((millis() - *lastDebounceTime) > 50)  // 50 ms debounce delay 
    {
      if (reading != *buttonState) 
      {
        *buttonState = reading;
        if (*buttonState == HIGH) 
        {
          output = true;
        }
      }
    }
  
    *lastButtonState = reading;
    return output;
  }

  void motor_cmd_cb(const std_msgs::Float32& msg) {
    flexStepperObj.processGuiCommand(msg.data);
  }

  void motor_selection_cb(const std_msgs::Int8& msg) {
    curr_motor = msg.data;
    switch (curr_motor) {
      case motorSelection::motor1:
        flexStepperObj.setup();
        break;
      case motorSelection::motor2:
        lightServoObj.setup();
        break;
      case motorSelection::motor3:
        // do smth
        break;
    }
  }

  template <class T>
  void sensorMotorPairRun(T obj) {
    obj.run();
    motor_fb_msg.data = obj.motorFeedback();
    sensor_fb_msg.data = obj.sensorFeedback();
  }
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
