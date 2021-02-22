#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <Servo.h> 
#include <math.h>
#include <Encoder.h>
#include <std_msgs/Bool.h>

#define SERVOPIN 11       //Servo (PWM) writing
#define LIGHTSENSORPIN A1 //Ambient light sensor reading
#define FLEXSENSORPIN A0 //Pin in for flex meter signal
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
    float ratio = filter_read / 1023.0; // Get percent of maximum value (1023)
    float vel = gain * ratio * max_vel; // Get servo velocity based on ratio
  
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
  const float vcc = 5.05;//voltage output from the arduino  
  const float rDiv = 47500;//Measured resistance on the voltage devider resistor
  const float rStraight = 22540;//resistance of the flex meter when straight
  const float rBend = 45000;//resistance of the flex meter when bent at 90degrees
  
  const float motorStep = 0.9;//degrees
  
  const int flexPin = A0;//Pin in for flex meter signal
  const int  stepPin = 8;//pin out for stepper step command
  const int dirPin = 7;//pin out for stepper direction command
  
  float motorState = 0;//starting motor state value (degrees)
  float sensorState = 0;//starting flex state value (will be immediately overwritten

  const float emaAlpha = 2.0/(1 + 40);//exponential moving average weighting term
  float avFlexAngle = 0;
  float gain = 1.0;
  
  void setup() {
    //set pin IO
    pinMode(flexPin, INPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void processGuiCommand(float command){
    gain = command;
  }

  float motorFeedback() {
    return motorState;
  }

  float sensorFeedback() {
    return avFlexAngle;
  }

  void run(){
    //read the flex sensor
    int flexIn = analogRead(flexPin);
    float flexV = flexIn * vcc / 1023.0;
    float flexR = rDiv * (vcc / flexV - 1.0);
    sensorState = map(flexR, rStraight, rBend,
                     0, 90.0);

    avFlexAngle = (emaAlpha * sensorState) + (1.0 - emaAlpha) * avFlexAngle;

    //update the stepper motor
    if (abs(motorState - sensorState) * gain > 2 * motorStep) {
      if (sensorState < motorState) {
        digitalWrite(dirPin, HIGH);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5000);
        digitalWrite(stepPin, LOW);
        motorState -= motorStep;
      } else {
        digitalWrite(dirPin, LOW);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5000);
        digitalWrite(stepPin, LOW);
        motorState += motorStep;
      }
    }
    
    delayMicroseconds(5000);    
  }
};

struct forceDCPair {
  // pins
//  const int FSR_PIN = A0;
//  const int I1_PIN = 5;
//  const int I2_PIN = 6;
//  const int ENC_A_PIN = 2;
//  const int ENC_B_PIN = 3;
  const int FSR_PIN = A3;
  const int I1_PIN = 5;
  const int I2_PIN = 6;
  const int ENC_A_PIN = 12;
  const int ENC_B_PIN = 13;

  // constant numbers
  const int MS_PER_REV = 674;
  const float VCC = 4.98; // supply voltage
  const float R_DIV = 10000; // resistance in voltage divider
  
  // motor stuff
  long rotorPosition;
  int curTime;
  int timePerRev;
  float revsPerMin;
  bool positionControl = true;

  // sensor stuff
  float appliedForce;

  // pid stuff
  float pGain = 0;
  float dGain = 0;
  float iGain = 0;
  float desiredPosition;
  float desiredRPM;
  float prevErr;
  float errAccumulation;
  float controlSignal;
  float scaleInputFactor = MS_PER_REV / 255;
  float scaleDegreesFactor = MS_PER_REV / 360;
  Encoder * structEnc;

  void togglePositionControl() {
    positionControl = !positionControl;
  }

  void setup(){
    pinMode(FSR_PIN, INPUT);
    pinMode(I1_PIN, OUTPUT);
    pinMode(I2_PIN, OUTPUT);
    Encoder enc(ENC_A_PIN, ENC_B_PIN);
    structEnc = & enc;
    setMotionGains(1, 1, 0, 270, 40, false);
  }

  void run(){
    float reference;
    float current;
  
    calculatePosVel();
    calculateForce();

    
    if (positionControl){
        reference = desiredPosition;
        current = rotorPosition;
        desiredPosition = appliedForce;
    }
    else{
        reference = desiredRPM;
        current = revsPerMin;
        desiredRPM = appliedForce;
    }

    float err = reference - current;
//    Serial.print("Position: ");
//    Serial.println(rotorPosition);
//    Serial.print("RPM: ");
//    Serial.println(revsPerMin);
//    Serial.print("Error: ");
//    Serial.println(err);
    float errDot = err - prevErr;
    errAccumulation += err;
    prevErr = err;

    controlSignal = pGain*err + dGain*errDot + iGain*errAccumulation;
    sendMotorInput(controlSignal);
  }

  
  void calculatePosVel(){
    rotorPosition = abs(structEnc->read());
    
    // calculate RPM
    if (rotorPosition >= MS_PER_REV){
      structEnc->write(0);
      timePerRev = millis() - curTime;
      curTime = millis();
      revsPerMin = 1 / (timePerRev / (1000.0 * 60));
    }
  }
  
  // calculate force from the sensor
  void calculateForce(){
    int frsADC; // the raw analog to digital converter reading
    float frsV; // force resistive sensor voltage
    float frsR; // force resistive sensor resistance  
    
    // read the analog input
    frsADC = analogRead(FSR_PIN);

    if (frsADC != 0){
      frsV = frsADC * VCC / 1023.0; // calculate voltage from raw analog input
      frsR = R_DIV * (VCC / frsV - 1.0); // calculate resistance

      // this sensor graph is logarithmic so I'm breaking it up into several sections
      if (frsV <= 1 && frsV > 0)
      {
        appliedForce = frsV;
      }
      else if (frsV <= 2.5 && frsV > 1)
      {
        appliedForce = 50 * (frsV - 1);
      }
      else if (frsV <= 5 && frsV > 2.5)
      {
        appliedForce = 1200 * (frsV - 2.5);
      }
    }
  }

  void sendMotorInput(float speed){
    int input = (int) speed/scaleInputFactor;
//    Serial.print("Input Signal: ");
//    Serial.println(input);
    
    if (input >= 0){
      if (abs(input) > 10)
      {input = constrain(input, 70, 255);
      analogWrite(I1_PIN, abs(input));
      digitalWrite(I2_PIN, LOW);
      }
      else{
        digitalWrite(I1_PIN, LOW);
        digitalWrite(I2_PIN, LOW);
      }
    }
    else {
      if (abs(input) > 10){
        input = constrain(input, -255, -70);
        digitalWrite(I1_PIN, LOW);
        analogWrite(I2_PIN, abs(input));
      }
      else{
        digitalWrite(I1_PIN, LOW);
        digitalWrite(I2_PIN, LOW);
      }
    }

  }

  void setMotionGains(float Kp, float  Kd, float Ki, float positionDes, float speedDes, bool posCtrl){
    pGain = Kp;
    dGain = Kd;
    iGain = Ki;
    desiredPosition = positionDes;
    desiredRPM = speedDes;
    positionControl = posCtrl;
  }

  float motorFeedback(){
    return rotorPosition;
  }
  float sensorFeedback(){
    return appliedForce;
  }

  void processGuiCommand(float cmd){
    
  }

};

class SensorMotorLab {
public:
  SensorMotorLab() :
  motor_cmd_sub("motor_cmd", &SensorMotorLab::motor_cmd_cb, this),
  motor_selection_sub("motor_selection", &SensorMotorLab::motor_selection_cb, this),
  motor_fb_pub("motor_fb", &motor_fb_msg),
  sensor_fb_pub("sensor_fb", &sensor_fb_msg),
  dc_position_control_pub("dc_position_control", &dc_position_control_msg)
  {}

  void init(ros::NodeHandle& nh) {
    flexStepperObj.setup();
    
    nh.advertise(motor_fb_pub);
    nh.advertise(sensor_fb_pub);
    nh.advertise(dc_position_control_pub);
  
    nh.subscribe(motor_cmd_sub);
    nh.subscribe(motor_selection_sub);
  }

  void run() {
    switch (curr_motor) {
      case motorSelection::motor1:
        flexStepperObj.run();
        motor_fb_msg.data = flexStepperObj.motorFeedback();
        sensor_fb_msg.data = flexStepperObj.sensorFeedback();
        break;
      case motorSelection::motor2:
        lightServoObj.run();
        motor_fb_msg.data = lightServoObj.motorFeedback();
        sensor_fb_msg.data = lightServoObj.sensorFeedback();
        break;
      case motorSelection::motor3:
        forceDCObj.run();
        motor_fb_msg.data = forceDCObj.motorFeedback();
        sensor_fb_msg.data = forceDCObj.sensorFeedback();
        break;
    }
    motor_fb_pub.publish(&motor_fb_msg);
    sensor_fb_pub.publish(&sensor_fb_msg);

    if (readButtonWithDebounce(BUTTONPIN, &buttonState, &prevButtonState, &lastDebounceTime)) {
      forceDCObj.togglePositionControl();
      dc_position_control_msg.data = forceDCObj.positionControl;
      dc_position_control_pub.publish(&dc_position_control_msg);
    }
  }

private:
  std_msgs::Float32 motor_fb_msg, sensor_fb_msg;
  std_msgs::Bool dc_position_control_msg;

  enum motorSelection {
    motor1,
    motor2,
    motor3
  };

  short int curr_motor = motorSelection::motor1;

  ros::Publisher motor_fb_pub;
  ros::Publisher sensor_fb_pub;
  ros::Publisher dc_position_control_pub;

  ros::Subscriber<std_msgs::Float32, SensorMotorLab> motor_cmd_sub;
  ros::Subscriber<std_msgs::Int8, SensorMotorLab> motor_selection_sub;

  flexStepperPair flexStepperObj;
  lightServoPair lightServoObj;
  forceDCPair forceDCObj;

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
        forceDCObj.setup();
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
