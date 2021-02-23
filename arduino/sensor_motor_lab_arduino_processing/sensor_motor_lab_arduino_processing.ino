#include <Servo.h> 
#include <math.h>
#include <Encoder.h>
#include <AP_Sync.h>
#define SERVOPIN 11       //Servo (PWM) writing
#define LIGHTSENSORPIN A1 //Ambient light sensor reading
#define FLEXSENSORPIN A0 //Pin in for flex meter signal
#define BUTTONPIN 4  // pin in for button

AP_Sync streamer(Serial);

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
  const float rBend = 52000;//resistance of the flex meter when bent at 90degrees
  
  const float motorStep = 0.9;//degrees
  
  const int flexPin = A0;//Pin in for flex meter signal
  const int  stepPin = 8;//pin out for stepper step command
  const int dirPin = 7;//pin out for stepper direction command
  
  float motorState = 0;//starting motor state value (degrees)
  float sensorState = 0;//starting flex state value (will be immediately overwritten

  const float emaAlpha = 2.0/(1 + 40);//exponential moving average weighting term
  float avFlexAngle = 0;
  
  void setup() {
    //set pin IO
    pinMode(flexPin, INPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void processGuiCommand(float command){
    //no commands atm
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
    if (abs(motorState - sensorState) > 2 * motorStep) {
      if (sensorState < motorState) {
        digitalWrite(dirPin, HIGH);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(4000);
        digitalWrite(stepPin, LOW);
        motorState -= motorStep;
      } else {
        digitalWrite(dirPin, LOW);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(4000);
        digitalWrite(stepPin, LOW);
        motorState += motorStep;
      }
    }
    
    delayMicroseconds(4000);    
  }

  
};

const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;
Encoder enc(ENC_A_PIN, ENC_B_PIN);

struct forceDCPair {
  // pins
//  const int FSR_PIN = A0;
//  const int I1_PIN = 5;
//  const int I2_PIN = 6;
//  const int ENC_A_PIN = 2;
//  const int ENC_B_PIN = 13;
  const int FSR_PIN = A3;
  const int I1_PIN = 5;
  const int I2_PIN = 6;
//  const int ENC_A_PIN = 2;
//  const int ENC_B_PIN = 3;
  
  // constant numbers
  const int MS_PER_REV = 674;
  const float VCC = 4.98; // supply voltage
  const float R_DIV = 10000; // resistance in voltage divider
  
  // motor stuff
  long rotorPosition;
  float revsPerMin;
  bool positionControl = false;

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
  float scaleInputFactor = MS_PER_REV / 255 * 2;
  float scaleDegreesFactor = MS_PER_REV / 360;

  //moving average of velocity
  const float emaAlpha = 2.0/(1 + 20);//exponential moving average weighting term
  float prevPos = 0;
  float prevTime = 0;

  void setup(){
    pinMode(FSR_PIN, INPUT);
    pinMode(I1_PIN, OUTPUT);
    pinMode(I2_PIN, OUTPUT);
  }

  void run(){
    float reference;
    float current;
  
    calculatePosVel();
    calculateForce();

    
    if (positionControl){
        reference = desiredPosition;
        current = rotorPosition;
    }
    else{
        desiredRPM = appliedForce;
        reference = desiredRPM;
        current = revsPerMin;
        desiredPosition = appliedForce;
    }

    
    float err = reference - current;
    float errDot = err - prevErr;
    errAccumulation += err;
    prevErr = err;
    
    controlSignal = pGain*err + dGain*errDot + iGain*errAccumulation;
    sendMotorInput(controlSignal);
  }

  bool wrappedAround = false;
    
  void calculatePosVel(){
    rotorPosition = abs(enc.read());

    unsigned long now = millis(); 
    unsigned long dt = now - prevTime;
    prevTime = now;
    float dW = 0;
    if (wrappedAround) {
      dW = (rotorPosition + MS_PER_REV - prevPos)/float(MS_PER_REV);//fraction of a revolution 0-1
      wrappedAround = false;
    } else {
      dW = (rotorPosition - prevPos)/float(MS_PER_REV);
    }
    float instW = 0;
    if (dt > 0) {
      instW = dW / dt * 1000 * 60;
    }
    revsPerMin = (emaAlpha * instW) + (1.0 - emaAlpha) * revsPerMin ;
    prevPos = rotorPosition;
    
    // calculate RPM
    if (rotorPosition >= MS_PER_REV){
      enc.write(0);
      wrappedAround = true;
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
    return rotorPosition * 360./MS_PER_REV;
  }
  float sensorFeedback(){
    return appliedForce;
  }

  void processGuiCommand(float cmd){
    if (positionControl){
      desiredPosition = cmd * scaleDegreesFactor;
    }
  }

};

//button debounce
unsigned long lastButtonPress = 0;
int msDebounce = 500;
volatile int buttonState = 0;
int totalNumButtonStates = 2;

enum motorSelection {
    motor1,
    motor2,
    motor3
  };

short int curr_motor = motorSelection::motor1;

flexStepperPair flexStepperObj;
lightServoPair lightServoObj;
forceDCPair forceDCObj;

void setup() {
  Serial.begin(57600);
  pinMode(BUTTONPIN, INPUT);
  switch(curr_motor) {
    case motorSelection::motor1:
      flexStepperObj.setup();
      break;
    case motorSelection::motor2:
      lightServoObj.setup();
      break;
    case motorSelection::motor3:
      forceDCObj.setup();
      forceDCObj.setMotionGains(1, 1, 0, 300*forceDCObj.scaleDegreesFactor, 40, forceDCObj.positionControl);
      break;
  } 
}


void loop() {
  
  if (true) {
      if(Serial.available()){
        String command = Serial.readString();
        if(command == "motor1"){
          curr_motor = motorSelection::motor1;
          flexStepperObj.setup();
        }else if (command == "motor2") {
          curr_motor = motorSelection::motor2;
          lightServoObj.setup();
        }else if (command == "motor3"){
          curr_motor = motorSelection::motor3;
          forceDCObj.setup();
          forceDCObj.setMotionGains(1, 1, 0, 300*forceDCObj.scaleDegreesFactor, 40, forceDCObj.positionControl);
        }
      }
    }

    //run motor once and get data
    float data1;
    float data2;
    switch (curr_motor) {
      case motorSelection::motor1:
        flexStepperObj.run();
        data1 = flexStepperObj.motorFeedback();
        data2 = flexStepperObj.sensorFeedback();
        break;
      case motorSelection::motor2:
        lightServoObj.run();
        data1 = lightServoObj.motorFeedback();
        data2 = lightServoObj.sensorFeedback();
        break;
      case motorSelection::motor3:
        int buttonSignal = digitalRead(BUTTONPIN);
        if (buttonSignal) {
          unsigned long timeSince = millis() - lastButtonPress;
          if (timeSince > msDebounce) {
            //change buttonState
            buttonState = (buttonState + 1)%totalNumButtonStates;

            //set control mode
            forceDCObj.setMotionGains(1, 1, 0, 300*forceDCObj.scaleDegreesFactor, 40, buttonState);
            lastButtonPress = millis();
          } 
        }
        
        forceDCObj.run();
        if (forceDCObj.positionControl) {
          data1 = forceDCObj.motorFeedback();
          
        } else {
          data1 = forceDCObj.revsPerMin;
        }
        streamer.sync("dcPosControl", forceDCObj.positionControl);
        data2 = forceDCObj.sensorFeedback();
        
        break;
    }

    
    streamer.sync("arduinoMotorLoc", data1);
    streamer.sync("arduinoSensorLoc", data2);

}
