#include <Encoder.h>



struct abstractMotorSensorPair {
  virtual void setup() = 0;
  virtual void run() = 0;//single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;//returns motor state 0 to 1
  virtual float sensorFeedback() = 0;//return sensor state 0 to 1

  virtual void processGuiCommand(float cmd) = 0;
};

struct forceResitiveSensorDCPair : abstractMotorSensorPair {
  // assign pin names
  const int FSR_PIN = A0;
  const int I1_PIN = 5;
  const int I2_PIN = 6;
  const int ENC_PIN_A = 2;
  const int ENC_PIN_B = 3;
  
  // constant numbers
  const int MS_PER_REV = 674;
  const float VCC = 4.98; // supply voltage
  const float R_DIV = 10000; // resistance in voltage divider
  
  // calculation numbers
  bool returnPosition = true;
  int frsADC; // the raw analog to digital converter reading
  int timePerRev = 0; // time elapsed between revolutions
  int curTime; // current time since arduino activation
  long rotorPosition; // current encoder reading
  float revsPerMin; // revolutions per minute
  float frsV; // force resistive sensor voltage
  float frsR; // force resistive sensor resistance
  float appliedForce; // force applied to the sensor
  float sensorOutput; // output of sensor in grams
  float motorOutput; // motor output either position or RPM
  
  // PID parameters
  float referenceSignal; 
  float desiredPosition; 
  float desiredRPM; 
  float motorInputSignal; 
  float propGain=0; 
  float derivGain=0;
  float integralGain=0; 
  float errLastTimeStep;
  float errorAccumulation;
  Encoder * encoder; // main encoder object
  
  void setup(){
    // configure pins correctly
    pinMode(FSR_PIN, INPUT);
    pinMode(I1_PIN, OUTPUT);
    pinMode(I2_PIN, OUTPUT);
    
    // Encoder
    Encoder enc(ENC_PIN_A, ENC_PIN_B);
    encoder = &enc;
  }

  // main running function
  void run(){
    // output is different depending on what the value of returnPosition is
    motorOutput = motorFeedback();
    sensorOutput = sensorFeedback();

    // select position or velocity as reference
    if (returnPosition){
      referenceSignal = desiredPosition; // degrees
    }
    else{
      referenceSignal = desiredRPM; // RPM
    }

    float err = referenceSignal - motorOutput;
    float errdot = (err - errLastTimeStep);
    errorAccumulation += err;

    motorInputSignal = propGain*err + derivGain*errdot + integralGain*errorAccumulation;

    sendInputToMotor(motorInputSignal);
    errLastTimeStep = err;
  }

  void setMotionGains(float Kp, float  Kd, float Ki, float positionDes, float speedDes){
    propGain = Kp;
    derivGain = Kd;
    integralGain = Ki;
    desiredPosition = positionDes;
    desiredRPM = speedDes;
  }
  
  // feedback for motor, can return speed or position
  float motorFeedback(){
    if (returnPosition){
      return calculatePosVel(encoder);
    }
    else {
      return revsPerMin;
    }
    
  }

  // feedback from sensor
  float sensorFeedback(){
    return calculateForce();
  }

  void processGuiCommand(float cmd){
    // nothing here yet
  }

  // calculate force from the sensor
  float calculateForce(){
    // read the analog input
    frsADC = analogRead(FSR_PIN);

    if (frsADC != 0){
      float frsV = frsADC * VCC / 1023.0; // calculate voltage from raw analog input
      float frsR = R_DIV * (VCC / frsV - 1.0); // calculate resistance

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
    return appliedForce;
  }

  // calculate motor position and velocity
  long calculatePosVel(Encoder * encoder){
    rotorPosition = encoder->read();
    if (rotorPosition >= MS_PER_REV){
      encoder->write(0);
      timePerRev = millis() - curTime;
      curTime = millis();
      revsPerMin = 1 / (timePerRev / (1000.0 * 60));
    }

    return rotorPosition;
  }

  void sendInputToMotor(float input){
    // input to PWM range 0-255
    int inputSignal = (int) input/255;

    // set spin direction
    if (inputSignal >= 0){
      analogWrite(I1_PIN, inputSignal);
      digitalWrite(I2_PIN, LOW);
    }
    else{
      digitalWrite(I1_PIN, LOW);
      analogWrite(I2_PIN, inputSignal);
    }
  }
  
};

// now the test program
forceResitiveSensorDCPair frs;

void setup() {
  Serial.begin(9600);
  frs.setup();
  frs.setMotionGains(1, 1, 0, 90, 40);

}

void loop() {
  frs.run();

}
