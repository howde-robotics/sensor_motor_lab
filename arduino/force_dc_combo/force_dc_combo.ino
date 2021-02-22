#include <Encoder.h>

struct abstractMotorSensorPair {
  virtual void setup() = 0;
  virtual void run() = 0;//single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;//returns motor state 0 to 1
  virtual float sensorFeedback() = 0;//return sensor state 0 to 1

  virtual void processGuiCommand(float cmd) = 0;
};

struct forceDCPair {
  // pins
  const int FSR_PIN = A0;
  const int I1_PIN = 5;
  const int I2_PIN = 6;
  const int ENC_A_PIN = 2;
  const int ENC_B_PIN = 3;

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

  void setup(){
    pinMode(FSR_PIN, INPUT);
    pinMode(I1_PIN, OUTPUT);
    pinMode(I2_PIN, OUTPUT);
    Encoder enc(ENC_A_PIN, ENC_B_PIN);
    structEnc = & enc;
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
        reference = desiredRPM;
        current = revsPerMin;
        desiredPosition = appliedForce;
        desiredRPM = appliedForce;
    }

    float err = reference - current;
    Serial.print("Position: ");
    Serial.println(rotorPosition);
    Serial.print("RPM: ");
    Serial.println(revsPerMin);
    Serial.print("Error: ");
    Serial.println(err);
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
    Serial.print("Input Signal: ");
    Serial.println(input);
    
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
    if (positionControl){
      desiredPosition = cmd * scaleDegreesFactor;
    }
  }

};

forceDCPair frs;

void setup() {
    Serial.begin(9600);
    frs.setup();
    frs.setMotionGains(1, 1, 0, 270, 40, false);
}

void loop() {
  frs.run();
//      digitalWrite(frs.I1_PIN, LOW);
//      digitalWrite(frs.I2_PIN, LOW);
}
