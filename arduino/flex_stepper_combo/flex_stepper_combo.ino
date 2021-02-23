struct abstractMotorSensorPair {
  virtual void setup() = 0;
  virtual void run() = 0;//single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;//returns motor state 0 to 1
  virtual float sensorFeedback() = 0;//return sensor state 0 to 1

  virtual void processGuiCommand(float cmd) = 0;
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

flexStepperPair fsp;

void setup() 
{
  Serial.begin(9600);
  fsp.setup();
  
}


void loop() 
{
  fsp.run();
  float currAngle = fsp.sensorFeedback();
  Serial.println(currAngle);
}
