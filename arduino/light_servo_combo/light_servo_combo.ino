#include <Servo.h> 
#include <math.h>
#define SERVOPIN 11       //Servo (PWM) writing
#define LIGHTSENSORPIN A1 //Ambient light sensor reading 

struct abstractMotorSensorPair {
  virtual void setup() = 0;
  virtual void run() = 0;//single loop of read the sensor and set the motor
  
  virtual float motorFeedback() = 0;//returns motor state 0 to 1
  virtual float sensorFeedback() = 0;//return sensor state 0 to 1

  virtual void processGuiCommand(int cmd) = 0;
};

struct lightServoPair : abstractMotorSensorPair {
  Servo myservo; 
  int inc = 1;//If inc == 1, servo is going from 0 to 180 degrees; reverse if inc == 0
  unsigned long timer;
  const float max_vel = 315.78948;//max velocity of servo is 315.79 degrees per second
  float pos_float = 0.0;//position of servo from 0 to 180
  float filter_read = 0.0;//filtered ambient light value
  const float filter_percent = 0.1;//percent of prveious value used in filtered ambient light value
  
  void setup() {
    pinMode(LIGHTSENSORPIN,  INPUT);  
    myservo.attach(SERVOPIN);
    myservo.write(0);//Set servo position to 0 degrees
    timer = millis();
  }

  void processGuiCommand(int command){
    //no commands atm
  }

  float motorFeedback() {
    //Returns angle of the servo, from 0 to 180 degrees
    return myservo.read();
  }

  float sensorFeedback() {
    //Returns sensor illuminance in lux(lx)
    return analogRead(LIGHTSENSORPIN);
  }

  void run(){
    float reading = analogRead(LIGHTSENSORPIN); //Read light level
    Serial.println("sensor reading");
    Serial.println(reading);
    filter_read = filter_percent*filter_read+(1-filter_percent)*reading;
    float ratio = filter_read / 1023.0; //Get percent of maximum value (1023)
    float vel = ratio*max_vel; //Get servo velocity based on ratio
  
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
  
};

lightServoPair lsp;

void setup() 
{
  Serial.begin(9600);
  lsp.setup();
}

void loop() 
{
  lsp.run();
}
