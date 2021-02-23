import controlP5.*; //import ControlP5 library
import processing.serial.*;
import apsync.*; // Include the library

//Serial port;
AP_Sync streamer;

ControlP5 cp5; //create ControlP5 object

int chartWidth = 400;
int chartHeight = 200;

int topChartX = 300;
int topChartY = 40;

int bottomChartX = topChartX;
int bottomChartY = topChartY + chartHeight + 40;

Chart sensorChart1;//flex 0-90
Chart motorChart1;//stepper 0-90
float sensor1Max = 180;
float motor1Max = 180;

Chart sensorChart2;//flex 0-90
Chart motorChart2;//stepper 0-90
float sensor2Max = 1000;
float motor2Max = 180;

Chart sensorChart3;//flex 0-90
Chart motorChart3;//stepper 0-90
float sensor3Max = 2000;
float motor3Max = 10;

float currSensorMax = sensor1Max;
float currMotorMax = motor1Max;

public float arduinoMotorLoc = 0;
public float arduinoSensorLoc = 0;

void setup(){ //same as arduino program

  size(1500, 1000);    //window size, (width, height)
  streamer = new AP_Sync(this,"/dev/ttyACM0", 57600);
  printArray(Serial.list());   //prints all available serial ports
  
  cp5 = new ControlP5(this);
  
  setupCharts();
  
  cp5.addButton("Flex_Stepper")     //"red" is the name of button
    .setPosition(100, 50)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .activateBy(ControlP5.RELEASE)
  ;   

  cp5.addButton("Light_Servo")     //"yellow" is the name of button
    .setPosition(100, 150)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .activateBy(ControlP5.RELEASE)
  ;

  cp5.addButton("Force_DC")     //"blue" is the name of button
    .setPosition(100, 250)  //x and y coordinates of upper left corner of button
    .setSize(120, 70)      //(width, height)
    .activateBy(ControlP5.RELEASE)
  ;
  
}

void draw(){  //same as loop in arduino

  background(150, 0 , 150); // background color of window (r, g, b) or (0 to 255)
  
  text(0, topChartX - 30, topChartY + chartHeight);
  text(int(currSensorMax/2), topChartX - 30, topChartY + chartHeight/2 + 5);
  text(int(currSensorMax), topChartX - 30, topChartY + 10);
 
  text(0, bottomChartX - 30, bottomChartY + chartHeight);
  text(int(currMotorMax/2), bottomChartX - 30, bottomChartY + chartHeight/2 + 5);
  text(int(currMotorMax), bottomChartX - 30, bottomChartY + 10);
 
  sensorChart1.addData("sensor", arduinoSensorLoc);
  sensorChart1.removeData("sensor", 0);
  sensorChart2.addData("sensor", arduinoSensorLoc);
  sensorChart2.removeData("sensor", 0);
  sensorChart3.addData("sensor", arduinoSensorLoc);
  sensorChart3.removeData("sensor", 0);
  
  motorChart1.addData("motor", arduinoMotorLoc);
  motorChart1.removeData("motor",0);
  motorChart2.addData("motor", arduinoMotorLoc);
  motorChart2.removeData("motor",0);
  motorChart3.addData("motor", arduinoMotorLoc);
  motorChart3.removeData("motor",0);
  
  //lets give title to our window
  fill(0, 255, 0);               //text color (r, g, b)
  text("Sensor Motor GUI", 80, 30);  // ("text", x coordinate, y coordinat)
  //text(arduinoMotorLoc, 90, 30);
  //text(arduinoSensorLoc, 90, 40);
}

public void setupCharts() {

  sensorChart1 = cp5.addChart("Flex Sensor Reading")
               .setPosition(topChartX, topChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, sensor1Max)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               ;
  sensorChart1.getColor().setBackground(color(255, 100));
  sensorChart1.addDataSet("sensor");
  sensorChart1.setColors("sensor", color(255));
  sensorChart1.setData("sensor", new float[50]);
  sensorChart1.setStrokeWeight(1.5);
  motorChart1 = cp5.addChart("Stepper Reading")
               .setPosition(bottomChartX, bottomChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, motor1Max)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               ;
  motorChart1.getColor().setBackground(color(255, 100));
  motorChart1.addDataSet("motor");
  motorChart1.setColors("motor", color(255));
  motorChart1.setData("motor", new float[50]);
  motorChart1.setStrokeWeight(1.5);
  
  sensorChart2 = cp5.addChart("Light Sensor Reading")
               .setPosition(topChartX, topChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, sensor2Max)
               .setView(Chart.LINE)
               .hide()
               ;
  sensorChart2.getColor().setBackground(color(255, 100));
  sensorChart2.addDataSet("sensor");
  sensorChart2.setColors("sensor", color(255));
  sensorChart2.setData("sensor", new float[50]);
  sensorChart2.setStrokeWeight(1.5);
  motorChart2 = cp5.addChart("Servo Reading")
               .setPosition(bottomChartX, bottomChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, motor2Max)
               .setView(Chart.LINE)
               .hide()
               ;
  motorChart2.getColor().setBackground(color(255, 100));
  motorChart2.addDataSet("motor");
  motorChart2.setColors("motor", color(255));
  motorChart2.setData("motor", new float[50]);
  motorChart2.setStrokeWeight(1.5);
  
  sensorChart3 = cp5.addChart("Force Sensor Reading")
               .setPosition(topChartX, topChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, sensor3Max)
               .setView(Chart.LINE)
               .hide()
               ;
  sensorChart3.getColor().setBackground(color(255, 100));
  sensorChart3.addDataSet("sensor");
  sensorChart3.setColors("sensor", color(255));
  sensorChart3.setData("sensor", new float[50]);
  sensorChart3.setStrokeWeight(1.5);
  motorChart3 = cp5.addChart("DC Reading")
               .setPosition(bottomChartX, bottomChartY)
               .setSize(chartWidth, chartHeight)
               .setRange(0, motor3Max)
               .setView(Chart.LINE)
               .hide()
               ;
  motorChart3.getColor().setBackground(color(255, 100));
  motorChart3.addDataSet("motor");
  motorChart3.setColors("motor", color(255));
  motorChart3.setData("motor", new float[50]);
  motorChart3.setStrokeWeight(1.5);
}

public void Flex_Stepper() {
  currSensorMax = sensor1Max;
  currMotorMax = motor1Max;
  sensorChart1.show();
  motorChart1.show();
  sensorChart2.hide();
  motorChart2.hide();
  sensorChart3.hide();
  motorChart3.hide();
  
}

public void Light_Servo() {
  currSensorMax = sensor2Max;
  currMotorMax = motor2Max;
  sensorChart1.hide();
  motorChart1.hide();
  sensorChart2.show();
  motorChart2.show();
  sensorChart3.hide();
  motorChart3.hide();
}

public void Force_DC() {
  currSensorMax = sensor3Max;
  currMotorMax = motor3Max;
  sensorChart1.hide();
  motorChart1.hide();
  sensorChart2.hide();
  motorChart2.hide();
  sensorChart3.show();
  motorChart3.show();
}
