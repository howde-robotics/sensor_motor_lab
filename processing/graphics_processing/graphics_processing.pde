import apsync.*; // Include the library
import processing.serial.*;

PFont f;
AP_Sync streamer;

public float sensorLoc = 0.;
public float motorLoc = 0.;

void setup(){
  size(500,300);

  streamer = new AP_Sync(this,"/dev/ttyACM0", 9600);

  background(0);
  f = createFont("Arial",36,true);
  textFont(f,36);
  fill(255);
}

void draw() {
  background(0);
  textAlign(CENTER);
  text(sensorLoc,width/4,height/2);
  text(motorLoc,3*width/4,height/2);
}
