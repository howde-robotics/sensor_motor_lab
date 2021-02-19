
//Button input and debouncing variables
volatile unsigned long lastButtonPress = 0;
int msDebounce = 500;
volatile int buttonState = 0;
const int buttonInPin = 4;
int totalNumButtonStates = 2;

//Interrupt function for button press to advance to next button mode
//void buttonPushed() {
//  unsigned long timeSince = millis() - lastButtonPress;
//  if (timeSince > msDebounce) {
//    
//    buttonState = (buttonState + 1)%totalNumButtonStates;
//    lastButtonPress = millis();
//  } 
//}

void setup() {

  //Setup button debouncing
  Serial.begin(9600);
  pinMode(buttonInPin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(buttonInPin), 
//                  buttonPushed, RISING);
  
}

void loop() {

  int buttonSignal = digitalRead(buttonInPin);
  if (buttonSignal) {
    unsigned long timeSince = millis() - lastButtonPress;
    if (timeSince > msDebounce) {
      
      buttonState = (buttonState + 1)%totalNumButtonStates;
      lastButtonPress = millis();
    } 
  }
  Serial.println(buttonState);
  delay(500);

}
