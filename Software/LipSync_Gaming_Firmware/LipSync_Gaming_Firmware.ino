/*
//                                                                                                  
//  +++         .+++:    /++++++++/:.     .:/+++++/: .+++/`     .+++/  ++++.      ++++.     `-/++++++/:
//  oooo         .ooo:    +ooo:--:+ooo/   :ooo/:::/+/  -ooo+`   .ooo+`  ooooo:     .o-o`   `/ooo+//://+:
//  oooo         .ooo:    +ooo`    :ooo-  oooo`     `   .ooo+` .ooo+`   oooooo/`   .o-o`  .oooo-`       
//  oooo         .ooo:    +ooo`    -ooo-  -ooo+:.`       .ooo+.ooo/`    ooo:/oo+.  .o-o`  +ooo.         
//  oooo         .ooo:    +ooo.`..:ooo+`   `:+oooo+:`     `+ooooo/      ooo: :ooo- .o-o`  oooo          
//  oooo         .ooo:    +ooooooooo+:`       `-:oooo-     `+ooo/       ooo/  .+oo/.o-o`  +ooo.         
//  oooo         .ooo:    +ooo-...``             `oooo      /ooo.       ooo/   `/oo-o-o`  .oooo-        
//  oooo::::::.  .ooo:    +ooo`           :o//:::+ooo:      /ooo.       ooo/     .o-o-o`   ./oooo/:::/+/
//  +ooooooooo:  .ooo:    /ooo`           -/++ooo+/:.       :ooo.       ooo:      `.o.+      `-/+oooo+/-
//
//    ++++:     +      ++    ++  ++  +++ ++    ++++
//   oo""oo     oo     oo    oo  oo  ooo oo   oo""oo 
//  oo   `"    oooo    ooo  ooo  oo  ooYboo  oo   `" 
//  oo  "oo   oo__oo   ooYbdPoo  oo  oo Yoo  oo  "oo 
//   oooooo  oo""""oo  oo oo oo  oo  oo  oo   oooooo 
//
//A mouth operated gaming joystick based on the LipSync
*/

//Developed by : MakersMakingChange
//VERSION: 1.15 (20 February 2020) 


#include <EEPROM.h>
#include "Joystick.h"
#include <math.h>


//***PIN ASSIGNMENTS***//

#define BUTTON_UP_PIN 8                           // Joystick Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define BUTTON_DOWN_PIN 7                         // Joystick Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1_PIN 4                               // LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL_PIN A3                      // Unused Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Unused PIO4_PIN Command Pin - digital output pin A4

#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***CUSTOMIZE VALUES***//

#define PRESSURE_THRESHOLD 0.5                    //Pressure sip and puff threshold 
#define FIXED_DELAY 5
#define DEAD_ZONE 8
#define DEBUG_MODE true
#define BUTTON_MODE 0                             //Set button mode ( 0 = Default , 1 = analog trigger )

//***Map Sip & Puff actions to joystick buttons***//

int actionButton1 = 0;                            //A1.Short puff is mapped to button number 1 or button X1(Left USB)/View(Right USB) in XAC                               
int actionButton2 = 1;                            //A2.Short sip is mapped to button number 2 or button X2(Left USB)/Menu(Right USB) in XAC                               
int actionButton3 = 2;                            //A3.Long puff is mapped to button number 3 or button LS(Left USB)/RS(Right USB) in XAC
int actionButton4 = 3;                            //A4.Long sip ( Used for Shift action ) is mapped to button number 4 or button LB(Left USB)/RB(Right USB) in XAC 
int actionButton5 = 4;                            //A5.Very Long puff is mapped to button number 5 or button A(Left USB)/X(Right USB) in XAC
int actionButton6 = 5;                            //A6.Very Long sip is mapped to button number 6 or button B(Left USB)/Y(Right USB) in XAC

//***VARIABLE DECLARATION***//

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 8, 0,
  true, true, false, 
  false, false, false,
  false, false, 
  false, false, false);                           //Defining the joystick REPORT_ID and profile type


int xHigh, yHigh, xLow, yLow;   

int speedCounter = 5;                             //Declare variables for speed functionality 
int joystickDelay;

float sipThreshold;                               //Declare sip and puff variables 
float puffThreshold;
float joystickPress;
unsigned int puffCount;
unsigned int sipCount;

int lastButtonState[5];                           //Last state of the button
int buttonMode;

bool settingsEnabled = false; 

int deadZone;

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           //Set baud rate for serial coms for diagnostic data return from microcontroller
  Serial1.begin(115200);                          

  pinMode(LED_1_PIN, OUTPUT);                      //Set the visual feedback #1 LED pin to output mode
  pinMode(LED_2_PIN, OUTPUT);                      //Set the visual feedback #2 LED pin to output mode
  pinMode(TRANS_CONTROL_PIN, OUTPUT);              //Set the transistor pin to output mode
  pinMode(PIO4_PIN, OUTPUT);                       //Set the unused pin to output mode
  
  pinMode(PRESSURE_PIN, INPUT);                    //Set the pressure sensor pin to input mode
  
  pinMode(X_DIR_HIGH_PIN, INPUT);                  //Set the FSR pin to input mode
  pinMode(X_DIR_LOW_PIN, INPUT);
  pinMode(Y_DIR_HIGH_PIN, INPUT);
  pinMode(Y_DIR_LOW_PIN, INPUT);

  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);             //Set the increase joystick speed pin to input mode with pullup
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);           //Set the decrease joystick speed pin to input mode with pullup

  pinMode(2, INPUT_PULLUP);                         //Set the unused pins to input mode with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  while(!Serial1);
  
  // Initialize Joystick Library
  Joystick.begin();
  delay(10);
  pressureSensorInitialization();                   //Initialize the pressure sensor
  delay(10);

  //Initialize the last state of buttons
  lastButtonState[0] = 0;
  lastButtonState[1] = 0;
  lastButtonState[2] = 0;
  lastButtonState[3] = 0;
  lastButtonState[4] = 0;
  lastButtonState[5] = 0;
  
  getJoystickSpeed();                              //Reads saved joystick speed parameter from EEPROM and sets the speed counter
  delay(10);
  joystickCenterCalibration();
  delay(10);
  getButtonMode();
  delay(10);

  if(DEBUG_MODE) {
    int execTime = millis();
    Serial.print("Configuration time: ");
    Serial.println(execTime);
  }

  ledBlink(4, 250, 3);                                //End the initialization visual feedback

  displayVersion();                                   //Display the list of features 
  
  calculateJoystickDelay();                           //Calculate joystick action delay

  if(DEBUG_MODE) {
    Serial.print("Speed level: ");
    Serial.println(speedCounter+1);
    delay(5);

    Serial.print("Dead zone value : ");
    Serial.println(deadZone);
    delay(5);

    Serial.print("Button Mode (0:Default,1:Analog Trigger}: ");
    Serial.println(buttonMode);
    delay(5);
  }

  delay(5);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {

  settingsEnabled=serialSettings(settingsEnabled);       //Check to see if setting option is enabled in Lipsync
  
  xHigh = analogRead(X_DIR_HIGH_PIN);
  xLow = analogRead(X_DIR_LOW_PIN);
  yHigh = analogRead(Y_DIR_LOW_PIN);
  yLow = analogRead(Y_DIR_HIGH_PIN);

  xHigh = map(xHigh, 0, 1023, 0, 16);                 //Map x and y values from (0 to 1023) bound to (0 to 16) as target bound
  xLow = map(xLow, 0, 1023, 0, 16);                   //The actual input values are approximately in (0 to 800) bound range
  yHigh = map(yHigh, 0, 1023, 0, 16);
  yLow = map(yLow, 0, 1023, 0, 16);

  int xDelta = xHigh - xLow;                          //Calculate the x and y delta values   
  int yDelta = yHigh - yLow;
 
  int xx = (xDelta >= 0)? sq(xDelta):-sq(xDelta);     //Square the magnitude of x and y Delta values
  int yy = (yDelta >= 0)? sq(yDelta):-sq(yDelta);
  
  xx -= (xDelta >= 0)? int(sqrt(yDelta)):-int(sqrt(-yDelta));   //Subtract the square root of y Delta value from x Delta value to make movement smoother 
  yy -= (yDelta >= 0)? int(sqrt(xDelta)):-int(sqrt(-xDelta));   //Subtract the square root of x Delta value from y Delta value to make movement smoother 

  xx = constrain(xx, -128, 128);                      //Put constrain to set x and y range between -128 and 128 as lower and upper bounds 
  yy = constrain(yy, -128, 128);

  xx = (xx <= deadZone && xx>=-deadZone)? 0:xx;     //Make sure the default values of x and y are at the center when joystick is released 
  yy = (yy <= deadZone && yy>=-deadZone)? 0:yy;

  xx = ((sqrt(sq(xx)+sq(yy))>=128) && (xx>=deadZone || xx <= -deadZone))? 128*sgn(xx):xx;   //Make sure the maximum diagonal movement in joystick results maximum diagonal movement 
  yy = ((sqrt(sq(xx)+sq(yy))>=128) && (yy>=deadZone || yy <= -deadZone))? 128*sgn(yy):yy;

  xx = map(xx, -128, 128, 0, 1023);                   //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yy = map(yy, -128, 128, 0, 1023);

  /*
  if(DEBUG_MODE){
    Serial.print("yy: ");
    Serial.println(yy);
  
    Serial.print("xx: ");
    Serial.println(xx);     
  }
  */

  delay(joystickDelay);

  Joystick.setXAxis(xx);                              //Set Joystick's x and y values
  Joystick.setYAxis(yy);
  
  pushButtonHandler();                                //The joystick speed control push button functions

  sipAndPuffHandler(buttonMode);                               //Pressure sensor sip and puff functions

}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***FIND SIGN OF VARIABLE FUNCTION***//

int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

//***PUSH BUTTON SPEED HANDLER FUNCTION***//

void pushButtonHandler(void) {
  
  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
      joystickCenterCalibration();                   //Joystick default center values 
    } else {
      increaseJoystickSpeed();                        //The increase joystick speed with push button up
    }
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
      joystickCenterCalibration();                   //Joystick default center values
    } else {
      decreaseJoystickSpeed();                        //The decrease joystick speed with push button down
    }
  }
}

//***SIP AND PUFF BUTTON HANDLER FUNCTION***//

void sipAndPuffHandler(int mode) {

  joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPress < puffThreshold) {
    switch (mode) {
      case 0:
        while (joystickPress < puffThreshold) {
          joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          puffCount++;                                    //Threshold counter
          delay(5);
        }
        if (puffCount < 150) {
          if (!lastButtonState[0]) {
              Joystick.pressButton(actionButton1);
              delay(250);
              Joystick.releaseButton(actionButton1);
              delay(50);
              lastButtonState[0] = 0;
            }
          } else if (puffCount > 150 && puffCount < 450) {
            if (!lastButtonState[2]) {
              Joystick.pressButton(actionButton3);
              delay(250);
              Joystick.releaseButton(actionButton3);
              delay(50);
              lastButtonState[2] = 0;
            } 
          } else if (puffCount > 450) {
            if (!lastButtonState[4]) {
              Joystick.pressButton(actionButton5);
              delay(250);
              Joystick.releaseButton(actionButton5);
              delay(50);
              lastButtonState[4] = 0;
            } 
          }
        puffCount = 0;
        break;
      case 1: 
        Joystick.pressButton(actionButton1);
        delay(10);
        lastButtonState[0] = 1;
        break;
    }
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPress > sipThreshold) {
    switch (mode) {
      case 0:
        while (joystickPress > sipThreshold) {
          joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          sipCount++;
          delay(5);
        }
        if (sipCount < 150) {
          if (!lastButtonState[1]) {
              Joystick.pressButton(actionButton2);
              delay(250);
              Joystick.releaseButton(actionButton2);
              delay(50);
              lastButtonState[1] = 0;
            } 
          } else if (sipCount > 150 && sipCount < 450) {
            ledBlink(1, 250, 1); 
            if (!lastButtonState[3]) {
              Joystick.pressButton(actionButton4);
              delay(50);
              lastButtonState[3] = 1;
            } 
            else {
              Joystick.releaseButton(actionButton4);
              delay(50);  
              lastButtonState[3] = 0;    
            }
          } else if (sipCount > 450) {
             if (!lastButtonState[5]) {
              Joystick.pressButton(actionButton6);
              delay(250);
              Joystick.releaseButton(actionButton6);
              delay(50);
              lastButtonState[5] = 0;
            } 
          }
        sipCount = 0;
        break;
      case 1:
        Joystick.pressButton(actionButton2);
        delay(10);
        lastButtonState[1] = 1;
        break;
    }
  }
  if (joystickPress <= sipThreshold && joystickPress >= puffThreshold && mode==1) {
    Joystick.releaseButton(actionButton1);
    Joystick.releaseButton(actionButton2);
    delay(10);
    lastButtonState[0] = 0;
    lastButtonState[1] = 0;
    
  }
}

//***GET BUTTON MODE FUNCTION***//

void getButtonMode(void) {
  delay(10);
  EEPROM.get(32, buttonMode);
  delay(10);
  if(buttonMode !=0 && buttonMode !=1){
    buttonMode=BUTTON_MODE;
    EEPROM.put(32, buttonMode);
    delay(10);
  }
}

//***SET BUTTON MODE FUNCTION***//

void setButtonMode(int mode) {
  if(mode ==0 || mode ==1){
    buttonMode=mode;
    EEPROM.put(32, buttonMode);
    delay(10);
    Serial.print("Button mode is set to: ");
    Serial.println(buttonMode);
  }
}

//***JOYSTICK DEFAULT VALUES FUNCTION***//

void joystickCenterCalibration(void) {
  
  int xDefaultHigh = analogRead(X_DIR_HIGH_PIN);
  int xDefaultLow = analogRead(X_DIR_LOW_PIN);
  int yDefaultHigh = analogRead(Y_DIR_LOW_PIN);
  int yDefaultLow = analogRead(Y_DIR_HIGH_PIN);

  xDefaultHigh = map(xDefaultHigh, 0, 1023, 0, 16);                 //Map x and y values from (0 to 1023) bound to (0 to 16) as target bound
  xDefaultLow = map(xDefaultLow, 0, 1023, 0, 16);                   //The actual input values are approximately in (0 to 800) bound range
  yDefaultHigh = map(yDefaultHigh, 0, 1023, 0, 16);
  yDefaultLow = map(yDefaultLow, 0, 1023, 0, 16);

  int xDelta = xDefaultHigh - xDefaultLow;                          //Calculate the x and y delta values   
  int yDelta = yDefaultHigh - yDefaultLow;
 
  int xDefault = (xDelta >= 0)? sq(xDelta):-sq(xDelta);     //Square the magnitude of x and y Delta values
  int yDefault = (yDelta >= 0)? sq(yDelta):-sq(yDelta);
  
  xDefault -= (xDelta >= 0)? int(sqrt(yDelta)):-int(sqrt(-yDelta));   //Subtract the square root of y Delta value from x Delta value to make movement smoother 
  yDefault -= (yDelta >= 0)? int(sqrt(xDelta)):-int(sqrt(-xDelta));   //Subtract the square root of x Delta value from y Delta value to make movement smoother 

  xDefault = constrain(xDefault, -128, 128);                      //Put constrain to set x and y range between -128 and 128 as lower and upper bounds 
  yDefault = constrain(yDefault, -128, 128);

  deadZone = max(max (abs(xDefault), abs(xDefault)), DEAD_ZONE);    //Select the largest value as dead zone

  if(DEBUG_MODE) {
    Serial.print("Dead zone value : ");
    Serial.println(deadZone);
    delay(5);
  }
  delay(5);
}

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
    
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "settings" is in input string.
       if (settingsFlag==false && inString=="settings") {
       Serial.println("Actions:");                //Display list of possible actions 
       Serial.println("S,(+ or -)");
       Serial.println("C,0");
       Serial.println("B,(0 or 1)");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString.length()==((2*2)-1)){ //Check if the input parameter is true and the received string is 3 characters only
        inString.replace(",","");                 //Remove commas 
        if(inString.length()==2) {                //Perform settings actions if there are only two characters in the string.
          writeSettings(inString);
          Serial.println("Successfully changed.");
        }   
        Serial.println("Exiting the settings.");
        settingsFlag=false;   
       }
       else if (settingsFlag==true){
        Serial.println("Exiting the settings.");
        settingsFlag=false;         
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SPEED USING SOFTWARE***//

void writeSettings(String changeString) {
    char changeChar[changeString.length()+1];
    //Set the communication mode to USB if received "M0" and to Bluetooth mode if received "M1"
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Increase the cursor speed if received "S+" and decrease the cursor speed if received "S-"
    if(changeChar[0]=='S' && changeChar[1]=='+') {
      increaseJoystickSpeed();
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='-') {
      decreaseJoystickSpeed();
      delay(5);
    } 

     //Joystick center Calibration
     if(changeChar[0]=='C' && changeChar[1]=='0') {
      joystickCenterCalibration();
      delay(5);
    }

    //Change Button mode
    if(changeChar[0]=='B' && changeChar[1]=='0') {
      setButtonMode(0);
      delay(5);
    } else if (changeChar[0]=='B' && changeChar[1]=='1') {
      setButtonMode(1);
      delay(5);
    } 
    
}

//***DISPLAY VERSION FUNCTION***//

void displayVersion(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is LipSync Gaming V1.15 (20 February 2020)");
  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println(" ");

}

//***LED ON FUNCTION***//

void ledOn(int ledNumber) {
  switch (ledNumber) {
    case 1: {
        digitalWrite(LED_1_PIN, HIGH);
        delay(5);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
    case 2: {
        digitalWrite(LED_2_PIN, HIGH);
        delay(5);
        digitalWrite(LED_1_PIN, LOW);
        break;
      }
  }
}

//***LED CLEAR FUNCTION***//

void ledClear(void) {
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
}

//***LED BLINK FUNCTION***//

void ledBlink(int numBlinks, int delayBlinks, int ledNumber ) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
  }
}

//***HID JOYSTICK SPEED FUNCTION***//

void getJoystickSpeed(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  if(var>=0){
    speedCounter = var;
  } 
  else {
    EEPROM.put(2, speedCounter);
    delay(5);
    Serial.println("Setting the speed value for the first time.");
  }
}

//***CALCULATE JOYSTICK DELAY FUNCTION***//

void calculateJoystickDelay(void) {
  joystickDelay = pow(1.6,(11-speedCounter))*FIXED_DELAY;
  delay(5);
}

//***INCREASE JOYSTICK SPEED FUNCTION***//

void increaseJoystickSpeed(void) {
  speedCounter++;

  if (speedCounter == 11) {
    ledBlink(6, 50, 3);
    speedCounter = 10;
  } else {
    ledBlink(speedCounter+1, 100, 1);
    joystickDelay = pow(1.6,(11-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  if(DEBUG_MODE){
    Serial.print("Speed level: ");
    Serial.println(speedCounter+1);    
    delay(5);
  }
}

//***DECREASE JOYSTICK SPEED FUNCTION**//

void decreaseJoystickSpeed(void) {
  speedCounter--;

  if (speedCounter == -1) {
    ledBlink(6, 50, 3);     // twelve very fast blinks
    speedCounter = 0;
  } else if (speedCounter == 0) {
    ledBlink(1, 350, 1);
    joystickDelay = pow(1.6,(11-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  } else {
    ledBlink(speedCounter+1, 100, 1);
    joystickDelay = pow(1.6,(11-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  }

  if(DEBUG_MODE){
    Serial.print("Speed level: ");
    Serial.println(speedCounter+1);
    delay(5);
  }
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  
  float nominalJoystickValue = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  sipThreshold = nominalJoystickValue + PRESSURE_THRESHOLD;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = nominalJoystickValue - PRESSURE_THRESHOLD;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  
}
