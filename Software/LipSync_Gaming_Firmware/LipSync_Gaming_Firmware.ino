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
//VERSION: 1.13 (16 September 2019) 


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
#define FIXED_DELAY 10

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

bool settingsEnabled = false; 

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
  
  joystickSpeedValue();                              //Reads saved joystick speed parameter from EEPROM and sets the speed counter
  delay(10);
  
  int execTime = millis();
  Serial.print("Configuration time: ");
  Serial.println(execTime);

  ledBlink(4, 250, 3);                                //End the initialization visual feedback

  displayVersion();                                   //Display the list of features 
  
  calculateJoystickDelay();                           //Calculate joystick action delay

  Serial.print("Speed level: ");
  Serial.println(speedCounter+1);
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
  
  xx = map(xx, -128, 128, 0, 1023);                   //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yy = map(yy, -128, 128, 0, 1023);

  Joystick.setXAxis(xx);                              //Set Joystick's x and y values
  Joystick.setYAxis(yy);
  delay(joystickDelay);
  
  //The joystick speed control push button functions

  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
      //Additional function 
    } else {
      increaseJoystickSpeed();                        //The increase joystick speed with push button up
    }
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
      //Additional function 
    } else {
      decreaseJoystickSpeed();                        //The decrease joystick speed with push button down
    }
  }

  //Pressure sensor sip and puff functions

  joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPress < puffThreshold) {
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
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPress > sipThreshold) {
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
  }
  
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

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
}

//***DISPLAY VERSION FUNCTION***//

void displayVersion(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is LipSync Gaming V1.13 (11 June 2019)");
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

void joystickSpeedValue(void) {
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
  joystickDelay = pow(1.6,(9-speedCounter))*FIXED_DELAY;
  delay(5);
}

//***INCREASE JOYSTICK SPEED FUNCTION***//

void increaseJoystickSpeed(void) {
  speedCounter++;

  if (speedCounter == 9) {
    ledBlink(6, 50, 3);
    speedCounter = 8;
  } else {
    ledBlink(speedCounter+1, 100, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println(speedCounter+1);
}

//***DECREASE JOYSTICK SPEED FUNCTION**//

void decreaseJoystickSpeed(void) {
  speedCounter--;

  if (speedCounter == -1) {
    ledBlink(6, 50, 3);     // twelve very fast blinks
    speedCounter = 0;
  } else if (speedCounter == 0) {
    ledBlink(1, 350, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  } else {
    ledBlink(speedCounter+1, 100, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*FIXED_DELAY;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println(speedCounter+1);
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  
  float nominalJoystickValue = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  sipThreshold = nominalJoystickValue + PRESSURE_THRESHOLD;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = nominalJoystickValue - PRESSURE_THRESHOLD;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  
}
