/*
//                                                                                                  
//    ,ad8888ba,                                                ad88888ba            88  88         
//   d8"'    `"8b                                              d8"     "8b    ,d     ""  88         
//  d8'                                                        Y8,            88         88         
//  88             ,adPPYYba,  88,dPYba,,adPYba,    ,adPPYba,  `Y8aaaaa,    MM88MMM  88  88   ,d8   
//  88      88888  ""     `Y8  88P'   "88"    "8a  a8P_____88    `"""""8b,    88     88  88 ,a8"    
//  Y8,        88  ,adPPPPP88  88      88      88  8PP"""""""          `8b    88     88  8888[      
//   Y8a.    .a88  88,    ,88  88      88      88  "8b,   ,aa  Y8a     a8P    88,    88  88`"Yba,   
//    `"Y88888P"   `"8bbdP"Y8  88      88      88   `"Ybbd8"'   "Y88888P"     "Y888  88  88   `Y8a  
//
//A mouth operated gaming joystick based on the LipSync
*/

//Developed by : MakersMakingChange
//VERSION: 1.11 (20 October 2018) 


#include <EEPROM.h>
#include "Joystick.h"
#include <math.h>


//***PIN ASSIGNMENTS***//

#define PUSH_BUTTON_UP 8                          // Joystick Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define PUSH_BUTTON_DOWN 7                        // Joystick Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1 4                                   // LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL A3                          // Unused Transistor Control Pin - digital output pin A3
#define PIO4 A4                                   // Unused PIO4 Command Pin - digital output pin A4

#define PRESSURE_JOYSTICK A5                      // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH A0                             // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW A1                              // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH A2                             // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW A10                             // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***VARIABLE DECLARATION***//

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 8, 0,
  true, true, false, 
  false, false, false,
  false, false, 
  false, false, false);                         //Defining the joystick REPORT_ID and profile type

int button1 = 0;                                //Declare button number variables 
int button2 = 1;
int button3 = 2;
int button4 = 3;
int button5 = 4;
int button6 = 5;
int button7 = 6;
int button8 = 7;

int xHigh, yHigh, xLow, yLow;   

int speedCounter = 2;                         //Declare variables for speed functionality 
int fixedDelay = 10;
int joystickDelay;

float sipThreshold;                           //Declare sip and puff variables 
float puffThreshold;
float joystickPress;
unsigned int puffCount;
unsigned int sipCount;

int lastButtonState[8];                       //Last state of the button

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           //Set baud rate for serial coms for diagnostic data return from microcontroller
  Serial1.begin(115200);                          

  pinMode(LED_1, OUTPUT);                         //Set the visual feedback #1 LED pin to output mode
  pinMode(LED_2, OUTPUT);                         //Set the visual feedback #2 LED pin to output mode
  pinMode(TRANS_CONTROL, OUTPUT);                 //Set the transistor pin to output mode
  pinMode(PIO4, OUTPUT);                          //Set the unused pin to output mode
  
  pinMode(PRESSURE_JOYSTICK, INPUT);              //Set the pressure sensor pin to input mode
  
  pinMode(X_DIR_HIGH, INPUT);                     //Set the FSR pin to input mode
  pinMode(X_DIR_LOW, INPUT);
  pinMode(Y_DIR_HIGH, INPUT);
  pinMode(Y_DIR_LOW, INPUT);

  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          //Set the increase joystick speed pin to input mode with pullup
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        //Set the decrease joystick speed pin to input mode with pullup

  pinMode(2, INPUT_PULLUP);                       //Set the unused pins to input mode with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  while(!Serial1);
  
  // Initialize Joystick Library
  Joystick.begin();
  delay(10);
  pressureSensorInitialization();               //Initialize the pressure sensor
  delay(10);

  //Initialize the last state of buttons
  lastButtonState[0] = 0;
  lastButtonState[1] = 0;
  lastButtonState[2] = 0;
  lastButtonState[3] = 0;
  lastButtonState[4] = 0;
  lastButtonState[5] = 0;
  lastButtonState[6] = 0;
  lastButtonState[7] = 0;

  joystickSpeedValue();                         //Reads saved joystick speed parameter from EEPROM and sets the speed counter
  delay(10);
  
  int execTime = millis();
  Serial.print("Configuration time: ");
  Serial.println(execTime);

  blink(4, 250, 3);                             //End the initialization visual feedback

  displayFeatureList();                         //Display the list of features 
  
  joystickDelay = pow(1.6,(9-speedCounter))*fixedDelay;

  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
  delay(5);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  xHigh = analogRead(X_DIR_HIGH);
  xLow = analogRead(X_DIR_LOW);
  yHigh = analogRead(Y_DIR_LOW);
  yLow = analogRead(Y_DIR_HIGH);

  xHigh = map(xHigh, 0, 1023, 0, 16);                 //Map x and y values from (0 to 1023) bound to (0 to 16) as target bound
  xLow = map(xLow, 0, 1023, 0, 16);                   //The actual input values are approximately in (0 to 800) bound range
  yHigh = map(yHigh, 0, 1023, 0, 16);
  yLow = map(yLow, 0, 1023, 0, 16);

  int xDelta = xHigh - xLow;                        //Calculate the x and y delta values   
  int yDelta = yHigh - yLow;
 
  int xx = (xDelta >= 0)? sq(xDelta):-sq(xDelta);     //Square the magnitude of x and y Delta values
  int yy = (yDelta >= 0)? sq(yDelta):-sq(yDelta);
  
  xx -= (xDelta >= 0)? int(sqrt(yDelta)):-int(sqrt(-yDelta));   //Subtract the square root of y Delta value from x Delta value to make movement smoother 
  yy -= (yDelta >= 0)? int(sqrt(xDelta)):-int(sqrt(-xDelta));   //Subtract the square root of x Delta value from y Delta value to make movement smoother 

  xx = constrain(xx, -128, 128);          //Put constrain to set x and y range between -128 and 128 as lower and upper bounds 
  yy = constrain(yy, -128, 128);
  
  xx = map(xx, -128, 128, 0, 1023);       //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yy = map(yy, -128, 128, 0, 1023);

  Joystick.setXAxis(xx);                  //Set Joystick's x and y values
  Joystick.setYAxis(yy);
  delay(joystickDelay);
  
  //The joystick speed control push button functions

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      //Additional function 
    } else {
      increaseJoystickSpeed();      //The increase joystick speed with push button up
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      //Additional function 
    } else {
      decreaseJoystickSpeed();      //The decrease joystick speed with push button down
    }
  }

  //Pressure sensor sip and puff functions

  joystickPress = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;   
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPress < puffThreshold) {
    while (joystickPress < puffThreshold) {
      joystickPress = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      puffCount++;                  //Threshold counter
      delay(5);
    }
      if (puffCount < 150) {
        //Press joystick button number 5 or button A/X in XAC
        if (!lastButtonState[4]) {
          Joystick.pressButton(button5);
          delay(250);
          Joystick.releaseButton(button5);
          delay(50);
          lastButtonState[4] = 0;
        }
      } else if (puffCount > 150 && puffCount < 450) {
        //Press joystick button number 4 or button LB/RB in XAC
        if (!lastButtonState[3]) {
          Joystick.pressButton(button4);
          delay(250);
          Joystick.releaseButton(button4);
          delay(50);
          lastButtonState[3] = 0;
        } 
      } else if (puffCount > 450) {
        //Press joystick button number 7 or button View/X1 in XAC
        if (!lastButtonState[6]) {
          Joystick.pressButton(button7);
          delay(250);
          Joystick.releaseButton(button7);
          delay(50);
          lastButtonState[6] = 0;
        } 
      }
    puffCount = 0;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPress > sipThreshold) {
    while (joystickPress > sipThreshold) {
      joystickPress = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      sipCount++;                //Threshold counter
      delay(5);
    }
      if (sipCount < 150) {
        //Press joystick button number 6 or button B/Y in XAC
        if (!lastButtonState[5]) {
          Joystick.pressButton(button6);
          delay(250);
          Joystick.releaseButton(button6);
          delay(50);
          lastButtonState[5] = 0;
        } 
      } else if (sipCount > 150 && sipCount < 450) {
        //Press joystick button number 1 or button X1/View in XAC ( Used for Shift action )
        blink(1, 250, 1); 
        if (!lastButtonState[0]) {
          Joystick.pressButton(button1);
          delay(50);
          lastButtonState[0] = 1;
        } 
        else {
          Joystick.releaseButton(button1);
          delay(50);  
          lastButtonState[0] = 0;    
        }
      } else if (sipCount > 450) {
        //Press joystick button number 8 or button Menu/X2 in XAC
         if (!lastButtonState[7]) {
          Joystick.pressButton(button8);
          delay(250);
          Joystick.releaseButton(button8);
          delay(50);
          lastButtonState[7] = 0;
        } 
      }
    sipCount = 0;
  }
  
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***DISPLAY FEATURE LIST FUNCTION***//

void displayFeatureList(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the GameStik firmware");
  Serial.println(" ");
  Serial.println("VERSION: 1.11 (20 October 2018)");
  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println(" ");

}


//***LED BLINK FUNCTION***//

void blink(int numBlinks, int delayBlinks, int ledNumber ) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2, LOW);
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
  speedCounter = var;
}

//***INCREASE JOYSTICK SPEED FUNCTION***//

void increaseJoystickSpeed(void) {
  speedCounter++;

  if (speedCounter == 9) {
    blink(6, 50, 3);
    speedCounter = 8;
  } else {
    blink(speedCounter, 100, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
}

//***DECREASE JOYSTICK SPEED FUNCTION**//

void decreaseJoystickSpeed(void) {
  speedCounter--;

  if (speedCounter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speedCounter = 0;
  } else if (speedCounter == 0) {
    blink(1, 350, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  } else {
    blink(speedCounter, 100, 1);
    joystickDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  
  float nominal_joystick_value = (((float)analogRead(PRESSURE_JOYSTICK)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  sipThreshold = nominal_joystick_value + 0.5;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = nominal_joystick_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  
}


