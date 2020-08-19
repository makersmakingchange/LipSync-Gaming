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
//VERSION: 1.17 (18 Aug 2020) 



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

//***SERIAL SETTINGS VARIABLE***//

#define SERIAL_SETTINGS true

//***CUSTOMIZABLE VARIABLES***//

#define DEBUG_MODE false
#define RAW_MODE false
#define BUTTON_MODE 1                             //Set button mode ( 1 = Digital buttons , 2 = Analog buttons )
#define SENSITIVITY_COUNTER 5
#define PRESSURE_THRESHOLD 10                   //Pressure sip and puff threshold 


#define ACTION_BUTTON_1 0                       //A1.Short Puff is mapped to button number 1 or button X1(Left USB)/View(Right USB) in XAC   
#define ACTION_BUTTON_2 1                       //A2.Short Sip is mapped to button number 2 or button X2(Left USB)/Menu(Right USB) in XAC    
#define ACTION_BUTTON_3 2                       //A3.Long Puff is mapped to button number 3 or button LS(Left USB)/RS(Right USB) in XAC
#define ACTION_BUTTON_4 3                       //A4.Long Sip ( Used for Shift action ) is mapped to button number 4 or button LB(Left USB)/RB(Right USB) in XAC 
#define ACTION_BUTTON_5 4                       //A5.Very Long Puff is mapped to button number 5 or button A(Left USB)/X(Right USB) in XAC
#define ACTION_BUTTON_6 5                       //A6.Very Long Sip is mapped to button number 6 or button B(Left USB)/Y(Right USB) in XAC
                         

//***DON'T CHANGE THESE VARIABLES***//

#define JS_DELAY 10                              //The fixed delay for each loop action 
#define JS_MOVE_RADIUS  30                       //The deadzone for input FSR analog value
#define LONG_PRESS_TIME 2
#define JS_MAPPED_IN_DEADZONE 0.50
#define JS_MAPPED_IN_NEUTRAL 12
#define JS_MAPPED_IN_MAX 16.00
#define JS_OUT_DEAD_ZONE 1
#define JS_OUT_MAX 127
#define CHANGE_DEFAULT_TOLERANCE 0.44             //The tolerance in % for changes between current reading and previous reading ( %100 is max FSRs reading )


//***VARIABLE DECLARATION***//

int buttonMode;                                   //The button mode variable 

long switchTimer[3];
bool switchPreviousState[2];

//***Map Sip & Puff actions to joystick buttons for mode 1***//
int actionButton[6] = {ACTION_BUTTON_1, ACTION_BUTTON_2, ACTION_BUTTON_3, ACTION_BUTTON_4, ACTION_BUTTON_5, ACTION_BUTTON_6};
int lastButtonState[5];                           //Last state of the buttons

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 8, 0,
  true, true, false, 
  false, false, false,
  false, false, 
  false, false, false);                           //Defining the joystick REPORT_ID and profile type


typedef struct {                                  //Structure for a degree five polynomial 
  float _equationACoef;
  float _equationBCoef;
  float _equationCCoef;
  float _equationDCoef;
  float _equationECoef;
  float _equationFCoef;
} _equationCoef;

//Initialize the equation coefficient structure for each FSR reading 
_equationCoef xHighEquation = {};
_equationCoef xLowEquation = {};
_equationCoef yHighEquation = {};
_equationCoef yLowEquation = {};

int xHigh, xLow, yHigh, yLow;                                                   //FSR raw values 

int xHighPrev, xLowPrev, yHighPrev, yLowPrev;

int xHighNeutral, xLowNeutral, yHighNeutral, yLowNeutral;                       //Neutral FSR values at the resting position 

int xHighMax, xLowMax, yHighMax, yLowMax;                                       //Max FSR values which are set to the values from EEPROM

float xHighYHigh, xHighYLow, xLowYLow, xLowYHigh;

float xHighMapped, xLowMapped, yHighMapped, yLowMapped;

int xHighChangeTolerance, yHighChangeTolerance, xLowChangeTolerance, yLowChangeTolerance;       //The tolerance of changes in FSRs readings 


int xOut, yOut;

float xDelta, yDelta;

//The input to output (x to y) curve equation using degree five polynomial equation for each sensitivity level
_equationCoef levelEquation1 = {0.0004,-0.0041,0.0000,-0.0185,1.8000,0.0000};
_equationCoef levelEquation2 = {0.0002,-0.0021,0.0201,-0.3704,4.3000,0.0000};
_equationCoef levelEquation3 = {-0.0008,0.0314,-0.3565,1.2731,3.6056,0.0000};
_equationCoef levelEquation4 = {0.0001,-0.0005,0.0309,-0.4954,7.2167,0.0000};
_equationCoef levelEquation5 = {-0.0004,0.0175,-0.2145,1.0093,5.1333,0.0000};
_equationCoef levelEquation6 = {0.0000,0.0000,0.0000,0.0000,8.4667,0.0000};
_equationCoef levelEquation7 = {-0.0001,0.0062,-0.125,0.7778,9.3000,0.0000};
_equationCoef levelEquation8 = {-0.0004,0.0195,-0.3133,1.6574,10.6889,0.0000};
_equationCoef levelEquation9 = {0.0001,-0.0010,0.0093,-0.9907,21.2444,0.0000};
_equationCoef levelEquation10 = {0.0008,-0.0303,0.5062,-5.1157,35.5500,0.0000};
_equationCoef levelEquation11 = {-0.0001,-0.0051,0.3441,-6.1204,45.4111,0.0000};


//All sensitivity levels
_equationCoef levelEquations[11] = {levelEquation1, levelEquation2, levelEquation3, levelEquation4, levelEquation5, levelEquation6, levelEquation7, levelEquation8, levelEquation9, levelEquation10, levelEquation11};

bool debugModeEnabled;                                  //Declare raw and debug enable variable
bool rawModeEnabled;

int sensitivityCounter;                                 //Declare variables for sensitivity adjustment  

float sipThreshold;                                     //Declare sip and puff variables 
float puffThreshold;
float joystickPressure;

int joystickDeadzone;                                   //Declare joystick deadzone variable 

unsigned int puffCount;                                 //Declare sip and puff counter variables 
unsigned int sipCount;

int modelNumber;                                        //Declare LipSync model number variable 

bool settingsEnabled = false;                           //Serial input settings command mode enabled or disabled 


//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                            //Set baud rate for serial coms for diagnostic data return from microcontroller      
                     
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

  Joystick.setXAxisRange(-JS_OUT_MAX, JS_OUT_MAX);    //Set the range of joystick X axis
  Joystick.setYAxisRange(-JS_OUT_MAX, JS_OUT_MAX);    //Set the range of joystick y axis


  switchPreviousState[0] = HIGH;
  switchPreviousState[1] = HIGH;

  //Initialize the last state of buttons
  lastButtonState[0] = 0;
  lastButtonState[1] = 0;
  lastButtonState[2] = 0;
  lastButtonState[3] = 0;
  lastButtonState[4] = 0;
  lastButtonState[5] = 0;
  
  // Initialize Joystick Library
  Joystick.begin();                                     //Initialize the HID joystick functions
  delay(1000);
  getModelNumber(false);                                //Get LipSync model number 
  delay(10);
  sensitivityCounter = getJoystickSensitivity(false);   //Get saved joystick sensitivity parameter from EEPROM and sets the sensitivity counter
  delay(10);
  setJoystickInitialization(false);                     //Initialize the joystick ( Max and Neutral FSR values )
  delay(10);
  getJoystickCalibration(false);                        //Get FSR Max calibration values 
  delay(10);
  getFSREquation();                                     //Get FSR equations
  delay(10);
  getChangeTolerance(CHANGE_DEFAULT_TOLERANCE,false);    // Get change tolerance using max FSR readings and default tolerance percentage 
  delay(10);
  getPressureThreshold(false);                          //Initialize the pressure sensor
  delay(10);
  debugModeEnabled = getDebugMode(false);               //Get the debug mode state
  delay(10);
  rawModeEnabled = getRawMode(false);                   //Get the raw mode state
  delay(50); 
  joystickDeadzone = getDeadzone(false);                //Get the deadzone value 
  delay(10);
  buttonMode = getButtonMode(false);                    //Get saved joystick button mode parameter from EEPROM
  delay(10);
  getButtonMapping(false);                              //Get button mappings 
  delay(10);

  ledBlink(4, 250, 3);                                   //End the initialization visual feedback
  delay(10);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  
  settingsEnabled=serialSettings(settingsEnabled);              //Check to see if setting option is enabled in Lipsync
  
  xHigh = analogRead(X_DIR_HIGH_PIN);
  xLow = analogRead(X_DIR_LOW_PIN);
  yHigh = analogRead(Y_DIR_HIGH_PIN);
  yLow = analogRead(Y_DIR_LOW_PIN);

  //Check the FSR changes from previous reading and set the skip flag to true if the changes are in the change tolerance range
  bool skipChange = abs(xHigh - xHighPrev) < xHighChangeTolerance && abs(xLow - xLowPrev) < xLowChangeTolerance && abs(yHigh - yHighPrev) < yHighChangeTolerance && abs(yLow - yLowPrev) < yLowChangeTolerance;
  xHighPrev = xHigh;
  xLowPrev = xLow;
  yHighPrev = yHigh;
  yLowPrev = yLow;

  xHighYHigh = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
  xHighYLow = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));    //The sqrt() function raises input to power 1/2, returning a float type
  xLowYHigh = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));          //These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xLowYLow = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));         //a larger digital value with a positive application force, a large negative difference

  if ((xHighYHigh > joystickDeadzone) || (xHighYLow > joystickDeadzone) || (xLowYLow > joystickDeadzone) || (xLowYHigh > joystickDeadzone)) {
    //Map FSR values to (0 to 16 ) range 
    xHighMapped=getMappedFSRValue(xHigh, joystickDeadzone, xHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xHighEquation);
    xLowMapped=getMappedFSRValue(xLow, joystickDeadzone, xLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xLowEquation);
    yHighMapped=getMappedFSRValue(yHigh, joystickDeadzone, yHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yHighEquation);
    yLowMapped=getMappedFSRValue(yLow, joystickDeadzone, yLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yLowEquation);
      
    //Calculate the x and y delta values 
    xDelta = xHighMapped - xLowMapped;                            
    yDelta = yHighMapped - yLowMapped;   
      
    //Get the final X and Y output values for Joystick set axis function
    xOut = getXYValue(xDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);
    yOut = -getXYValue(yDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);

    //Serial out raw data if raw mode is enabled otherwise perform Joystick X and Y move 
    if(!skipChange) {
      (rawModeEnabled) ? sendRawData(xOut,yOut,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : setXYAxis(xOut,yOut);
    }
  } else{
    //Serial out raw data if raw mode is enabled otherwise perform Joystick X and Y move to center point
    (rawModeEnabled) ? sendRawData(0,0,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : setXYAxis(0,0);
  }

  if(!rawModeEnabled){
    sipAndPuffHandler(buttonMode);              //Pressure sensor sip and puff functions
  }

  //Debug information 
  if(debugModeEnabled && !rawModeEnabled) {
    
    Serial.print("LOG:3:");
    Serial.print(xHigh);
    Serial.print(",");
    Serial.print(xLow);
    Serial.print(",");
    Serial.print(yHigh);
    Serial.print(",");
    Serial.println(yLow); 
    delay(150);
  }
  
  pushButtonHandler(BUTTON_UP_PIN,BUTTON_DOWN_PIN);     //The joystick buttons function
  
  delay(JS_DELAY);                                      //The fixed delay for each action loop

}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***SET JOYSTICK OUTPUT X AND Y VALUES FUNCTION***//

void setXYAxis(int x, int y) {
  Joystick.setXAxis(x); 
  Joystick.setYAxis(y);   
}

//***GET MODEL NUMBER FUNCTION***//

void getModelNumber(bool responseEnabled) {
  EEPROM.get(0, modelNumber);
  if (modelNumber != 2) {                 //If the previous firmware was different model then factory reset the settings 
    modelNumber = 2;                      //And store the model number in EEPROM
    EEPROM.put(0, modelNumber);
    delay(10);
    factoryReset(false);
    delay(10);
  }  
  if(responseEnabled){
    Serial.println("SUCCESS:MN,0:2");
  }
}

//***GET VERSION FUNCTION***//

void getVersionNumber(void) {
  Serial.println("SUCCESS:VN,0:V1.17");
}

//***HID JOYSTICK SENSITIVITY FUNCTION***//

int getJoystickSensitivity(bool responseEnabled) {
  int sensitivity = SENSITIVITY_COUNTER;
  EEPROM.get(2, sensitivity);
  delay(5);
  if(sensitivity<0 || sensitivity >10){
    sensitivity = SENSITIVITY_COUNTER;
    EEPROM.put(2, sensitivity);
    delay(5);
  }
  if(responseEnabled){
    Serial.print("SUCCESS:SS,0:");
    Serial.println(sensitivity);      
  } 
  delay(5);
  return sensitivity;
}


//***INCREASE JOYSTICK SENSITIVITY FUNCTION***//

int increaseJoystickSensitivity(int sensitivity,bool cmdResponseEnabled) {
  sensitivity++;

  if (sensitivity == 11) {
    ledBlink(6, 50, 3);
    sensitivity = 10;
  } else {
    ledBlink(sensitivity+1, 100, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
  }
  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:"); 
  Serial.print("SS,1:");
  Serial.println(sensitivity); 
  delay(5);
  return sensitivity;
}

//***DECREASE JOYSTICK SENSITIVITY FUNCTION**//

int decreaseJoystickSensitivity(int sensitivity,bool cmdResponseEnabled) {
  sensitivity--;

  if (sensitivity == -1) {
    ledBlink(6, 50, 3);     // twelve very fast blinks
    sensitivity = 0;
  } else if (sensitivity == 0) {
    ledBlink(1, 350, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
  } else {
    ledBlink(sensitivity+1, 100, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
  }
  
  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:"); 
  Serial.print("SS,1:");
  Serial.println(sensitivity);  
  delay(5);
  return sensitivity;
}

//***GET PRESSURE THRESHOLD FUNCTION***//
void getPressureThreshold(bool responseEnabled) {
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  int pressureThreshold = PRESSURE_THRESHOLD;
  if(SERIAL_SETTINGS) {
    EEPROM.get(32, pressureThreshold);
    delay(5);
    if(pressureThreshold<=0 || pressureThreshold>50) {
      EEPROM.put(32, PRESSURE_THRESHOLD);
      delay(5);
      pressureThreshold = PRESSURE_THRESHOLD;
    }    
  } else {
    pressureThreshold = PRESSURE_THRESHOLD;
  }
  sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  if(responseEnabled) {
    Serial.print("SUCCESS:PT,0:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal);
    delay(5);
  }
}

//***SET PRESSURE THRESHOLD FUNCTION***//

void setPressureThreshold(int pressureThreshold, bool responseEnabled) {
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  if(SERIAL_SETTINGS && (pressureThreshold>0 && pressureThreshold<=50)) {
    EEPROM.put(32, pressureThreshold);
    delay(5); 
  } else {
    pressureThreshold = PRESSURE_THRESHOLD;
    delay(5); 
  }
  sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  if(responseEnabled) {
    Serial.print("SUCCESS:PT,1:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal); 
    delay(5);
  }
}

//***GET DEBUG MODE STATE FUNCTION***//

bool getDebugMode(bool responseEnabled) {
  bool debugState=DEBUG_MODE;
  if(SERIAL_SETTINGS) {
    EEPROM.get(34, debugState);
    delay(5);
    if(debugState!=0 && debugState!=1) {
      EEPROM.put(34, DEBUG_MODE);
      delay(5);
      debugState=DEBUG_MODE;
      }   
  } else {
    debugState=DEBUG_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS:DM,0:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SET DEBUG MODE STATE FUNCTION***//

bool setDebugMode(bool debugState,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    (debugState) ? EEPROM.put(34, 1) : EEPROM.put(34, 0);
    delay(5);    
  } else {
    debugState=DEBUG_MODE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DM,1:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SEND DEBUG DATA FUNCTION***//

void sendDebugData() {
  delay(100);
  Serial.print("LOG:1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(100);
  Serial.print("LOG:2:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(100);
}

//***SEND RAW DATA FUNCTION***//

void sendRawData(int x, int y, int action, int xUp, int xDown,int yUp,int yDown) {
  Serial.print("RAW:1:"); 
  Serial.print(x); 
  Serial.print(","); 
  Serial.print(y); 
  Serial.print(",");
  Serial.print(action); 
  Serial.print(":"); 
  Serial.print(xUp); 
  Serial.print(","); 
  Serial.print(xDown); 
  Serial.print(",");
  Serial.print(yUp); 
  Serial.print(",");
  Serial.println(yDown); 
}

//***GET RAW MODE STATE FUNCTION***//

bool getRawMode(bool responseEnabled) {
  bool rawState=RAW_MODE;
  if(SERIAL_SETTINGS) {
    EEPROM.get(36, rawState);
    delay(5);
    if(rawState!=0 && rawState!=1) {
      EEPROM.put(36, RAW_MODE);
      delay(5);
      rawState=RAW_MODE;
      }   
  } else {
    rawState=RAW_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS:RM,0:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***SET RAW MODE STATE FUNCTION***//

bool setRawMode(bool rawState,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    (rawState) ? EEPROM.put(36, 1) : EEPROM.put(36, 0);
    delay(5);    
  } else {
    rawState=RAW_MODE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:RM,1:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***GET DEADZONE VALUE FUNCTION***//

int getDeadzone(bool responseEnabled) {
  int deadzoneValue = JS_MOVE_RADIUS;
  if(SERIAL_SETTINGS) {
    EEPROM.get(38, deadzoneValue);
    delay(5);
    if(deadzoneValue<30 || deadzoneValue>99) {
      EEPROM.put(38, JS_MOVE_RADIUS);
      delay(5);
      deadzoneValue=JS_MOVE_RADIUS;
      }    
  } else {
    deadzoneValue=JS_MOVE_RADIUS;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DZ,0:");
    Serial.println(deadzoneValue); 
    delay(5);
   }
  return deadzoneValue;
}

//***SET DEADZONE VALUE FUNCTION***//

int setDeadzone(int deadzoneValue,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    if(deadzoneValue>30 && deadzoneValue<=99) {
      EEPROM.put(38, deadzoneValue);
      delay(5);
    } else {
      EEPROM.put(38, JS_MOVE_RADIUS);
      delay(5);
      deadzoneValue=JS_MOVE_RADIUS;
    }
  } else {
    deadzoneValue=JS_MOVE_RADIUS;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DZ,1:");
    Serial.println(deadzoneValue); 
    delay(5);
   }
  return deadzoneValue;
}


//***GET JOYSTICK INITIALIZATION FUNCTION***//

void getJoystickInitialization() {
  Serial.print("SUCCESS:IN,0:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(10);  
}

//***SET JOYSTICK INITIALIZATION FUNCTION***//

void setJoystickInitialization(bool cmdResponseEnabled) {

  ledOn(1);
  
  xHigh = getAverage(X_DIR_HIGH_PIN,10);               //Set the initial neutral x-high value of joystick
  delay(10);

  xLow = getAverage(X_DIR_LOW_PIN,10);                 //Set the initial neutral x-low value of joystick
  delay(10);

  yHigh = getAverage(Y_DIR_HIGH_PIN,10);               //Set the initial neutral y-high value of joystick
  delay(10);

  yLow = getAverage(Y_DIR_LOW_PIN,10);                 //Set the initial Initial neutral y-low value of joystick
  delay(10);

  //Set the neutral values 
  xHighPrev = xHighNeutral = xHigh;
  xLowPrev = xLowNeutral = xLow;
  yHighPrev = yHighNeutral = yHigh;
  yLowPrev = yLowNeutral = yLow;

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.print("IN,1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  
  ledClear();
}

//*** GET JOYSTICK CALIBRATION FUNCTION***//

void getJoystickCalibration(bool responseEnable) {
  //Get the max values from Memory 
  EEPROM.get(22, xHighMax);
  delay(10);
  EEPROM.get(24, xLowMax);
  delay(10);
  EEPROM.get(26, yHighMax);
  delay(10);
  EEPROM.get(28, yLowMax);
  delay(10);

  if(responseEnable){
    Serial.print("SUCCESS:CA,0:"); 
    Serial.print(xHighMax); 
    Serial.print(","); 
    Serial.print(xLowMax); 
    Serial.print(",");
    Serial.print(yHighMax); 
    Serial.print(",");
    Serial.println(xHighMax); 
  }
  delay(10);
}

//*** SET JOYSTICK CALIBRATION FUNCTION***//

void setJoystickCalibration(bool cmdResponseEnabled) {

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:0");                                                   //Start the joystick calibration sequence 
  ledBlink(4, 300, 3);

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:1"); 
  ledBlink(6, 500, 1);
  //yHighMax = analogRead(Y_DIR_HIGH_PIN);
  yHighMax = getAverage(Y_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:2"); 
  ledBlink(6, 500, 1);
  //xHighMax = analogRead(X_DIR_HIGH_PIN);
  xHighMax = getAverage(X_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:3"); 
  ledBlink(6, 500, 1);
  //yLowMax = analogRead(Y_DIR_LOW_PIN);
  yLowMax = getAverage(Y_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:4"); 
  ledBlink(6, 500, 1);
  //xLowMax = analogRead(X_DIR_LOW_PIN);
  xLowMax = getAverage(X_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);

  EEPROM.put(22, xHighMax);
  delay(10);
  EEPROM.put(24, xLowMax);
  delay(10);
  EEPROM.put(26, yHighMax);
  delay(10);
  EEPROM.put(28, yLowMax);
  delay(10);

  ledBlink(5, 250, 3);

  (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.print("CA,1:5:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(10);
}

//*** GET CHANGE TOLERANCE VALUE CALIBRATION FUNCTION***//

void getChangeTolerance(float changePercent, bool responseEnabled) {
  xHighChangeTolerance=(int)(xHighMax * (changePercent/100.0));
  xLowChangeTolerance=(int)(xLowMax * (changePercent/100.0));
  yHighChangeTolerance=(int)(yHighMax * (changePercent/100.0));
  yLowChangeTolerance=(int)(yLowMax * (changePercent/100.0));
  if(responseEnabled){
    Serial.print("SUCCESS:CT,0:"); 
    Serial.print(changePercent); 
    Serial.print(","); 
    Serial.print(xHighChangeTolerance); 
    Serial.print(","); 
    Serial.print(xLowChangeTolerance); 
    Serial.print(","); 
    Serial.print(yHighChangeTolerance); 
    Serial.print(",");
    Serial.println(yLowChangeTolerance); 
  }
  delay(10);
}

//***GET BUTTON MODE FUNCTION***//

int getButtonMode(bool responseEnabled) {
  int mode = 1;
  EEPROM.get(40, mode);                   //Get the button mode from memory 
  delay(5);
  if(mode !=1 && mode !=2){
    mode=BUTTON_MODE;                     //Set the button mode if it's not set before and save it in the memory 
    EEPROM.put(40, mode);
    delay(5);
  }
  if(responseEnabled){
    Serial.print("SUCCESS:BM,0:");
    Serial.println(mode); 
    delay(5);
  }
  return mode;
}

//***SET BUTTON MODE FUNCTION***//

int setButtonMode(int mode,bool cmdResponseEnabled) {                
  if(mode ==1 || mode ==2){
    ledBlink(mode, 250, 3);
    EEPROM.put(40, mode);                                                       //Set the button mode and save it in the memory 
    delay(5);
    (cmdResponseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
    Serial.print("BM,1:");
    Serial.println(mode); 
    delay(10);
  }
  return mode;
}

//***GET BUTTON MAPPING FUNCTION***//

void getButtonMapping(bool responseEnabled) {
  if (SERIAL_SETTINGS) {
    for (int i = 0; i < 6; i++) {
      int buttonMapping;
      EEPROM.get(42+i*2, buttonMapping);
      delay(5);
      if(buttonMapping<1 || buttonMapping >8) {
        EEPROM.put(42+i*2, actionButton[i]);
        delay(5);
      } else {
        actionButton[i]=buttonMapping;
        delay(5);
      }
    }
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:MP,0:");
    Serial.print(actionButton[0]); 
    Serial.print(actionButton[1]); 
    Serial.print(actionButton[2]); 
    Serial.print(actionButton[3]); 
    Serial.print(actionButton[4]); 
    Serial.println(actionButton[5]); 
    delay(5);
   }
}

//***SET BUTTON MAPPING FUNCTION***//

void setButtonMapping(int buttonMapping[],bool responseEnabled) {
  if (SERIAL_SETTINGS) {
   for(int i = 0; i < 6; i++){
    EEPROM.put(42+i*2, buttonMapping[i]);
    delay(5);
    actionButton[i]=buttonMapping[i];
    delay(5);
   }     
  } 
  if(responseEnabled) {
    Serial.print("SUCCESS:MP,1:");
    Serial.print(actionButton[0]); 
    Serial.print(actionButton[1]); 
    Serial.print(actionButton[2]); 
    Serial.print(actionButton[3]); 
    Serial.print(actionButton[4]); 
    Serial.println(actionButton[5]); 
    delay(5);
   }
}

//***FACTORY RESET FUNCTION***//

void factoryReset(bool responseEnabled) {
  if (SERIAL_SETTINGS) {
    int defaultButtonMapping[6] = {ACTION_BUTTON_1, ACTION_BUTTON_2, ACTION_BUTTON_3, ACTION_BUTTON_4, ACTION_BUTTON_5, ACTION_BUTTON_6};
    EEPROM.put(2, SENSITIVITY_COUNTER);
    delay(10);
    setPressureThreshold(PRESSURE_THRESHOLD,false);
    delay(10);
    EEPROM.put(34, DEBUG_MODE);
    delay(10);  
    EEPROM.put(36, RAW_MODE);
    delay(10); 
    EEPROM.put(38, JS_MOVE_RADIUS);
    delay(10);
    EEPROM.put(40, BUTTON_MODE);
    delay(10);
    setButtonMapping(defaultButtonMapping,false);
    delay(10);
    
    sensitivityCounter=SENSITIVITY_COUNTER;
    debugModeEnabled=DEBUG_MODE;  
    rawModeEnabled=RAW_MODE; 
    joystickDeadzone=JS_MOVE_RADIUS;
    buttonMode=BUTTON_MODE;

  }

  if(responseEnabled) {
    Serial.println("SUCCESS:FR,0:0");
    delay(5);
   }
   ledBlink(2, 250, 1);
}

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "SETTINGS" is in input string.
       if (settingsFlag==false && inString=="SETTINGS") {
        Serial.println("SUCCESS:SETTINGS");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString=="EXIT") {
        Serial.println("SUCCESS:EXIT");
       settingsFlag=false;                         //Set the return flag to false so settings actions can be exited
       }
       else if (settingsFlag==true && (inString.length()==(6) || inString.length()==(7) || inString.length()==(11)) && inString.charAt(2)==',' && inString.charAt(4)==':'){ //Check if the input parameter is true and the received string is 3 characters only
        inString.replace(",","");                 //Remove commas 
        inString.replace(":","");                 //Remove :
        writeSettings(inString); 
        settingsFlag=false;   
       }
       else {
        Serial.println("FAIL:SETTINGS");
        settingsFlag=false;      
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SPEED USING SOFTWARE***//

void writeSettings(String changeString) {
    char changeChar[changeString.length()+1];
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Get Model number : "MN,0:0"
    if(changeChar[0]=='M' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getModelNumber(true);
      delay(5);
    } 
    //Get version number : "VN,0:0"
    else if(changeChar[0]=='V' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getVersionNumber();
      delay(5);
    }   
    //Get joystick sensitivity value if received "SS,0:0", decrease the sensitivity if received "SS,1:1" and increase the sensitivity if received "SS,1:2"
    else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      sensitivityCounter = getJoystickSensitivity(true);
      delay(5);
    } else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      sensitivityCounter = decreaseJoystickSensitivity(sensitivityCounter,true);
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='1' && changeChar[3]=='2' && changeString.length()==4) {
      sensitivityCounter = increaseJoystickSensitivity(sensitivityCounter,true);
      delay(5);
    } 
     //Get pressure threshold values if received "PT,0:0" and pressure threshold values if received "PT,1:{threshold 1% to 50%}"
      else if(changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getPressureThreshold(true);
      delay(5);
    } else if (changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='1' && ( changeString.length()==4 || changeString.length()==5)) {
      String pressureThresholdString = changeString.substring(3);
      setPressureThreshold(pressureThresholdString.toInt(),true);
      delay(5);
    } 
     //Get debug mode value if received "DM,0:0" , set debug mode value to 0 if received "DM,1:0" and set debug mode value to 1 if received "DM,1:1"
     else if(changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = getDebugMode(true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(0,true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(1,true);
      delay(5);
    } 
    //Get raw mode value if received "RM,0:0" , set raw mode value to 0 if received "RM,1:0" and set raw mode value to 1 if received "RM,1:1"
     else if(changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = getRawMode(true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = setRawMode(0,true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      rawModeEnabled = setRawMode(1,true);
      delay(5);
    } 
     //Get deadzone value if received "DZ,0:0" , set deadzone value if received "DZ,1:{Value 1 to 99}" 
     else if(changeChar[0]=='D' && changeChar[1]=='Z' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      joystickDeadzone = getDeadzone(true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='Z' && changeChar[2]=='1' && (changeString.length()==4 || changeString.length()==5)) {
      String deadzoneString = changeString.substring(3);
      joystickDeadzone = setDeadzone(deadzoneString.toInt(),true);
      delay(5);
    }
     //Get joystick initialization values if received "IN,0:0" and perform joystick initialization if received "IN,1:1"
     else if(changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getJoystickInitialization();
      delay(5);
    } else if (changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setJoystickInitialization(true);
      delay(5);
    } 
     //Get joystick calibration values if received "CA,0:0" and perform joystick calibration if received "CA,1:1"
      else if(changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getJoystickCalibration(true);
      delay(5);
    } else if (changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setJoystickCalibration(true);
      delay(5);
    } 
     //Get change tolerance values if received "CT,0:0" 
      else if(changeChar[0]=='C' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getChangeTolerance(CHANGE_DEFAULT_TOLERANCE,true);
      delay(5);
    }
    //Get Button mode : "BM,0:0" , Set Button mode to 1 : "BM,1:1" , Set Button mode to 2 : "BM,1:2"
    else if(changeChar[0]=='B' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      buttonMode = getButtonMode(true);
      delay(5);
    } else if(changeChar[0]=='B' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      buttonMode = setButtonMode(1,true);
      delay(5);
    } else if (changeChar[0]=='B' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='2' && changeString.length()==4) {
      buttonMode = setButtonMode(2,true);
      delay(5);
    } 
    //Get Button mapping : "MP,0:0" , Set Button mapping : "MP,1:012345"
    else if (changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getButtonMapping(true);
      delay(5);
    } else if(changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='1' && changeString.length()==9) {
      int buttonTempMapping[6];
      for(int i = 0; i< 6; i++){
         buttonTempMapping[i]=changeChar[3+i] - '0';
      }
      setButtonMapping(buttonTempMapping,true);
      delay(5);
    }
     //Perform factory reset if received "FR,0:0"
     else if(changeChar[0]=='F' && changeChar[1]=='R' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      factoryReset(true);
      delay(5);
    } else {
      Serial.println("FAIL:SETTINGS");
      delay(5);        
      }
    
}

//***GET AVERAGE ANALOG VALUE FUNCTION***//

int getAverage(int dataPin, int number) {
  long averageValue=0;
  for (int i=0; i<number; i++) {
     averageValue+=analogRead(dataPin);
     delay(2);
  }
  averageValue=averageValue/number;
  delay(10);
  return averageValue;
}

//***GET X AND Y VALUE IN (-maxOutputValue,maxOutputValue) RANGE FOR HOST DEVICE BASED ON MAPPED FSR VALUE AND THE DEGREE 5 POLYNOMIAL EQUATION COEFFICIENTS FUNCTION***//

int getXYValue(float rawValue, int deadzoneOutputValue , int maxOutputValue, _equationCoef equationCoef) {
  int xySign = sgn(rawValue);                                                 //Get the sign of input
  rawValue = abs(rawValue);                                                   //Solve for output regardless of the input sign and multiply the output by the sign ( the polynomial in quadrant 1 and 3 )
  int xyValue = (int)((equationCoef._equationACoef*pow(rawValue,5))+(equationCoef._equationBCoef*pow(rawValue,4))+(equationCoef._equationCCoef*pow(rawValue,3))+(equationCoef._equationDCoef*pow(rawValue,2))+(equationCoef._equationECoef*rawValue)+equationCoef._equationFCoef);
  delay(1);                                                                   //The points in quadrant 1 and 3 only (mirror (+,+) to (-,-) )
  xyValue = (xyValue >= maxOutputValue-deadzoneOutputValue) ? maxOutputValue: (xyValue<=deadzoneOutputValue) ? 0: xyValue;        //Set output value to maximum value if it's in maximum deadzone area and set output value to center value if it's in center deadzone area 
  delay(1);
  xyValue = xySign*xyValue;
  delay(1);
  return xyValue;
}

//***GET MAPPED FSR VALUE BASED ON THE EQUATION COEFFICIENTS FUNCTION***//

float getMappedFSRValue(int rawValue, int deadzoneInputValue, int neutralValue, float deadzoneOutputValue, float maxOutputValue, _equationCoef equationCoef) {
  float mappedValue;
  rawValue = (rawValue <= (neutralValue+deadzoneInputValue) && rawValue >=(neutralValue-deadzoneInputValue))? neutralValue:rawValue; //Set input value to neutral value if it's in neutral deadzone area 
  mappedValue = ((equationCoef._equationDCoef*pow(rawValue,2))+(equationCoef._equationECoef*rawValue));                    //Solve for mapped FSR Value using the coefficients of the equation ( result : value from 0 to 16 )
  mappedValue = (mappedValue>=maxOutputValue-deadzoneOutputValue) ? maxOutputValue: (mappedValue<=deadzoneOutputValue) ? 0.00: mappedValue;                     //Set output value to maximum value if it's in maximum deadzone area and set output value to center value if it's in center deadzone area 
  return mappedValue;
}

//***GET THE MAPPED FSR EQUATIONS FUNCTION***//

void getFSREquation() {
  //Create equations to map FSR behavior 
  xHighEquation = setFSREquation(xHighNeutral,xHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  xLowEquation = setFSREquation(xLowNeutral,xLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yHighEquation = setFSREquation(yHighNeutral,yHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yLowEquation = setFSREquation(yLowNeutral,yLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
}

//***SET THE EQUATION COEFFICIENTS FOR MAPPING RAW FSR VALUES TO MAPPED VALUE FUNCTION***//

_equationCoef setFSREquation(int x1,int x2,int y1,int y2) {
  
  //Convert input values from int to float
  float x1Value = (float)x1;
  float x2Value = (float)x2;
  float y1Value = (float)y1;
  float y2Value = (float)y2;

  //Solve for coefficient d
  float dValue = (y2Value - ((y1Value*x2Value)/x1Value))/(x2Value*(x2Value-x1Value));
  
  //Solve for coefficient e
  float eValue = (y1Value/x1Value)-dValue*x1Value;

  //Output coefficients ( all others are zero )
  _equationCoef resultFactor = {0.00, 0.00, 0.00, dValue, eValue, 0.00};
  return resultFactor;
}

//***FIND SIGN OF VARIABLE FUNCTION***//

int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

//***PUSH BUTTON SPEED HANDLER FUNCTION***//

void pushButtonHandler(int switchPin1, int switchPin2) {
  
  if (digitalRead(switchPin1) == LOW) {
    delay(250);
    if (switchPreviousState[0] == HIGH && digitalRead(switchPin2) == HIGH) {
      //Switch 1 state changed from 1 to 0
      switchTimer[0] = millis();
      switchPreviousState[0] = LOW;
      switchPreviousState[1] = HIGH;
    } else if (switchPreviousState[0] == HIGH && digitalRead(switchPin2) == LOW && switchPreviousState[1] == HIGH) {
      //Switch 1 state and switch 2 state changed from 1 to 0
      switchTimer[2] = millis();      
      switchPreviousState[0] = LOW;
      switchPreviousState[1] = LOW;
    }
  } 
  else if (digitalRead(switchPin1) == HIGH) {
    if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == HIGH && (millis() - switchTimer[0] >= LONG_PRESS_TIME*1000)) {
      //Switch 1 was released after 2 seconds ( switch 1 long press)
      buttonMode = setButtonMode(2,false);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == HIGH && (millis() - switchTimer[0] < LONG_PRESS_TIME*1000)) {
      //Switch 1 was released before 2 seconds ( switch 1 short press)
      sensitivityCounter = increaseJoystickSensitivity(sensitivityCounter,false); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == LOW && (millis() - switchTimer[2] >= LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released after 2 seconds ( switch 1 and 2 long press)
      setJoystickCalibration(false);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == LOW && (millis() - switchTimer[2] < LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released before 2 seconds ( switch 1 and 2 short press)
      setJoystickInitialization(false); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
  }

  if (digitalRead(switchPin2) == LOW) {
    delay(250);
    if (switchPreviousState[1] == HIGH && digitalRead(switchPin1) == HIGH) {
      //Switch 1 state changed from 1 to 0
      switchTimer[1] = millis();
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = LOW;
    } else if (switchPreviousState[1] == HIGH && digitalRead(switchPin1) == LOW && switchPreviousState[0] == HIGH) {
      //Switch 1 state and switch 2 state changed from 1 to 0
      switchTimer[2] = millis();   
      switchPreviousState[0] = LOW;
      switchPreviousState[1] = LOW;   
    } 
  } 
  else if (digitalRead(switchPin2) == HIGH) {
    if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == HIGH && (millis() - switchTimer[1] >= LONG_PRESS_TIME*1000)) {
      //Switch 2 was released after 2 seconds ( switch 1 long press)
      buttonMode = setButtonMode(1,false);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == HIGH && (millis() - switchTimer[1] < LONG_PRESS_TIME*1000)) {
      //Switch 2 was released before 2 seconds ( switch 1 short press)
      sensitivityCounter=decreaseJoystickSensitivity(sensitivityCounter,false); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == LOW && (millis() - switchTimer[2] >= LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released after 2 seconds ( switch 1 and 2 long press)
      setJoystickCalibration(false);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == LOW && (millis() - switchTimer[2] < LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released before 2 seconds ( switch 1 and 2 short press)
      setJoystickInitialization(false); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
  }
  delay(5);
}

//***SIP AND PUFF ACTION HANDLER FUNCTION***//

void sipAndPuffHandler(int mode) {
  joystickPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPressure < puffThreshold) {
    switch (mode) {
      case 1:                                             //Default button mode (short/long puff)
        while (joystickPressure < puffThreshold) {
          joystickPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          puffCount++;                                    //Threshold counter
          delay(5);
        }
        if (puffCount < 150) {                            //Short puff
          if (!lastButtonState[0]) {
              Joystick.pressButton(actionButton[0]);
              delay(150);
              Joystick.releaseButton(actionButton[0]);
              delay(50);
              lastButtonState[0] = 0;
            }
          } else if (puffCount > 150 && puffCount < 450) {  //Long puff
            if (!lastButtonState[2]) {
              Joystick.pressButton(actionButton[2]);
              delay(150);
              Joystick.releaseButton(actionButton[2]);
              delay(50);
              lastButtonState[2] = 0;
            } 
          } else if (puffCount > 450) {                      //Very long puff
            if (!lastButtonState[4]) {
              Joystick.pressButton(actionButton[4]);
              delay(150);
              Joystick.releaseButton(actionButton[4]);
              delay(50);
              lastButtonState[4] = 0;
            } 
          }
        puffCount = 0;
        break;
      case 2:                                         //Analog trigger button mode ( Option to hold puff )
        Joystick.pressButton(actionButton[0]);
        delay(10);
        lastButtonState[0] = 1;
        break;
    }
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPressure > sipThreshold) {
    switch (mode) {
      case 1:                                           //Default button mode (short/long puff)
        while (joystickPressure > sipThreshold) {
          joystickPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          sipCount++;
          delay(5);
        }
        if (sipCount < 150) {                           //Short sip
          if (!lastButtonState[1]) {
              Joystick.pressButton(actionButton[1]);
              delay(150);
              Joystick.releaseButton(actionButton[1]);
              delay(50);
              lastButtonState[1] = 0;
            } 
          } else if (sipCount > 150 && sipCount < 450) { //Long sip
            ledBlink(1, 250, 1); 
            if (!lastButtonState[3]) {
              Joystick.pressButton(actionButton[3]);
              delay(50);
              lastButtonState[3] = 1;
            } 
            else {
              Joystick.releaseButton(actionButton[3]);
              delay(50);  
              lastButtonState[3] = 0;    
            }
          } else if (sipCount > 450) {                    //Very long sip
             if (!lastButtonState[5]) {
              Joystick.pressButton(actionButton[5]);
              delay(150);
              Joystick.releaseButton(actionButton[5]);
              delay(50);
              lastButtonState[5] = 0;
            } 
          }
        sipCount = 0;
        break;
      case 2:                                             //Analog trigger button mode ( Option to hold sip )
        Joystick.pressButton(actionButton[1]);
        delay(10);
        lastButtonState[1] = 1;
        break;
    }
  }
  if (joystickPressure <= sipThreshold && joystickPressure >= puffThreshold && mode==2) {       //Release buttons in analog trigger button mode
    Joystick.releaseButton(actionButton[0]);
    Joystick.releaseButton(actionButton[1]);
    delay(10);
    lastButtonState[0] = 0;
    lastButtonState[1] = 0;
    
  }
}

//***SIP AND PUFF RAW ACTION HANDLER FUNCTION***//

int sipAndPuffRawHandler() {
  int currentAction = 0;
  joystickPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPressure < puffThreshold) {
        delay(5);
        currentAction = 1;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPressure > sipThreshold) {
        delay(5);
        currentAction = 2;
  }
  return currentAction;
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
