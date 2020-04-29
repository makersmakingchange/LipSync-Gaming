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
//VERSION: 1.16 (28 April 2020) 


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

//***CUSTOMIZE VARIABLES***//

#define PRESSURE_THRESHOLD 0.5                   //Pressure sip and puff threshold 
#define JS_DELAY 10                              //The fixed delay for each loop action
#define JS_FSR_DEADZONE 60                       //The deadzone for input FSR analog value 

//***DON'T CHANGE VARIABLES***//

#define JS_MAPPED_IN_DEADZONE 0.50
#define JS_MAPPED_IN_NEUTRAL 12
#define JS_MAPPED_IN_MAX 16.00
#define JS_OUT_DEAD_ZONE 1
#define JS_OUT_MAX 127
#define DEBUG_MODE false
#define BUTTON_MODE 0                             //Set button mode ( 0 = Digital buttons , 1 = Analog buttons )
#define LONG_PRESS_TIME 2



//***Map Sip & Puff actions to joystick buttons for mode 1***//

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

int xHighNeutral, xLowNeutral, yHighNeutral, yLowNeutral;                       //Neutral FSR values at the resting position 

int xHighMax, xLowMax, yHighMax, yLowMax;                                       //Max FSR values which are set to the values from EEPROM


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

int sensitivityCounter = 5;                             //Declare variables for sensitivity adjustment  

float sipThreshold;                                     //Declare sip and puff variables 
float puffThreshold;
float joystickPress;
unsigned int puffCount;
unsigned int sipCount;

int lastButtonState[5];                                 //Last state of the buttons
int buttonMode;                                         //The button mode variable 

long switchTimer[3];
bool switchPreviousState[2];

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
  
  // Initialize Joystick Library
  Joystick.begin();
  delay(1000);
  
  pressureSensorInitialization();                    //Initialize the pressure sensor
  delay(10);

  //Initialize the last state of buttons
  lastButtonState[0] = 0;
  lastButtonState[1] = 0;
  lastButtonState[2] = 0;
  lastButtonState[3] = 0;
  lastButtonState[4] = 0;
  lastButtonState[5] = 0;

  getJoystickSensitivity();                              //Get saved joystick sensitivity parameter from EEPROM and sets the sensitivity counter
  delay(10);
  getButtonMode();                                       //Get saved joystick button mode parameter from EEPROM
  delay(100); 
  joystickInitialization();                               //Initialize the joystick ( Max and Neutral FSR values )
  delay(10);


  ledBlink(4, 250, 3);                                //End the initialization visual feedback

  displayVersion();                                   //Display the firmware version 

  Serial.print("Sensitivity level: ");                //Print the sensitivity level ( stored in a variable starting at 0 )
  Serial.println(sensitivityCounter+1);
  delay(5);


  Serial.print("Button Mode: ");   //Print the button mode 
  Serial.println(buttonMode+1);
  delay(5);

  //Debug information 
  
  if(DEBUG_MODE) {

    delay(2000);
    
    Serial.print("xHighMax: ");
    Serial.print(xHighMax);
    Serial.print(",xLowMax: ");
    Serial.print(xLowMax);
    Serial.print(",yHighMax: ");
    Serial.print(yHighMax);
    Serial.print(",yLow: ");
    Serial.println(yLowMax);
  
    delay(10);
  
    Serial.print("xHighNeutral: ");
    Serial.print(xHighNeutral);
    Serial.print(",xLowNeutral: ");
    Serial.print(xLowNeutral);
    Serial.print(",yHighNeutral: ");
    Serial.print(yHighNeutral);
    Serial.print(",yLowNeutral: ");
    Serial.println(yLowNeutral);    
  }

  delay(5);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  
  settingsEnabled=serialSettings(settingsEnabled);              //Check to see if setting option is enabled in Lipsync
  
  xHigh = analogRead(X_DIR_HIGH_PIN);
  xLow = analogRead(X_DIR_LOW_PIN);
  yHigh = analogRead(Y_DIR_HIGH_PIN);
  yLow = analogRead(Y_DIR_LOW_PIN);


  //Map FSR values to (0 to 16 ) range 
  float xHighMapped=getMappedFSRValue(xHigh, JS_FSR_DEADZONE, xHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xHighEquation);
  float xLowMapped=getMappedFSRValue(xLow, JS_FSR_DEADZONE, xLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xLowEquation);
  float yHighMapped=getMappedFSRValue(yHigh, JS_FSR_DEADZONE, yHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yHighEquation);
  float yLowMapped=getMappedFSRValue(yLow, JS_FSR_DEADZONE, yLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yLowEquation);
    
  //Calculate the x and y delta values 
  float xDelta = xHighMapped - xLowMapped;                            
  float yDelta = yHighMapped - yLowMapped;   
    
  //Get the final X and Y output values for Joystick set axis function
  int xOut = getXYValue(xDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);
  int yOut = -getXYValue(yDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);
 
  //Perform Joystick X and Y move 
  Joystick.setXAxis(xOut); 
  Joystick.setYAxis(yOut); 

  sipAndPuffHandler(buttonMode);                                                       //Pressure sensor sip and puff functions
  delay(5);
  pushButtonHandler(BUTTON_UP_PIN,BUTTON_DOWN_PIN);                                    //The joystick buttons function
  delay(JS_DELAY);                                                                     //The fixed delay for each action loop

}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

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

//***JOYSTICK INITIALIZATION FUNCTION***//

void joystickInitialization(void) {

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
  xHighNeutral = xHigh;
  xLowNeutral = xLow;
  yHighNeutral = yHigh;
  yLowNeutral = yLow;

  //Get the max values from Memory 
  EEPROM.get(22, xHighMax);
  delay(10);
  EEPROM.get(24, xLowMax);
  delay(10);
  EEPROM.get(26, yHighMax);
  delay(10);
  EEPROM.get(28, yLowMax);
  delay(10);

  //Create equations to map FSR behavior 
  xHighEquation = getFSREquation(xHighNeutral,xHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  xLowEquation = getFSREquation(xLowNeutral,xLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yHighEquation = getFSREquation(yHighNeutral,yHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yLowEquation = getFSREquation(yLowNeutral,yLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);

  Serial.println("Initialization is complete.");

  ledClear();
}

//***JOYSTICK CALIBRATION FUNCTION***//

void joystickCalibration(void) {
  
  Serial.println("Prepare for joystick calibration!");       //Start the joystick calibration sequence 
  Serial.println(" ");
  ledBlink(4, 300, 3);

  Serial.println("Move mouthpiece to max vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  //yHighMax = analogRead(Y_DIR_HIGH_PIN);
  yHighMax = getAverage(Y_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);
  Serial.println(yHighMax);

  Serial.println("Move mouthpiece to max horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  //xHighMax = analogRead(X_DIR_HIGH_PIN);
  xHighMax = getAverage(X_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);
  Serial.println(xHighMax);

  Serial.println("Move mouthpiece to max vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  //yLowMax = analogRead(Y_DIR_LOW_PIN);
  yLowMax = getAverage(Y_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);
  Serial.println(yLowMax);

  Serial.println("Move mouthpiece to max horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  //xLowMax = analogRead(X_DIR_LOW_PIN);
  xLowMax = getAverage(X_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);
  Serial.println(xLowMax);

  EEPROM.put(22, xHighMax);
  delay(10);
  EEPROM.put(24, xLowMax);
  delay(10);
  EEPROM.put(26, yHighMax);
  delay(10);
  EEPROM.put(28, yLowMax);
  delay(10);

  ledBlink(5, 250, 3);

  Serial.println("Calibration is complete.");
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

//***GET THE EQUATION COEFFICIENTS FOR MAPPING RAW FSR VALUES TO MAPPED VALUE FUNCTION***//

_equationCoef getFSREquation(int x1,int x2,int y1,int y2) {
  
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
      setButtonMode(1);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == HIGH && (millis() - switchTimer[0] < LONG_PRESS_TIME*1000)) {
      //Switch 1 was released before 2 seconds ( switch 1 short press)
      increaseJoystickSensitivity(); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == LOW && (millis() - switchTimer[2] >= LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released after 2 seconds ( switch 1 and 2 long press)
      joystickCalibration();
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[0] == LOW && digitalRead(switchPin2) == HIGH && switchPreviousState[1] == LOW && (millis() - switchTimer[2] < LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released before 2 seconds ( switch 1 and 2 short press)
      joystickInitialization(); 
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
      setButtonMode(0);
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == HIGH && (millis() - switchTimer[1] < LONG_PRESS_TIME*1000)) {
      //Switch 2 was released before 2 seconds ( switch 1 short press)
      decreaseJoystickSensitivity(); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == LOW && (millis() - switchTimer[2] >= LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released after 2 seconds ( switch 1 and 2 long press)
      joystickCalibration();
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
    else if (switchPreviousState[1] == LOW && digitalRead(switchPin1) == HIGH && switchPreviousState[0] == LOW && (millis() - switchTimer[2] < LONG_PRESS_TIME*1000)) {
      //Switch 1 and switch 2 were released before 2 seconds ( switch 1 and 2 short press)
      joystickInitialization(); 
      switchPreviousState[0] = HIGH;
      switchPreviousState[1] = HIGH;
    } 
  }
  delay(5);
}

//***SIP AND PUFF BUTTON HANDLER FUNCTION***//

void sipAndPuffHandler(int mode) {

  joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (joystickPress < puffThreshold) {
    switch (mode) {
      case 0:                                             //Default button mode (short/long puff)
        while (joystickPress < puffThreshold) {
          joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          puffCount++;                                    //Threshold counter
          delay(5);
        }
        if (puffCount < 150) {                            //Short puff
          if (!lastButtonState[0]) {
              Joystick.pressButton(actionButton1);
              delay(150);
              Joystick.releaseButton(actionButton1);
              delay(50);
              lastButtonState[0] = 0;
            }
          } else if (puffCount > 150 && puffCount < 450) {  //Long puff
            if (!lastButtonState[2]) {
              Joystick.pressButton(actionButton3);
              delay(150);
              Joystick.releaseButton(actionButton3);
              delay(50);
              lastButtonState[2] = 0;
            } 
          } else if (puffCount > 450) {                      //Very long puff
            if (!lastButtonState[4]) {
              Joystick.pressButton(actionButton5);
              delay(150);
              Joystick.releaseButton(actionButton5);
              delay(50);
              lastButtonState[4] = 0;
            } 
          }
        puffCount = 0;
        break;
      case 1:                                         //Analog trigger button mode ( Option to hold puff )
        Joystick.pressButton(actionButton1);
        delay(10);
        lastButtonState[0] = 1;
        break;
    }
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (joystickPress > sipThreshold) {
    switch (mode) {
      case 0:                                           //Default button mode (short/long puff)
        while (joystickPress > sipThreshold) {
          joystickPress = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
          sipCount++;
          delay(5);
        }
        if (sipCount < 150) {                           //Short sip
          if (!lastButtonState[1]) {
              Joystick.pressButton(actionButton2);
              delay(150);
              Joystick.releaseButton(actionButton2);
              delay(50);
              lastButtonState[1] = 0;
            } 
          } else if (sipCount > 150 && sipCount < 450) { //Long sip
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
          } else if (sipCount > 450) {                    //Very long sip
             if (!lastButtonState[5]) {
              Joystick.pressButton(actionButton6);
              delay(150);
              Joystick.releaseButton(actionButton6);
              delay(50);
              lastButtonState[5] = 0;
            } 
          }
        sipCount = 0;
        break;
      case 1:                                             //Analog trigger button mode ( Option to hold sip )
        Joystick.pressButton(actionButton2);
        delay(10);
        lastButtonState[1] = 1;
        break;
    }
  }
  if (joystickPress <= sipThreshold && joystickPress >= puffThreshold && mode==1) {       //Release buttons in analog trigger button mode
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
  EEPROM.get(32, buttonMode);                   //Get the button mode from memory 
  delay(10);
  if(buttonMode !=0 && buttonMode !=1){
    buttonMode=BUTTON_MODE;                     //Set the button mode if it's not set before and save it in the memory 
    EEPROM.put(32, buttonMode);
    delay(10);
  }
}

//***SET BUTTON MODE FUNCTION***//

void setButtonMode(int mode) {                
  if(mode ==0 || mode ==1){
    buttonMode=mode;
    ledBlink(mode+1, 250, 3);
    EEPROM.put(32, buttonMode);                 //Set the button mode and save it in the memory 
    delay(10);
    Serial.print("Button mode: ");
    Serial.println(buttonMode+1);
  }
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
       Serial.println("C,(0 or 1)");
       Serial.println("B,(1 or 2)");
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
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Increase the sensitivity if received "S,+" and decrease the sensitivity if received "S,-"
    if(changeChar[0]=='S' && changeChar[1]=='+') {
      increaseJoystickSensitivity();
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='-') {
      decreaseJoystickSensitivity();
      delay(5);
    } 

     //Perform joystick initialization if received "C,0" and joystick calibration if received "C,1"
     if(changeChar[0]=='C' && changeChar[1]=='0') {
      joystickInitialization();
      delay(5);
    } else if (changeChar[0]=='C' && changeChar[1]=='1') {
      joystickCalibration();
      delay(5);
    } 

    //Change Button mode
    if(changeChar[0]=='B' && changeChar[1]=='1') {
      setButtonMode(0);
      delay(5);
    } else if (changeChar[0]=='B' && changeChar[1]=='2') {
      setButtonMode(1);
      delay(5);
    } 
    
}

//***DISPLAY VERSION FUNCTION***//

void displayVersion(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("LipSync Gaming V1.16 (28 April 2020)");
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

//***HID JOYSTICK SENSITIVITY FUNCTION***//

void getJoystickSensitivity(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  if(var>=0){
    sensitivityCounter = var;
  } 
  else {
    EEPROM.put(2, sensitivityCounter);
    delay(5);
    Serial.println("Sensitivity setup for the first time.");
  }
}


//***INCREASE JOYSTICK SENSITIVITY FUNCTION***//

void increaseJoystickSensitivity(void) {
  sensitivityCounter++;

  if (sensitivityCounter == 11) {
    ledBlink(6, 50, 3);
    sensitivityCounter = 10;
  } else {
    ledBlink(sensitivityCounter+1, 100, 1);
    EEPROM.put(2, sensitivityCounter);
    delay(25);
  }
   Serial.print("Sensitivity level: ");
   Serial.println(sensitivityCounter+1);    
   delay(5);
}

//***DECREASE JOYSTICK SENSITIVITY FUNCTION**//

void decreaseJoystickSensitivity(void) {
  sensitivityCounter--;

  if (sensitivityCounter == -1) {
    ledBlink(6, 50, 3);     // twelve very fast blinks
    sensitivityCounter = 0;
  } else if (sensitivityCounter == 0) {
    ledBlink(1, 350, 1);
    EEPROM.put(2, sensitivityCounter);
    delay(25);
  } else {
    ledBlink(sensitivityCounter+1, 100, 1);
    EEPROM.put(2, sensitivityCounter);
    delay(25);
  }

   Serial.print("Sensitivity level: ");
   Serial.println(sensitivityCounter+1);
   delay(5);
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  
  float nominalJoystickValue = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  sipThreshold = nominalJoystickValue + PRESSURE_THRESHOLD;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = nominalJoystickValue - PRESSURE_THRESHOLD;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  
}
