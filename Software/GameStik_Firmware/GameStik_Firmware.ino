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
//VERSION: 1.1 (15 October 2018) 


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

#define PRESSURE_JOYSTICK A5                        // Sip & Puff Pressure Transducer Pin - analog input pin A5
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
  false, false, false);                         //Defining the joystick REPORT_ID and type

int button1 = 0;
int button2 = 1;
int button3 = 2;
int button4 = 3;
int button5 = 4;
int button6 = 5;
int button7 = 6;
int button8 = 7;

int x_h, y_h, x_l, y_l;   

int speed_counter = 2;
int fixed_delay = 10;

int joystick_delay;

float sip_threshold;
float puff_threshold;
float joystick_press;
float joystick_back;

unsigned int puff_count;
unsigned int sip_count;

int last_button_state[8];       // Last state of the button

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // setting baud rate for serial coms for diagnostic data return from microcontroller
  Serial1.begin(115200);                          // setting baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2
  pinMode(TRANS_CONTROL, OUTPUT);                 // transistor pin output
  pinMode(PIO4, OUTPUT);                          // command mode pin output
  
  pinMode(PRESSURE_JOYSTICK, INPUT);                // pressure sensor pin input
  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          // increase joystick speed button
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        // decrease joystick speed button

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  while(!Serial1);
  
  // Initialize Joystick Library
  Joystick.begin();
  delay(10);
  Pressure_Sensor_Initialization();
  delay(10);

  //Initialize the last state of buttons
  last_button_state[0] = 0;
  last_button_state[1] = 0;
  last_button_state[2] = 0;
  last_button_state[3] = 0;
  last_button_state[4] = 0;
  last_button_state[5] = 0;
  last_button_state[6] = 0;
  last_button_state[7] = 0;

  Joystick_Speed_Value();                         // reads saved joystick speed parameter from EEPROM
  delay(10);
  int exec_time = millis();
  Serial.print("Configuration time: ");
  Serial.println(exec_time);

  blink(4, 250, 3);                               // end initialization visual feedback

  Display_Feature_List();  
  joystick_delay = pow(1.6,(9-speed_counter))*fixed_delay;

  Serial.print("Speed level: ");
  Serial.println((9-speed_counter));
  delay(5);


}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  x_h = analogRead(X_DIR_HIGH);
  x_l = analogRead(X_DIR_LOW);
  y_h = analogRead(Y_DIR_LOW);
  y_l = analogRead(Y_DIR_HIGH);

  x_h = map(x_h, 0, 1023, 0, 16);
  x_l = map(x_l, 0, 1023, 0, 16);
  y_h = map(y_h, 0, 1023, 0, 16);
  y_l = map(y_l, 0, 1023, 0, 16);

  int xx_tmp = x_h - x_l;
  int yy_tmp = y_h - y_l;
  
  int xx = (xx_tmp >= 0)? sq(xx_tmp):-sq(xx_tmp);
  int yy = (yy_tmp >= 0)? sq(yy_tmp):-sq(yy_tmp);
  xx -= (xx_tmp >= 0)? int(sqrt(yy_tmp)):-int(sqrt(-yy_tmp));
  yy -= (yy_tmp >= 0)? int(sqrt(xx_tmp)):-int(sqrt(-xx_tmp));

  xx = constrain(xx, -128, 128);
  yy = constrain(yy, -128, 128);

  xx = map(xx, -128, 128, 0, 1023);
  yy = map(yy, -128, 128, 0, 1023);

  Joystick.setXAxis(xx);
  Joystick.setYAxis(yy);
  
  delay(joystick_delay);
  
  //joystick speed control push button functions below

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      //Additional function 
    } else {
      Increase_Joystick_Speed();      // increase joystick speed with push button up
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      //Additional function 
    } else {
      Decrease_Joystick_Speed();      // decrease joystick speed with push button down
    }
  }

  //pressure sensor sip and puff functions below

  joystick_press = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;

  if (joystick_press < puff_threshold) {
    while (joystick_press < puff_threshold) {
      joystick_press = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      puff_count++;                                                             //Threshold counter
      delay(5);
    }
      if (puff_count < 150) {
        //Press joystick button number 5 or button A/X in XAC
        if (!last_button_state[4]) {
          Joystick.pressButton(button5);
          delay(250);
          Joystick.releaseButton(button5);
          delay(50);
          last_button_state[0] = 0;
        }
      } else if (puff_count > 150 && puff_count < 500) {
        //Press joystick button number 7 or button View in XAC
        if (!last_button_state[6]) {
          Joystick.pressButton(button7);
          delay(250);
          Joystick.releaseButton(button7);
          delay(50);
          last_button_state[2] = 0;
        } 
      } 

    puff_count = 0;
  }

  if (joystick_press > sip_threshold) {
    while (joystick_press > sip_threshold) {
      joystick_press = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      sip_count++;                                                                //Threshold counter
      delay(5);
    }
      if (sip_count < 150) {
        //Press joystick button number 6 or button B/Y in XAC
        if (!last_button_state[5]) {
          Joystick.pressButton(button6);
          delay(250);
          Joystick.releaseButton(button6);
          delay(50);
          last_button_state[1] = 0;
        } 
      } else if (sip_count > 150 && sip_count < 500) {
        //Press joystick button number 8 or button Menu/X1 in XAC
         if (!last_button_state[7]) {
          Joystick.pressButton(button8);
          delay(250);
          Joystick.releaseButton(button8);
          delay(50);
          last_button_state[3] = 0;
        } 
      } else {
        delay(5);
      }
    sip_count = 0;
  }
  
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------


void Display_Feature_List(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the GameStik firmware");
  Serial.println(" ");
  Serial.println("VERSION: 1.1 (15 October 2018)");
  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println(" ");

}


//***LED BLINK FUNCTIONS***//

void blink(int num_blinks, int delay_blinks, int led_number ) {
  if (num_blinks < 0) num_blinks *= -1;

  switch (led_number) {
    case 1: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_blinks);
        }
        break;
      }
  }
}

//***HID JOYSTICK SPEED FUNCTIONS***//

void Joystick_Speed_Value(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  speed_counter = var;
}

void Increase_Joystick_Speed(void) {
  speed_counter++;

  if (speed_counter == 9) {
    blink(6, 50, 3);
    speed_counter = 8;
  } else {
    blink(speed_counter, 100, 1);
    joystick_delay = pow(1.6,(9-speed_counter))*fixed_delay;
    EEPROM.put(2, speed_counter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speed_counter));
}

void Decrease_Joystick_Speed(void) {
  speed_counter--;

  if (speed_counter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 0;
  } else if (speed_counter == 0) {
    blink(1, 350, 1);
    joystick_delay = pow(1.6,(9-speed_counter))*fixed_delay;
    EEPROM.put(2, speed_counter);
    delay(25);
  } else {
    blink(speed_counter, 100, 1);
    joystick_delay = pow(1.6,(9-speed_counter))*fixed_delay;
    EEPROM.put(2, speed_counter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speed_counter));
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void Pressure_Sensor_Initialization(void) {
  
  float nominal_joystick_value = (((float)analogRead(PRESSURE_JOYSTICK)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  sip_threshold = nominal_joystick_value + 0.5;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puff_threshold = nominal_joystick_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  
}


