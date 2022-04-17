/*  vacrouter-arduino.c - Based control of a relay driver linear actuator for a workshop shopvac

                          System consists of:
                          Arduino Mega 2560 running this code (persumably could be Uno)
                          An MQTT broker ('mosquitto' broker runs on my router via Entware)
                          A guest network containing smart power oulets running Tasmota 11 firmware
                          A Wyze Cam V2 running Openmiko firmware (has good wifi and USB port)
                            -'mosquitto' v2 installed, provides CLI subscribe and publish clients
                            -Runs vacrouter.sh & ardith.sh scripts
                              -vacrouter.sh subscribes to MQTT events & sends serial commands to the arduino
                                -Also publishes vacuum router status back to broker
                              -ardith.sh initializes the USB serial port, homes the machine & holds port open
                                -writes last line read to /tmp file for reading by vacrouter.sh
                              
 "In addition, some pins have specialized functions: External Interrupts: 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (interrupt 4), 20 (interrupt 3), and 21 (interrupt 2)." (think of these as D2, D3, D18, D19, D20, D21)

Revisions:      .2  March 13, 2022
                      Added detailed homing process
                      Added current position reporting

                .3  March 14, 2002
                      Added MOVE options GOCNC, GOSHOPSAW, GOWORKBENCH

TODO:
  Button handlers
  LED Lights
  Determine which messages are debug and which are permanent
  Update MOVE return messages and finalize
  Upload to github
  Finalize wiring diagram & add fusing, power for OpenMiko box (12V adapter or USB power bar?)
  Make small enclosure & backplate, make labels for button panels?
  Document use and overall system


"interrupts 0 and 1 are on digital pins 43 and 44." These refer to physical package pins 43 and 44, PortD bit 0 and 1. The IDE maps these to software names D21 and D20 which is what you see marked along the side of the board. It would seem there is also some interrupt re-naming.

// Button de-bounce examples found here: https://github.com/VRomanov89/EEEnthusiast/blob/master/03.%20Arduino%20Tutorials/01.%20Advanced%20Button%20Control/ButtonSketch/ButtonSketch.ino
*/
#include <Arduino.h>        // Base header required for basic Arduino functions
#include <string.h>
#include <stdlib.h>

// PINS
#define PIN_PROX_SENSOR   3  // 5V Inductive Sensor trigger line  # Use interupt pin on different bank?
#define PIN_MOTOR_FWD     4  // Move vacuum arm RIGHT (viewed from front) (retract actuator)
#define PIN_MOTOR_REV     5  // Move vacuum arm LEFT (extend actuator)
#define PIN_BUTTON_RED    6  // Red button moves arm left (nautical port)
#define PIN_BUTTON_GREEN  7  // Green button moves arm right (starboard)
#define PIN_LED_RED       21 // Solid for movement, flash for errors?
#define PIN_LED_GREEN     20 // Triggers solid for x secs with sensor

// COMMANDLINE.h defines
//this following macro is good for debugging, e.g.  print2("myVar= ", myVar);
#define print1(x)   (Serial.println(x))
#define print2(x,y) (Serial.print(x), Serial.println(y))
#define CR '\r'
#define LF '\n'
#define BS '\b'
#define NULLCHAR '\0'
#define SPACE ' '

// CLI CONFIG
#define COMMAND_BUFFER_LENGTH        25                        //length of serial buffer for incoming commands

// MOVE COMMANDS - Add case IDs here and also in the top of the MOVE logic in MOVEcommand() for RXCOMMAND
#define STOP      0
#define RIGHT     1
#define LEFT      2
//#define HOME      3
#define GL1       4
#define GR1       5
#define H1        6
#define H2        7
#define H3        8
#define H4        9
#define WORKBENCH 11
#define CHOPSAW   12
#define CNC       13

// STATUS TRACKING
// Sources - where a command originated
#define INIT      0
#define CLI       1
#define BUTTON    2
#define SENSOR    3

// TIMINGS
#define SENSOR_FALLOFF 300
#define SAFETY_CUTOFF ((2000) - (SENSOR_FALLOFF))
#define SENSOR_DEBOUNCE_DELAY 50
#define HOMING_TIMEOUT_LONG ((SAFETY_CUTOFF) + (500))
#define HOMING_TIMEOUT_SHORT ((HOMING_TIMEOUT_LONG) / (2))

// HOMING POSTIION DEFINES
// END_POS
#define A 1
#define B 2
#define C 3

// START POS
#define XA  0
#define AA  1
#define ABA 2
#define ABB 3
#define BB  4
#define BCB 5
#define BCC 6
#define CC  7
#define CX  8

// Homing sequences where N is null
#define RNNRNRL   0
#define NNRNRL    1
#define LNNRNRL   2
#define RNNLRNRL  3
#define LRNNLRNRL 4
#define NNLRNRL   5
#define LNNLRNRL  6
#define RNNLRNL   7
#define LRNNLRNL  8
#define NNLRNL    9
#define LNNLRNL   10

// LED Colours
#define OFF       0
#define RED       1
#define GREEN     2
#define YELLOW    3

// VARIABLES
// Sensor
int PREV_SENSOR_STATE = HIGH;
int SENSOR_STATE = HIGH;                        // Set initial state to high, since we pull low when triggered
int SENSOR_OVERRIDE = LOW;
// Homing
int HOME_STATE = 0;
int TRIGGER_COUNT = 0;
bool HOMING_ACTIVE = 0;
int HOMING = 0;
int HOME_DIRECTION = 0;
int HOMED_POS = 0;
int CURRENT_POS = -1;
int PREVIOUS_POS = -1;
String TRIGGER_ORDER = "";
String TRIGGER_ORDER2 = "";

// Misc
int SOURCE = 0;           // What authority, CLI, sensor etc is calling the function 

// LED color lookup table
typedef struct { // Structure to store the alarm code light indicator configuration
    int COLOR;   // Trigger order as determined by homing sequence
    int R;       // Derived start position, based on trigger order
    int G;       // Derived end position, based on trigger order
} LED_CFG;

// Accessed as LED_ARRAY[COLOR].value, ordered by definitions and positions from left to right
// Example: LED_ARRAY[RED].R would yield "1" and LED_ARRAY[RED].G would yield "0"
static LED_CFG LED_ARRAY[] = {
  { OFF,      1,   1 },   // Because we trigger low, these are all inverted 
  { RED,      0,   1 },      
  { GREEN,    1,   0 },      
  { YELLOW,   0,   0 },      
};

// Homing position lookup table
typedef struct { // Structure to store the alarm code light indicator configuration
    int TORDER;      // Trigger order as determined by homing sequence
    int START_POS;    // Derived start position, based on trigger order
    int END_POS;         // Derived end position, based on trigger order
} HOMED_CFG;

// Accessed as HOMED_ARRAY[0].value, ordered by definitions and positions from left to right
// example HOMED_ARRAY[3].END_POS would yield "A"
// KNOWN HOMING SEQUENCES
// NNLRNL
static HOMED_CFG HOMED_ARRAY[] = { 
  { RNNRNRL,    XA,   A },      
  { NNRNRL,     AA,   A },
  { LNNRNRL,    ABA,  A },
  { RNNLRNRL,   ABB,  B },
  { LRNNLRNRL,  ABB,  B },
  { NNLRNRL,    BB,   B },
  { LNNLRNRL,   BB,   B },
  { RNNLRNL,    BCC,  C },
  { LRNNLRNL,   BCC,  C },
  { NNLRNL,     CC,   C },
  { LNNLRNL,    CC,   C },
//  { LNNLRNL,    XC,   C },
};

static unsigned long state_start_timestamp = 0;     // In milliseconds, for the delay_ms function

char   CommandLine[COMMAND_BUFFER_LENGTH + 1];                 //Read commands into this buffer from Serial.  +1 in length for a termination char

const char *delimiters            = ", \n \r \r\n";                    //commands can be separated by return, space or comma

uint8_t RX_COMMAND = 99;
char firstCMDVariable[COMMAND_BUFFER_LENGTH + 1];

String rxWord;

/*************************************************************************************************************
     your Command Names Here
*/
const char *addCommandToken       = "add";                     //Modify here
const char *subtractCommandToken  = "sub";                     //Modify here
const char *MOVECommandToken      = "MOVE";                    //Modify here
const char *HOMECommandToken      = "HOME";                    //Modify here

// FUNCTIONS

// Non-blocking ms delay function, &start_timestamp is a pointer so multiple functions can be using this simultaneously
  // Return false if within request duration, true if duration ms has elapsed.
  // boolean delay_ms(unsigned long start_timestamp, unsigned long duration)
  // Usage: delay_ms(ms)  e.g. delay_ms(500)
  boolean delay_ms(unsigned long duration) {
  
    unsigned long new_current_timestamp;

    new_current_timestamp = millis();

      if (new_current_timestamp - state_start_timestamp >= duration) {
          state_start_timestamp = new_current_timestamp;
          return true;
      } 
      return false;
  }

// Physically sets the requested RGB light combination.
// Always sets all LEDs to avoid unintended light combinations

static void rgb_set_led (uint8_t reqColor) { 
    static uint8_t currColor = 99;
    if ( currColor != reqColor) {
        currColor = reqColor;
        digitalWrite(PIN_LED_RED, LED_ARRAY[reqColor].R);
        digitalWrite(PIN_LED_GREEN, LED_ARRAY[reqColor].G);
    }
}

void drag_lights() {
  rgb_set_led(RED);
  delay(333);
  rgb_set_led(YELLOW);
  delay(333);
  rgb_set_led(GREEN);
  delay(333);
  rgb_set_led(OFF);
  delay(333);
}


void motor_stop() {
        // If either motor pin is engaged, stop them both by setting them to HIGH since relay is LOW trigger
        if (!(digitalRead(PIN_MOTOR_FWD)) || !(digitalRead(PIN_MOTOR_REV))) {
           print2("MOTOR: STOP ISSUED BY SOURCE: ", SOURCE);
          digitalWrite(PIN_MOTOR_FWD, HIGH); 
          digitalWrite(PIN_MOTOR_REV, HIGH);
          if ( SENSOR_STATE != 0 ) {
            rgb_set_led(OFF);  // If we didn't trigger the sensor, turn off the lights, otherwise sensor will
          }
        } 
}

// Proximity sensor pulls LOW when triggered
void isr_prox_sensor() {
    int PREV_SOURCE = SOURCE;
    SOURCE = SENSOR;
    // Should really de-bounce but may not be necessary giving the call to stop motor
    if (digitalRead(PIN_PROX_SENSOR) != SENSOR_STATE) {
      delay(50);
      //print2("SENSOR: Debounced for (ms): ", SENSOR_DEBOUNCE_DELAY);
      //print2("SENSOR: OVERRIDE is ", SENSOR_OVERRIDE);
      SENSOR_STATE = digitalRead(PIN_PROX_SENSOR);
      if ((PREV_SENSOR_STATE != SENSOR_STATE) && (SENSOR_OVERRIDE == LOW)) {
        if (SENSOR_STATE == LOW) {
            //print2("SENSOR: !! TRIGGERED !! STATE: ", SENSOR_STATE);
            motor_stop();
            // Single pulse the green on detect
            if (( HOMING <= 0 ) || ( HOMING >= 5 )) {
              rgb_set_led(YELLOW);
            } else {
              rgb_set_led(GREEN);
            }

            if ((HOMING) && (HOME_DIRECTION == RIGHT)) {
              TRIGGER_ORDER2 = TRIGGER_ORDER + 'R';
              TRIGGER_ORDER = TRIGGER_ORDER2;
            }
            if ((HOMING) && (HOME_DIRECTION == LEFT)) {
              TRIGGER_ORDER2 = TRIGGER_ORDER + 'L';
              TRIGGER_ORDER = TRIGGER_ORDER2;
            }
        } else {
            // print2("SENSOR: Trigger released.  STATE: ", SENSOR_STATE);
        }
      }
    }

  PREV_SENSOR_STATE = SENSOR_STATE;
  SOURCE = PREV_SOURCE;
} 

void sensor_bypass() {
    SENSOR_OVERRIDE = 1;
    //print2("SENSOR_BYPASS: Disabled sensor interupt, current PIN state: ", (digitalRead(PIN_PROX_SENSOR)));
    delay(SENSOR_FALLOFF);
    SENSOR_OVERRIDE = 0;
    //print2("SENSOR_BYPASS: Enabled sensor interupt, SENSOR_OVERRIDE: ", SENSOR_OVERRIDE);      
}

void report_pos() {
  Serial.print("OK ");
  Serial.print("PPOS: ");
  Serial.print(PREVIOUS_POS);
  Serial.print(" CPOS: ");
  Serial.println(CURRENT_POS);
}

void motor_forward() { 
    // Check that we aren't already engaged
    if (digitalRead(PIN_MOTOR_REV) == LOW) {
        Serial.println("ERROR: motor_forward ignored, motor_reverse already engaged");
   } else {
        if ((CURRENT_POS < 3) || (HOMING_ACTIVE == 1) || (CURRENT_POS <= -1)) {
          rgb_set_led(RED);
          print2("MOTOR Forward: HOMING = ", HOMING);
          if ( (HOMING >= 1) && (HOMING < 5) ) {
            // If we're homing, use yellow instead of red
            rgb_set_led(YELLOW);
          }
          //Serial.println("MOTOR: FORWARD");
          digitalWrite(PIN_MOTOR_FWD, LOW);
        } else {
          Serial.print("ERROR: Requested travel would exceed range.  CPOS: ");
          Serial.println(CURRENT_POS);
        }
     }
}

void motor_reverse() {
    // Check that we aren't already engaged
    if (digitalRead(PIN_MOTOR_FWD) == LOW)  { 
      Serial.println("ERROR: motor_reverse ignored, motor_forward already engaged");
    } else {
        if (( CURRENT_POS > 1) || (HOMING_ACTIVE == 1) || (CURRENT_POS <= -1)) { 
          rgb_set_led(RED);
          print2("MOTOR REVERSE: HOMING = ", HOMING);
          if ( (HOMING >= 1) && (HOMING < 5) ) {
            // If we're homing, use yellow instead of red
            rgb_set_led(YELLOW);
          }
          //Serial.println("MOTOR: Reverse");
          digitalWrite(PIN_MOTOR_REV, LOW);
        } else {
          Serial.print("ERROR: Requested travel would exceed range.  CPOS: ");
          Serial.println(CURRENT_POS);
        }
    }
  }

  void move_right() {
    PREVIOUS_POS = CURRENT_POS;
    motor_forward();
    sensor_bypass();
    // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
    delay(SAFETY_CUTOFF);
    motor_stop();
    if (CURRENT_POS < 3) {
      CURRENT_POS = ((CURRENT_POS) + 1);
    }
    report_pos();
    if (CURRENT_POS == 4) {
      print2("motor_forward: ERROR we moved past position 3. CURRENT_POS = ", CURRENT_POS);
    } 
  } 

void move_gr1() {
  motor_forward();
  sensor_bypass();
  // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
  delay(SENSOR_FALLOFF);
  motor_stop();
}

void move_left() {
  PREVIOUS_POS = CURRENT_POS;
  motor_reverse();
  sensor_bypass();
  // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
  delay(SAFETY_CUTOFF);
  motor_stop();
  if ( CURRENT_POS > 1) {
    CURRENT_POS = ((CURRENT_POS) - 1);
  }
  report_pos();
  if (CURRENT_POS == 0) {
    print2("ERROR: (move_left) Moved back past position 1. CURRENT_POS = ", CURRENT_POS);
  } 
}

void move_gl1() { 
        motor_reverse();
        sensor_bypass();
        // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
        delay(SENSOR_FALLOFF);
        motor_stop();
}


// Move half the expected distance between points to try and locate a neighbor for reference
void homing_1 () {
  // print2("\t\t\tHOMING STAGE: ", HOMING);
  if ((HOMING == 1) && (SENSOR_STATE != 0)) {
    HOME_DIRECTION = RIGHT;
    motor_forward();
    // No sensor_bypass here because we didn't start on a sensor
    //sensor_bypass();
    // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
    delay_ms(HOMING_TIMEOUT_SHORT);
    motor_stop();
    //delay(250);
    // Return to start position and seek the same distance in opposite direction
    if (SENSOR_STATE != LOW) {
      HOME_DIRECTION = LEFT;
      motor_reverse();
      // sensor_bypass(); // Not needed, we aren't starting on a sensor
      delay(HOMING_TIMEOUT_SHORT * 2);  // Because we need to return and then seek in the other dir
      motor_stop();
      delay(250);
    
      // If we still haven't found a starting reference, go further and try again
      if (SENSOR_STATE != LOW) {
        HOME_DIRECTION = RIGHT;
        motor_forward();
        // No sensor_bypass here because we didn't start on a sensor
        //sensor_bypass();
        // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
        delay((HOMING_TIMEOUT_SHORT) * 3 );
        motor_stop();
        delay(250);
      }
    }

    if (SENSOR_STATE != LOW) {
      print2("HOMING_1: No stops detected in HOMING STAGE 1.  NEED A BETTER APPROACH.  SOURCE: ", SOURCE);
    } else {
      if (HOME_DIRECTION == RIGHT) {
        print2("HOMING_1: First stop detected RIGHT of start position, TRIGGER_ORDER: ", TRIGGER_ORDER);
      } else {
       print2("HOMING_1: First stop detected LEFT of start position, TRIGGER_ORDER: ", TRIGGER_ORDER);
      }
    }
  }
  TRIGGER_ORDER2 = TRIGGER_ORDER + 'N';
  TRIGGER_ORDER = TRIGGER_ORDER2;
  // print2("HOMING_1: TRIGGER_ORDER: ", TRIGGER_ORDER);
  HOMING = 2; 
  }

void homing_2 () {
  // print2("\t\t\tHOMING STAGE: ", HOMING);
  if ((HOMING == 2) && (SENSOR_STATE != 0)) {
    switch (HOME_DIRECTION) {
      // Seek in the opposite direction of the first detected point or last seek direction
      case RIGHT:                         
        HOME_DIRECTION = LEFT;
        motor_reverse();
        sensor_bypass();
        // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
         delay((HOMING_TIMEOUT_SHORT) * (2));
        motor_stop();
        //delay(250);
        break;

      case LEFT:
        HOME_DIRECTION = RIGHT;
        motor_forward();
        sensor_bypass();
        // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
        delay((HOMING_TIMEOUT_SHORT) * (2));
        motor_stop();
        //delay(250);
        break;

      default:
        print2("ERROR: (HOMING2) HOME_DIRECTION invalid, fell through to switch case default. HOME_DIRECTION: ", HOME_DIRECTION);
    }

    if (SENSOR_STATE != LOW) {
      print2("HOMING_2: No stops detected in HOMING STAGE 2.  Starting HOMING STAGE 3.  SOURCE: ", SOURCE);
    } else {
      if (HOME_DIRECTION == RIGHT) {
        // print2("HOMING_2: First stop detected RIGHT of start position: ", SOURCE);
      } else {
        // print2("HOMING_2: First stop detected LEFT of start position, SOURCE: ", SOURCE);
      }
    }
  } 
  TRIGGER_ORDER2 = TRIGGER_ORDER + 'N';
  TRIGGER_ORDER = TRIGGER_ORDER2;
  // print2("HOMING_2: TRIGGER_ORDER: ", TRIGGER_ORDER); 
  HOMING = 3;
}


void homing_3 () {
  // print2("\t\t\tHOMING STAGE: ", HOMING);
  if (HOMING == 3) {
    // Test left and see if we hit a stop point
    HOME_DIRECTION = LEFT;
    motor_reverse();
    sensor_bypass();
    // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
    delay(HOMING_TIMEOUT_LONG);
    motor_stop();
    //delay(250);
    if (SENSOR_STATE == LOW) {
      //print2("HOMING_3: Stopped to the LEFT of our first point. SOURCE: ", SOURCE);
      //print2("HOMING_3: Moving RIGHT to starting point.  SOURCE: ", SOURCE);
      HOME_DIRECTION = RIGHT;
      motor_forward();
      sensor_bypass();
      // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
      delay(HOMING_TIMEOUT_LONG);
      motor_stop();
      //delay(250);
      if (SENSOR_STATE == LOW) {
        //print2("HOMING_3: Sitting on START point. SOURCE: ", SOURCE);
       HOMING = 4;
      } else {
        //print2("HOMING 3: Moving RIGHT, DID NOT find the starting point.  SOURCE:", SOURCE);
      }
    } else {
      print2("HOMING_3: No points detected left of first point.  SOURCE: ", SOURCE);
      HOME_DIRECTION = RIGHT;
      motor_forward();
      sensor_bypass();
      // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
      delay(HOMING_TIMEOUT_LONG);
      motor_stop();
      delay(250);
      if (SENSOR_STATE == LOW) {
        print2("HOMING_3: Sitting on START point. SOURCE: ", SOURCE);
       HOMING = 4;
      } else {
        print2("HOMING 3: Moving RIGHT, DID NOT find the starting point.  SOURCE:", SOURCE);
      }
    }
  }
  TRIGGER_ORDER2 = TRIGGER_ORDER + 'N';   // Denote we are at H3 (need to do , for H1 and H2 it seems)
  TRIGGER_ORDER = TRIGGER_ORDER2;
  // print2("HOMING_3: TRIGGER_ORDER: ", TRIGGER_ORDER);
}

void homing_4 () {
  //print2("\t\t\tHOMING STAGE: ", HOMING);
  //print2("\t\t\tHOMING_TIMEOUT_LONG: ", HOMING_TIMEOUT_LONG);
  //print2("\t\t\tHOMING_TIMEOUT_SHORT: ", HOMING_TIMEOUT_SHORT); 
  if (HOMING == 4) {
    // Test RIGHT and see if we hit a stop point
    HOME_DIRECTION = RIGHT;
    motor_forward();
    sensor_bypass();
    // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
    delay(HOMING_TIMEOUT_LONG);
    motor_stop();
    //delay(250);     // Delay to let serial buffer catch up so we don't wind up in SENSOR OVERRIDE on final return

    if (SENSOR_STATE == LOW) {
      //print2("HOMING_4: Found a stop point RIGHT of our first point, SOURCE: ", SOURCE);
      //print2("HOMING_4: Moving LEFT to starting point. SOURCE: ", SOURCE);
      HOME_DIRECTION = LEFT;
      motor_reverse();
      sensor_bypass();
      // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
      delay(HOMING_TIMEOUT_LONG);
      motor_stop();
      //delay(250);
      if (SENSOR_STATE == LOW) {
        //print2("HOMING_4: Sitting on START point, to the LEFT of last detected point.  SOURCE:", SOURCE);
        HOMING = 5;
        /// Exits here if all is well, right edge case below
      }
    
    } else {
      //print2("HOMING4: moving RIGHT DID NOT find the starting point.  SOURCE:", SOURCE);
      //print2("HOMING4: Returning to last known point. SOURCE:", SOURCE);
      HOME_DIRECTION = LEFT;
      motor_reverse();
      sensor_bypass();
      // Blocking, Safety stop after x milliseconds in case sensor hasn't tripped
      delay(HOMING_TIMEOUT_LONG);
      motor_stop();      
      //delay(250);     // Delay to let serial buffer catch up so we don't wind up in SENSOR OVERRIDE on final return
      if (SENSOR_STATE == LOW) {
        print2("HOMING_4: Sitting on START point, to the LEFT of last detected point.  SOURCE:", SOURCE);
        HOMING = 5;
      } else {
        print2("HOMING_4: Something went really really wrong. SOURCE: ", SOURCE);
      }
    }
  }
  //print2("HOMING_4: TRIGGER_ORDER: ", TRIGGER_ORDER);
  // SEE DEFINES
  if (TRIGGER_ORDER == "RNNRNRL")   { HOMED_POS = 0;}
  if (TRIGGER_ORDER == "NNRNRL")    { HOMED_POS = 1;}
  if (TRIGGER_ORDER == "LNNRNRL")   { HOMED_POS = 2;}
  if (TRIGGER_ORDER == "RNNLRNRL")  { HOMED_POS = 3;}
  if (TRIGGER_ORDER == "LRNNLRNRL") { HOMED_POS = 4;}
  if (TRIGGER_ORDER == "NNLRNRL")   { HOMED_POS = 5;}
  if (TRIGGER_ORDER == "LNNLRNRL")  { HOMED_POS = 6;}
  if (TRIGGER_ORDER == "RNNLRNL")   { HOMED_POS = 7;}
  if (TRIGGER_ORDER == "LRNNLRNL")  { HOMED_POS = 8;}
  if (TRIGGER_ORDER == "NNLRNL")    { HOMED_POS = 9;}
  if (TRIGGER_ORDER == "LNNLRNL")   { HOMED_POS = 10;}

  // print2("Starting position: ",(HOMED_ARRAY[HOMED_POS].START_POS));  // See defines for starting positions, they are not outlets
  // print2("Vacuum is estimated to be in L to R outlet: ", HOMED_ARRAY[HOMED_POS].END_POS);
  CURRENT_POS = HOMED_ARRAY[HOMED_POS].END_POS;

  if ( HOMING > 4 ) {
    if (CURRENT_POS != 2 ) {
      print2("Calibration complete, moving to default/start position (2). SOURCE: ", SOURCE);
      if ( CURRENT_POS == 1) { 
        move_right();
      } else {
        move_left();
      }
    } else { // If we are on the middle position already, report the POS
      report_pos();
    } 
  }
  TRIGGER_ORDER = TRIGGER_ORDER2 = "";  // Clear variables for re-use
  drag_lights();  
}

  /*****************************************************************************

  How to Use CommandLine:
    Create a sketch.  Look below for a sample setup and main loop code and copy and paste it in into the new sketch.

   Create a new tab.  (Use the drop down menu (little triangle) on the far right of the Arduino Editor.
   Name the tab CommandLine.h
   Paste this file into it.

  Test:
     Download the sketch you just created to your Arduino as usual and open the Serial Window.  Typey these commands followed by return:
      add 5, 10
      subtract 10, 5

    Look at the add and subtract commands included and then write your own!


*****************************************************************************
  Here's what's going on under the covers
*****************************************************************************
  Simple and Clear Command Line Interpreter

     This file will allow you to type commands into the Serial Window like,
        add 23,599
        blink 5
        playSong Yesterday

     to your sketch running on the Arduino and execute them.

     Implementation note:  This will use C strings as opposed to String Objects based on the assumption that if you need a commandLine interpreter,
     you are probably short on space too and the String object tends to be space inefficient.

   1)  Simple Protocol
         Commands are words and numbers either space or comma spearated
         The first word is the command, each additional word is an argument
         "\n" terminates each command

   2)  Using the C library routine strtok:
       A command is a word separated by spaces or commas.  A word separated by certain characters (like space or comma) is called a token.
       To get tokens one by one, I use the C lib routing strtok (part of C stdlib.h see below how to include it).
           It's part of C language library <string.h> which you can look up online.  Basically you:
              1) pass it a string (and the delimeters you use, i.e. space and comman) and it will return the first token from the string
              2) on subsequent calls, pass it NULL (instead of the string ptr) and it will continue where it left off with the initial string.
        I've written a couple of basic helper routines:
            readNumber: uses strtok and atoi (atoi: ascii to int, again part of C stdlib.h) to return an integer.
              Note that atoi returns an int and if you are using 1 byte ints like uint8_t you'll have to get the lowByte().
            readWord: returns a ptr to a text word

   4)  DoMyCommand: A list of if-then-elses for each command.  You could make this a case statement if all commands were a single char.
      Using a word is more readable.
          For the purposes of this example we have:
              Add
              Subtract
              nullCommand
*/
/******************sample main loop code ************************************

  #include "CommandLine.h"

  void
  setup() {
  Serial.begin(115200);
  }

  void
  loop() {
  bool received = getCommandLineFromSerialPort(CommandLine);      //global CommandLine is defined in CommandLine.h
  if (received) DoMyCommand(CommandLine);
  }

**********************************************************************************/


// COMMAND LINE SERIAL FUNCTIONS
/*************************************************************************************************************
    getCommandLineFromSerialPort()
      Return the string of the next command. Commands are delimited by return"
      Handle BackSpace character
      Make all chars lowercase
*************************************************************************************************************/

  bool
  getCommandLineFromSerialPort(char * commandLine) {
    static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
    //read asynchronously until full command input
    while (Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
        case LF:
          commandLine[charsRead] = NULLCHAR;       //null terminate our command char array
          if (charsRead > 0)  {
            charsRead = 0;                           //charsRead is static, so have to reset
            Serial.println(commandLine);
            return true;
          }
          break;
        case BS:                                    // handle backspace in input: put a space in last char
          if (charsRead > 0) {                        //and adjust commandLine and charsRead
            commandLine[--charsRead] = NULLCHAR;
            Serial << byte(BS) << byte(SPACE) << byte(BS);  //no idea how this works, found it on the Internet
          }
          break;
        default:
          // c = tolower(c);
          if (charsRead < COMMAND_BUFFER_LENGTH) {
            commandLine[charsRead++] = c;
          }
          commandLine[charsRead] = NULLCHAR;     //just in case
          break;
      }
    }
    return false;
  }


  /* ****************************
    readNumber: return a 16bit (for Arduino Uno) signed integer from the command line
    readWord: get a text word from the command line

  */
  int
  readNumber () {
    char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
    return atoi(numTextPtr);                              //K&R string.h  pg. 251
  }

  char * readWord() {
    char * word = strtok(NULL, delimiters);               //K&R string.h  pg. 250
    return word;
  }

  void
  nullCommand(char * ptrToCommandName) {
    print2("Command not found: ", ptrToCommandName);      //see above for macro print2
  }


  /****************************************************
     Add your commands here
  */

  int addCommand() {                                      //Modify here
    int firstOperand = readNumber();
    int secondOperand = readNumber();
    return firstOperand + secondOperand;
  }

  int subtractCommand() {                                //Modify here
    int firstOperand = readNumber();
    int secondOperand = readNumber();
    return firstOperand - secondOperand;
  }

  int HOMEcommand() {
    HOMING_ACTIVE = 1;
    HOMING = 1;
    homing_1();
    homing_2();
    homing_3();
    homing_4();
    HOMING_ACTIVE = 0;
    return 0;
  }

  int MOVEcommand() {
    SOURCE = CLI;
    RX_COMMAND = 99;      // Reset RX_COMMAND
    rxWord = readWord();  // Store the content returned by readWord() in a string

    // Switch statements only work with integers, so convert to integers
    if (rxWord == "STOP") {
      RX_COMMAND = STOP;
    }
    if (rxWord == "RIGHT") {
      RX_COMMAND = RIGHT;
    }
    if (rxWord == "LEFT") {
      RX_COMMAND = LEFT;
    }
    if (rxWord == "GOCNC") {
      RX_COMMAND = CNC;
    }
    if (rxWord == "GOCHOPSAW") {
      RX_COMMAND = CHOPSAW;
    }
    if (rxWord == "GOWORKBENCH") {
      RX_COMMAND = WORKBENCH;
    }
       if (rxWord == "GL1") {
      RX_COMMAND = GL1;
    }
    if (rxWord == "GR1") {
      RX_COMMAND = GR1;
    }
    if (rxWord == "H1") {
      RX_COMMAND = H1;
    }
    if (rxWord == "H2") {
      RX_COMMAND = H2;
    }
    if (rxWord == "H3") {
      RX_COMMAND = H3;
    }
    if (rxWord == "H4") {
      RX_COMMAND = H4;
    }
  
    switch (RX_COMMAND) {
      case STOP:
        motor_stop();
        return 0;
        break;

     case RIGHT:
        move_right();
        return 0;
        break;

      case GR1: 
        move_gr1();
        return 0;
        break;

      case LEFT: 
        move_left();
        return 0;
        break;

      case GL1: 
        move_gl1();
        return 0;
        break;

      case CNC: 
        if (CURRENT_POS == -1) {
          print2("ERROR: (MOVEcommand CNC) Machine not homed. SOURCE: ", SOURCE);
        } else {
            if (CURRENT_POS == 1) {
              move_right();
              move_right();
            }
            if (CURRENT_POS == 2) {
              move_right();
            }
            if (CURRENT_POS == 3) {
              // We're already where we need to be
            }        
          }
        return 0;
        break;
 
      case CHOPSAW:
        if (CURRENT_POS == -1) {
          print2("ERROR: (MOVEcommand CHOPSAW) Machine not homed. SOURCE: ", SOURCE);
        } else {
            if (CURRENT_POS == 1) {
              move_right();
            }
            if (CURRENT_POS == 2) {
              // We're already where we need to be
            }
            if (CURRENT_POS == 3) {
              move_left();
            }
          }
        return 0;
        break;
 
      case WORKBENCH:
        if (CURRENT_POS == -1) {
          print2("ERROR: (MOVEcommand WORKBENCH) Machine not homed. SOURCE: ", SOURCE);
        } else {
          if (CURRENT_POS == 1) {
            // We're already where we need to be
          }
          if (CURRENT_POS == 2) {
            move_left();
          }
          if (CURRENT_POS == 3) {
            move_left();
            move_left();
          }
        }
        return 0;
        break;
 
      case H1:

        if (digitalRead(PIN_PROX_SENSOR) == LOW) {
          HOMING = 3;
        } else {
          HOMING = 1;
        }
          homing_1();
          return 0;
          break;

      case H2:
          homing_2();
          return 0;
          break;

      case H3:
          homing_3();
          return 0;
          break;

      case H4:
          homing_4();
          return 0;
          break;

      default: 
        Serial.println("ERROR: (MOVECommand) Invalid command, fell through switch case");
        return 1;
        break;
    }
  }    
       

  /****************************************************
     DoMyCommand
  */
  bool
  DoMyCommand(char * commandLine) {
    //  print2("\nCommand: ", commandLine);
    int result;

    char * ptrToCommandName = strtok(commandLine, delimiters);
    //  print2("commandName= ", ptrToCommandName);

    if (strcmp(ptrToCommandName, addCommandToken) == 0) {                   //Modify here
      result = addCommand();
      print2(">    The sum is = ", result);

    } else {
      if (strcmp(ptrToCommandName, subtractCommandToken) == 0) {           //Modify here
        result = subtractCommand();                                       //K&R string.h  pg. 251
        print2(">    The difference is = ", result);

      } else {
        if (strcmp(ptrToCommandName, MOVECommandToken) == 0) {
          result = MOVEcommand();
          if (result != 0) {
            print2("ERROR: (DoMyCommand) Return result of MOVE command goes here, ", result);
          }   

        } else {
          if (strcmp(ptrToCommandName, HOMECommandToken) == 0) {
            result = HOMEcommand();
            if (result != 0) {
              print2("ERROR: (DoMyCommand) Return result of HOME command goes here, ", result);
            }

          } else {
              nullCommand(ptrToCommandName);
            }
        }
      }
    }
  return 0;
  }
  

// SETUP
  void setup() {
  Serial.begin(115200);
  Serial.println("Vacrouter Arduino Mega 2560 Interface - v.1");
  pinMode(PIN_MOTOR_FWD, OUTPUT);
  digitalWrite(PIN_MOTOR_FWD, HIGH);
  pinMode(PIN_MOTOR_REV, OUTPUT);
  digitalWrite(PIN_MOTOR_REV, HIGH);
  pinMode(PIN_PROX_SENSOR, INPUT_PULLUP);
  pinMode(PIN_BUTTON_RED, INPUT_PULLUP);
  pinMode(PIN_BUTTON_GREEN, INPUT_PULLUP);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_GREEN, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_PROX_SENSOR), isr_prox_sensor, CHANGE) ;
  // Do a command to print the timing defines
  // Serial.println("CONFIG VARIABLES:");
  //print2("SAFETY_CUTOFF: ", SAFETY_CUTOFF);
  //print2("SENSOR_FALLOFF: ", SENSOR_FALLOFF);
  //print2("HOMING_TIMEOUT_SHORT: ", HOMING_TIMEOUT_SHORT);
  //print2("HOMING_TIMEOUT_LONG: ", HOMING_TIMEOUT_LONG);
  print2("PIN_MOTOR_FWD (for left movement) is: ", PIN_MOTOR_FWD);
  print2("PIN_MOTOR_REV (for right movement) is: ", PIN_MOTOR_REV);
  SENSOR_STATE = (digitalRead(PIN_PROX_SENSOR));
  if (SENSOR_STATE == LOW) {
    print2("SENSOR: TRIGGERED (LOW) on PIN: ", PIN_PROX_SENSOR);
  } else {
    print2("SENSOR: NOT TRIGGERED (HIGH) on PIN: ", PIN_PROX_SENSOR); 
  }
}

// MAIN LOOP
  bool bootlight;
  void loop() {
    if ( HOMING == 0 ) {
      //print2("In the main loop RGB boot section.  bootlight: ", bootlight);
      
      if ( bootlight == 0 ) {
        rgb_set_led(OFF);
      } 
      
      if ( bootlight == 1 ) {
        rgb_set_led(GREEN);
        }

      if (delay_ms(3000)) {
        if ( bootlight == 0) {
          bootlight = true;
        } else {
          bootlight = false;
        }      
      }
    }
    if (( HOMING == 5 ) && (digitalRead(PIN_MOTOR_FWD == 1)) && (digitalRead(PIN_MOTOR_REV == 1)) ) {
      rgb_set_led(GREEN);
    } 
    bool received = getCommandLineFromSerialPort(CommandLine);      //global CommandLine is defined in CommandLine.h
    if (received) DoMyCommand(CommandLine);
  }
