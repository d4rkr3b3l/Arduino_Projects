/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Initial version.  Uses Mysensors 2.0
 * 
 * DESCRIPTION
 * This sketch implements automated dog water bowl.
 * Once bowl is empty, fill bowl by a specified amount, and prevent 
 * refill for a specified amount of time.
 */

 // todo:
 // - system init without gateway.  needs to run standalone.

/* ------------------------------------ */
/* ----  ENTER TIME VARIABLES BELOW --- */
/* ------------------------------------ */
#define FILL_TIME 180         // Time in Sec: 180 second fill => ~ 200 mL  (test param: 10)
#define mL_PER_FILL 200       // amount of water per fill event
#define FILL_TIMEOUT 900      // Time in Sec: 900 seconds = 15 minutes  (test param: 120)
#define TIME_RESYNC 600      // Time in Sec: 3600 seconds = 1 hr  (test param: 600)
#define SENSOR_DELAY 30       // Time in Sec: 30 seconds between sensor checks  (test param: 5)
#define SCREEN_REFRESH_DELAY 5// Time in 10th of sec: 0.5 second screen refresh rate  (test param: 1) 
#define SENSOR_THRESHOLD 750  // 2.4v threshold for ADC sensor (0 to 1024 => 0 to 3.3)... 1024/3.3 = 310.3 * 2.4 = 750
/* ------------------------------------ */
 
// Enable debug prints to serial monitor
//#define MY_DEBUG 
#define DEBUG_ON   // comment out to supress serial monitor output


#ifdef DEBUG_ON
#define DEBUG_PRINT(x)   Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define SERIAL_START(x)
#endif

// Enable and select radio type attached
#define MY_RADIO_NRF24

// My Sensors settings
#define MY_NODE_ID 1
#define MY_RF24_PA_LEVEL RF24_PA_MAX    // min, low, high, max (default is max)
#define MY_REGISTRATION_FEATURE 
#define CHILD_ID_BTN 1
#define CHILD_ID_STATS 2


#define MOTOR_DIGITAL_PIN 3   // Digital Pin for Motor
#define BUTTON_DIGITAL_PIN 4  // Digital Pin for Button
#define LED_DIGITAL_PIN 5     // Digital Pin for LED
#define SENSE0_ANALOG_PIN 0   // Analog Sensor Pin 0.
#define SENSE1_ANALOG_PIN 1   // Analog Sensor Pin 1.
#define SENSE2_ANALOG_PIN 2   // Analog Sensor Pin 2.
#define MOTOR_ON 1            // Value to use to turn Motor On
#define MOTOR_OFF 0           // Value to use to turn Motor Off

#include <SPI.h>
#include <MySensors.h>  
#include <Bounce2.h>
#include <TimeLib.h>
#include <TimerOne.h>
#include <Wire.h>
#include <GOFi2cOLED.h>

unsigned int volatile TimR1 = 0;  // used for fill timer
unsigned int volatile TimR2 = 0;  // used for fill time out
unsigned int volatile TimR3 = 0;  // 1 hr timer

GOFi2cOLED  GOFoled;

Bounce debouncer = Bounce(); 
int oldValue=1;
bool state;

bool showTime = true;
bool clockUpdating = false;
int oldDay = 0;

bool hBeat = true;

int fillcount = 0;          // Used to track how many times the bowl is filled in 1 day
int emptycount = 0;         // Used to track how many times sensor check has come back empty (need 2 sensor reads in a row to trigger a fill)
int secCount = 0;
int halfSecCount = 0;

int sensorA0 = 0;           // ADC value for A0
int sensorA1 = 0;           // ADC value for A1
int sensorA2 = 0;           // ADC value for A2

const char *waterlevel[] = {                  // define the enumeration for water level descriptions
  "Empty", "Low", "Medium", "Full", "ERROR"
};

char printBuf[24] ;                           // temporary output buffer for V_Text updates

MyMessage msgBtn(CHILD_ID_BTN,V_LIGHT);
MyMessage msgStats(CHILD_ID_STATS,V_TEXT);

void setup()  
{ 
  DEBUG_PRINTLN(F("... Starting Setup ...")); 
  
  // default slave address is 0x3D
  GOFoled.init(0x3C);  //initialze  OLED display
  GOFoled.display(); // show splashscreen
  wait(500);
  GOFoled.clearDisplay();             // clear the display
  GOFoled.setTextSize(1);
  GOFoled.setTextColor(WHITE);
  GOFoled.setCursor(0,0);
  GOFoled.println(F(" ... Initializing ..."));
  GOFoled.display();
  wait(500);
  GOFoled.println(F("- Setting up I/O"));
  GOFoled.display();
  wait(250);
    
  // Setup the button
  pinMode(BUTTON_DIGITAL_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_DIGITAL_PIN,HIGH);

  // Make sure led output is off
  digitalWrite (LED_DIGITAL_PIN, LOW);
  // Setup the LED
  pinMode(LED_DIGITAL_PIN, OUTPUT);

  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_DIGITAL_PIN);
  debouncer.interval(5);

  // Make sure motor is off when starting up
  digitalWrite(MOTOR_DIGITAL_PIN, MOTOR_OFF);
  // Then set relay pins in output mode
  pinMode(MOTOR_DIGITAL_PIN, OUTPUT);  

  // Set state to off
  state = false;

  GOFoled.println(F("- Setting up timers"));
  GOFoled.display();
  wait(250);

  // Init Timer 1 ISR
  Timer1.initialize (100000); // init timer to run every 100,000 uSec => 0.1 Sec
  Timer1.attachInterrupt (TimerISR);  // run TimerISR function every time interrupt occurs

  GOFoled.println(F("- Time SYNC"));
  GOFoled.display();
  wait(250);
  
  // Get Current Time
  TimR1 = 100;  // Allow 10 seconds to get current time
  while (timeStatus() == timeNotSet && TimR1 > 0)
  {
    requestTime();
    DEBUG_PRINTLN(F("Requesting time from Gateway:"));
    wait(1000);
    if (TimR1 < 50)  // 5 seconds have elapsed (100-50=50)
    {
      DEBUG_PRINTLN(F("Failed initial clock synchronization!"));
      GOFoled.clearDisplay();             // clear the display
      GOFoled.setCursor(0,0);
      GOFoled.println(F("Time Sync Failed"));
      GOFoled.display();
      wait(2000);
      GOFoled.clearDisplay();             // clear the display
      GOFoled.setCursor(0,0);
      GOFoled.println(F(" ... Initializing ..."));
      GOFoled.println(F("- Time SYNC"));
      GOFoled.display();
      break;
    }
  }
  oldDay = day();
  TimR1 = 0;  // Clear timer
  TimR2 = 0;
  TimR3 = SCREEN_REFRESH_DELAY;  // start time resync timer
  
  DEBUG_PRINTLN(F("Initialization Complete!"));
}

void presentation()  {
  GOFoled.println(F("- Presenting Sensors"));
  GOFoled.display();
  wait(250);
  
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Dog Water Bowl", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_BTN, S_LIGHT, "Motor Switch");
  present(CHILD_ID_STATS, S_INFO, "Stats"); 

  GOFoled.setTextColor(BLACK, WHITE); // 'inverted' text
  GOFoled.println(F("... INIT COMPLETE ..."));
  GOFoled.display();
  GOFoled.clearDisplay();             // clear the display
  
  wait(1000);
  DEBUG_PRINTLN(F("...Starting main loop..."));
}

void loop()      
{  
  debouncer.update();
  // Get the update value
  int value = debouncer.read();
  if (value != oldValue && value==1) {
      DEBUG_DispTime();
      DEBUG_PRINT(F("Button Pressed. State was: "));
      DEBUG_PRINT(state);
      state = state?false:true;       // toggle state
      DEBUG_PRINT(F("  State is: "));
      DEBUG_PRINTLN(state);
      send(msgBtn.set(state), true);  // send state and request ack back
      if (state) StartMotor();
      else {                          // if button is pressed to stop a fill, start fill time out  
        DEBUG_PRINTLN(F("Stopping Motor."));
        TimR2 = FILL_TIMEOUT *10;  
      }
  }
  oldValue = value;

  // state machine states (stand by, check sensors, run motor, wait for fill, fill timeout)

  // state = true ==> motor run
  if (state) {
    // stop motor if volume reached
    if (TimR1 == 0) { //fill timer reached
      state = false;
      DEBUG_DispTime();
      DEBUG_PRINTLN(F("Fill timer reached.  Stopping motor."));
      send(msgBtn.set(state), true);  // send state and request ack back
      TimR2 = FILL_TIMEOUT *10;  // start fill time out
    }
    else {  // fill timer not reached, check sensor to check for overflow
      CheckSensors();      
    }
  }
  
  // state = false ==> Motor off
  else {
    TimR1 = 0;    // if motor off, no need for motor timer
    // has fill timeout elapsed
    if (TimR2 == 0){
      // reset fill timeout timer to 15 seconds 
      TimR2 = SENSOR_DELAY * 10;
      
      DEBUG_DispTime();
      DEBUG_PRINTLN(F("Checking periodic sensor data."));
           
      // check sensors every 15 seconds.  if low change state to true. 
      CheckSensors();
    }
  }

  // Set motor output based on state
  digitalWrite(MOTOR_DIGITAL_PIN, state?MOTOR_ON:MOTOR_OFF);    
  // Turn on LED if motor is running
  digitalWrite(LED_DIGITAL_PIN, state?HIGH:LOW);

  // Tasks to be run every second
  if (TimR3 == 0) {
    //reset half sec timer
    TimR3 = SCREEN_REFRESH_DELAY;
    halfSecCount ++;
    UpdateDisplay();  // update oled display every half sec
    
    if (halfSecCount >= 2) {    // every qty 2 half seconds (ie. every second)
      halfSecCount = 0;
      secCount ++;    // increment second counter
      
      if (secCount % 5 == 0){   // perform this every 5 seconds
        sprintf(printBuf, "Bowl filled %i time(s)", fillcount);
        send(msgStats.set(printBuf), false);  // send state and request ack back
      }
      
      if (secCount >= TIME_RESYNC) {     // Resync time every resync period (every hour)
        updateClock();
        
        // Daily tasks (reset daily counters)  
        // Only run once if day changes
        if (oldDay != day()) {
          DEBUG_PRINTLN(F("...Performing start of day tasks..."));
          DEBUG_PRINT(F("Daily Stats:  Bowl filled "));
          DEBUG_PRINT(fillcount);
          DEBUG_PRINTLN(F(" time(s) today."));
    
          DEBUG_PRINTLN(F("\nResetting Stats."));
          fillcount = 0;
        }
        
        oldDay = day();
        
        //reset hour timer
        secCount = 0;  
      }
    }
  }

}

void UpdateDisplay() {
  // display states
  // 1 - Stand by (checking sensors, display number of fills, display amount of water)
  // 2 - filling (fill time, display number of fills, display amount of water)
  // 3 - 

  hBeat = hBeat?false:true;           // invert hBeat indicator status
  GOFoled.clearDisplay();             // clear the display  

  if (hBeat) GOFoled.drawCircle(GOFoled.width()-6, 4, 4, WHITE);
  else GOFoled.fillCircle(GOFoled.width()-6, 4, 4, WHITE);

  GOFoled.setTextSize(1);
  GOFoled.setTextColor(WHITE);
  GOFoled.setCursor(0,0);
  

  if ((!state) && (TimR1 == 0)) {   // System Standby
    GOFoled.println(F("State: Standby"));  
    GOFoled.println();
    GOFoled.print(int(TimR2/10));
    GOFoled.println(F(" sec to next chk"));
  }
  else if (state){
    int currfilltime = FILL_TIME - (TimR1/10);
    GOFoled.println(F("State: FILLING"));  
    GOFoled.println();
    GOFoled.print(currfilltime);
    GOFoled.print(F(" sec / "));
    GOFoled.print(FILL_TIME);
    GOFoled.println(F(" sec"));
  }

  // display consistant message
  // line 5 is bowl fill count
  // line 7 is amount filled
  GOFoled.setCursor(0, (8 * 5));   // each char is 7 pixels high with 1 pixel separation.  
  GOFoled.print(F("Bowl filled "));
  GOFoled.print(fillcount);
  GOFoled.print(F(" time"));
  GOFoled.println((fillcount == 1) ? F("") : F("s"));
  GOFoled.println();

  GOFoled.print(F("Vol of water: "));
  GOFoled.print(int(fillcount * mL_PER_FILL));
  GOFoled.print(F(" mL"));
  
  GOFoled.display();
}

void CheckSensors() {
  int fillLevel = 0;

  
  //read A2D A0, A1, and A2  (Empty, Low, Medium, Full)
  
  pinMode(SENSE0_ANALOG_PIN, INPUT);          // Change A0 to input
  sensorA0 = analogRead(SENSE0_ANALOG_PIN);   // Measure A0
  pinMode(SENSE0_ANALOG_PIN, OUTPUT);         // Set A0 to a digital output
  digitalWrite (SENSE0_ANALOG_PIN, LOW);      // Set A0 to digital low to keep voltage out of bowl

  pinMode(SENSE1_ANALOG_PIN, INPUT);
  sensorA1 = analogRead(SENSE1_ANALOG_PIN);
  pinMode(SENSE1_ANALOG_PIN, OUTPUT);
  digitalWrite (SENSE1_ANALOG_PIN, LOW);

  pinMode(SENSE2_ANALOG_PIN, INPUT);
  sensorA2 = analogRead(SENSE2_ANALOG_PIN);
  pinMode(SENSE2_ANALOG_PIN, OUTPUT);
  digitalWrite (SENSE2_ANALOG_PIN, LOW);

  // if water level is empty, change state to true and startmotor().
  if (sensorA0 < SENSOR_THRESHOLD) fillLevel++;
  if (sensorA1 < SENSOR_THRESHOLD) fillLevel++;
  if (sensorA2 < SENSOR_THRESHOLD) fillLevel++;

  // detect sensor error. if A0 is not submerged, but A1 or A2 are submerged, there is a sensor error
  if ((sensorA0 > SENSOR_THRESHOLD) && (fillLevel > 0))
    fillLevel = 4;

  //print out bowl level if doing a periodic level check, not during a fill check
  if (!state){
    DEBUG_PRINTLN(F("Sensor values:"));
    DEBUG_PRINT(F("A0 = "));
    DEBUG_PRINTLN(sensorA0);
    DEBUG_PRINT(F("A1 = "));
    DEBUG_PRINTLN(sensorA1);
    DEBUG_PRINT(F("A2 = "));
    DEBUG_PRINTLN(sensorA2);
    DEBUG_PRINT(F("Bowl water level: "));
    DEBUG_PRINTLN(waterlevel[fillLevel]);  
  }
  

  // if bowl is empty, set state to true to turn on motor 
  // if water level is Full, set state to false and turn off motor
  if ((fillLevel == 0) && (!state)) {
    emptycount ++;
    if (emptycount >= 2) {  //only turn on motor if sensors have detected empty twice in a row
      state = true;
      emptycount = 0;  // reset empty counter on fill
      send(msgBtn.set(state), true);  // send state and request ack back
      StartMotor();
    }
  }
  else if ((fillLevel > 2) && (state)) {  // if during the course of a fill, the sensors return a value of full, turn off motor
    DEBUG_PRINTLN(F("Bowl Full... Turning off motor!"));
    state = false;
    TimR2 = FILL_TIMEOUT *10;       // set fill timeout
    send(msgBtn.set(state), true);  // send state and request ack back
  }

  if ((!state) && (fillLevel > 0)) emptycount = 0;  // if a non-empty reading is taken, reset empty counter
  
}

void StartMotor() {
  // Clear timeout timer if button was pressed to turn motor on
  DEBUG_DispTime();
  DEBUG_PRINTLN(F("Starting Motor"));
  TimR1 = FILL_TIME * 10;  // set motor run timer to 60 seconds
  TimR2 = 0;  // reset fill time out if state turned true
  fillcount ++; // increment the fill counter  
  
  DEBUG_PRINT(F("Bowl filled "));
  DEBUG_PRINT(fillcount);
  DEBUG_PRINTLN(F(" time(s) today."));
}

void receive(const MyMessage &message) {
  bool oldstate = state;
  
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck()) {
     DEBUG_PRINTLN(F("This is an ack from gateway"));
  }

  if (message.type == V_LIGHT) {
     // Change relay state
     state = message.getBool();
      // Write some debug info
     DEBUG_DispTime();
     DEBUG_PRINT(F("Incoming change for sensor:"));
     DEBUG_PRINT(message.sensor);
     DEBUG_PRINT(F(", New status: "));
     DEBUG_PRINTLN(message.getBool());
     
     if (state && !oldstate) StartMotor();  // start motor only if it is currently off
     else if(!state && oldstate) {          // if button is pressed to stop a fill, start fill time out  
      DEBUG_PRINTLN(F("Stopping Motor."));
      TimR2 = FILL_TIMEOUT *10;  
     }
     // do nothing if the state hasnt changed
   } 
}

void updateClock()
{
  DEBUG_PRINTLN(F("Requesting time from Gateway..."));
  clockUpdating = true;
  requestTime();
}

void receiveTime(time_t newTime)
{
  DEBUG_PRINTLN(F("Time value received and updated..."));
  int lastSecond = second();
  int lastMinute = minute();
  int lastHour = hour();
  setTime(newTime);
  if (((second() != lastSecond) || (minute() != lastMinute) || (hour() != lastHour)) || showTime)
  {
    DEBUG_PRINTLN(F("Clock updated...."));
    DEBUG_PRINT(F("Sensor's time currently set to:"));
    DEBUG_PRINT(hourFormat12() < 10 ? F(" 0") : F(" "));
    DEBUG_PRINT(hourFormat12());
    DEBUG_PRINT(minute() < 10 ? F(":0") : F(":"));
    DEBUG_PRINT(minute());
    DEBUG_PRINT(second() < 10 ? F(":0") : F(":"));
    DEBUG_PRINT(second());
    DEBUG_PRINTLN(isAM() ? F("am") : F("pm"));
    DEBUG_PRINT(month());
    DEBUG_PRINT(F("/"));
    DEBUG_PRINT(day());
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(year());
    showTime = false;
  }
  else
  {
    DEBUG_PRINTLN(F("System's time did NOT need adjustment greater than 1 second."));
  }
  clockUpdating = false;
}

void DEBUG_DispTime() {
  DEBUG_PRINT(hourFormat12() < 10 ? F("0") : F(""));
  DEBUG_PRINT(hourFormat12());
  DEBUG_PRINT(minute() < 10 ? F(":0") : F(":"));
  DEBUG_PRINT(minute());
  DEBUG_PRINT(second() < 10 ? F(":0") : F(":"));
  DEBUG_PRINT(second());
  DEBUG_PRINT(isAM() ? F("am: ") : F("pm: "));
}

// Interrupt driven count down timer function.
// Will only decrement the independant timers if non 0 each time interrupt function is called
// 864,000 0.1 seconds in 24 hrs
void TimerISR () {
  if (TimR1)
    TimR1 --;
  if (TimR2)
    TimR2 --;
  if (TimR3)
    TimR3 --;
}

