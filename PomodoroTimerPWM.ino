/************************************************************************************************************************************
Smart Timer. Alberto Naranjo Galet 2013

Basic Operation:
1. Fade In all the leds as start-up
2. At first click, dial will fill in and start the countdown.
3. One click during countdown and it will pause. The last mark will pulse to show the pause.
4. Long click any time will reset all counts (red leds) and states.
5. After countdown is finished. One cycle count will be added. It will pulse and the alarm sounds.
6. One click the alarm stops, and rest time will countdown.
7. While rest time, the cycle count will pulse. And the dial will fill. After rest time it will idle waiting new cycle starts with click.
8. Double click during rest time, will cancel it.

************************************************************************************************************************************/

//BASIC FOR LATCHING FROM... I dont remember...
// You can choose the latch pin yourself.
#define SHIFTPWM_NOSPI
const int ShiftPWM_dataPin = 8; //pin 14 on the 75HC595
const int ShiftPWM_latchPin= 9; //pin 12 on the 75HC595
const int ShiftPWM_clockPin = 10; //pin 11 on the 75HC595
// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = false; 
// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;
#include "ShiftPWM.h"   // include ShiftPWM.h after setting the pins!
// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
// There is a calculator on my website to estimate the load.
unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 75;
int numRegisters = 2;
//int numRGBleds = numRegisters*8/3;


//Power saving Mode
#include <avr/power.h>
#include <avr/sleep.h>

//Time Alarms for clickButton
//http://playground.arduino.cc/Code/TimedAction
#include <TimedAction.h>
TimedAction timedActionCheckButton = TimedAction(1,checkButton);


//SOUND & BUZZER
#import "pitches.h"
#define BEEP_DELAY_OFFSET 11  
#define BEEP_PIN 5

//main button for interaction
const int buttonPin=2;
long lastTimeClick=0;
long lastTimeLongClick=0;
boolean wasClick=false;
boolean longClick=false;
boolean wasLongClick=false;
int clickCount=0;


//Functionality
const float cicleDurationMinutes = 25;
//const float markDurationMilsecs = (cicleDurationMinutes/12)*60*1000; //12 marks per 25 minutes
const int markDurationMilsecs = 100;
const float restDurationMinutes = 5;
//const float restmarkDurationMilsecs = (restDurationMinutes/12)*60*1000; //12 marks per 25 minutes
const float restmarkDurationMilsecs =100;
long lastTime = 0;  // the last time the output pin was toggled


#define LAST_TICK_LED 11
#define LAST_RED_LED 15

int markIndex=LAST_TICK_LED;
int markIndexRedLed=LAST_TICK_LED+1;
int restMarkIndex=0;

#define MAX_LED_INTENSITY 50

#define STATE_IDLE 0 //waiting to start cycle
#define STATE_COUNTDOWN 1 //cycle counting
#define STATE_PAUSED 2 //in countdown timer is paused
#define STATE_TIMEOUT 3 //When cycle finish and alarm should sound
#define STATE_RESTTIME 4 //After alarm and before next cycle, you rest 5 minutes

unsigned int STATE=STATE_IDLE;

void setup(){
  
  
  ShiftPWM.PrintInterruptLoad();
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  ShiftPWM.Start(pwmFrequency,maxBrightness);
  //ShiftPWM.SetAll(0);
  //ShiftPWM.SetAll(255);
  //ShiftPWM.SetAll(MAX_LED_INTENSITY);
  
 pinMode(buttonPin, INPUT);  
 pinMode(BEEP_PIN, OUTPUT);
 
 Serial.begin(9600);
 
 lastTime=millis();

 beep(NOTE_C8, 20); 
 beep(NOTE_D8, 40);
 delay(500);
 
 //CicleIn(100);
 /*
  //Fill red
  for(int pin = 12; pin <= 15; pin++){
    for(unsigned char brightness = 0; brightness < 255; brightness++){
      ShiftPWM.SetOne(pin, brightness);
      delayMicroseconds(100);
    }
  }*/

  

 fadeInAll(500);
 delay(1000);
 fadeOutAll(200);
  
}




void loop()
{
   
  
  /*
    beep(NOTE_D7, 25);
    beep(NOTE_G7, 25);
  
    delay(150);
    
    beep(NOTE_D7, 25);
    beep(NOTE_G7, 25);
    
    
    delay(2500);
*/
  
  //checkButton();    

  
  //ShiftPWM.SetOne(0, 255);
   
   if (STATE==STATE_PAUSED){
      heartBeat(markIndex,500);
   }
   
   
   if (STATE==STATE_TIMEOUT){  
     heartBeat(markIndexRedLed-1,150);
     beep(NOTE_D7, 50);
     beep(NOTE_G7, 50);
     //delay(500);
   }

  if (STATE==STATE_COUNTDOWN){
    if (millis()-lastTime>markDurationMilsecs){
      tick();
      lastTime=millis();
    }
  }
  
  if (STATE==STATE_RESTTIME){
    heartBeat(markIndexRedLed-1,200);
    if (millis()-lastTime>restmarkDurationMilsecs){
      restTick();
      lastTime=millis();
    }
  }
  
  
  timedActionCheckButton.check();
  
  //delay(1);
  
  
  
}

void checkButton(){
  
   int val = digitalRead(buttonPin);
   
    if (val!=HIGH){
      if (millis()-lastTimeClick>150){ 
        wasClick=false;
        longClick=false;
      }
      if (millis()-lastTimeClick>400){ 
        clickCount=0;
      }
      return;
    }
    
     
      //Filtering clicks
     if (millis()-lastTimeClick<250){ 
      return;
     }
      
     //Long Click 
     if (wasClick && !longClick){
       longClick=true;
       lastTimeLongClick=millis();
     }
     if (longClick && millis()-lastTimeLongClick>1500){
       
       
       
       Serial.println("Long Click");       
       
       if (STATE==STATE_IDLE)return;
       
       beep(NOTE_D7, 150);
       //longClick=false;
       //Reset
       ShiftPWM.SetAll(0);
       markIndex=LAST_TICK_LED;
       markIndexRedLed=LAST_TICK_LED+1;
       STATE=STATE_IDLE;
       
       //sleepNow();
       
       return;
       
       
     }
     
     /* 
     //Double click 
     if (!doubleClick && !wasClick && millis()-lastTimeLongClick<500){
       Serial.println("Double Click");       
       beep(NOTE_D7, 25);
       beep(NOTE_G7, 25);
       
       wasClick=true;
       doubleClick=true;
       lastTimeClick=millis();
       return;
     }else{
       
     }*/
     
     //single & Double Clicks
    if (!wasClick && !longClick){
       clickCount++;
       Serial.print("Button push. ClickCount:");
       Serial.println(clickCount);
       
       
       if(clickCount==2 && STATE==STATE_RESTTIME){
           
           STATE=STATE_IDLE;
           beep(NOTE_D7, 25);
           beep(NOTE_G7, 25);
           delay(100);
           //cancel actual cycle or rest time
           restMarkIndex=0;
           fadeOutAllBlue(100);
           delay(300);
           return;
       }
       
       if (STATE==STATE_COUNTDOWN){
         STATE=STATE_PAUSED;
       }else{
         if (STATE==STATE_PAUSED){
           STATE=STATE_COUNTDOWN;
         }
       }
       
       if (STATE==STATE_TIMEOUT){
         //STATE=STATE_IDLE;
         STATE=STATE_RESTTIME;
       }else{
         if(STATE==STATE_IDLE){
          startCicle();
         }  
       }
       
       
       
       Serial.print("STATE:");
       Serial.println(STATE);
       
    } 
      
     lastTimeClick=millis();    //For filtering, double click & long click
     wasClick=true; 
      
     /*
     beep(NOTE_D7, 25);
     beep(NOTE_G7, 25);
     */
     
      //demo();
      
     
      
     
  
  
}

void heartBeat(unsigned int pin,unsigned int timedelay){
  fadeOutMark(pin,timedelay);
  fadeInMark(pin,timedelay);
}


void startCicle(){
  //Fill dial
  STATE=STATE_COUNTDOWN;
  if (markIndexRedLed>LAST_RED_LED){
    markIndexRedLed=LAST_TICK_LED+1;
    fadeOutAllRed(100);
  }
  CicleIn(90);

  Serial.print("Timer start. Cycle is Milsecs:");
  Serial.println(markDurationMilsecs);
}

void fadeOutAll(unsigned int microDelay){
    ShiftPWM.SetAll(255);
    for(unsigned char brightness = 255; brightness > 0; brightness--){
      ShiftPWM.SetAll(brightness);
      delayMicroseconds(microDelay);
    }
    ShiftPWM.SetAll(0);
}

void fadeInAll(unsigned int microDelay){
     for(unsigned char brightness = 0; brightness < 255; brightness++){
      ShiftPWM.SetAll(brightness);
      delayMicroseconds(microDelay);
    }
    ShiftPWM.SetAll(255);
}

void fadeOutAllBlue(unsigned int microDelay){
  
  for(unsigned char brightness = 255; brightness > 0; brightness--){
    for (unsigned char i=0;i<=LAST_TICK_LED;i++){
       ShiftPWM.SetOne(i, brightness);
       timedActionCheckButton.check();
    }
    delayMicroseconds(microDelay);
  }
  for (unsigned char i=0;i<=LAST_TICK_LED;i++){
       ShiftPWM.SetOne(i, 0);
       timedActionCheckButton.check();
  }
}

void fadeOutAllRed(unsigned int microDelay){
  
  for(unsigned char brightness = 255; brightness > 0; brightness--){
    for (unsigned char i=LAST_TICK_LED+1;i<=LAST_RED_LED;i++){
       ShiftPWM.SetOne(i, brightness);
       timedActionCheckButton.check();
    }
    delayMicroseconds(microDelay);
  }
  for (unsigned char i=LAST_TICK_LED+1;i<=LAST_RED_LED;i++){
       ShiftPWM.SetOne(i, 0);
       timedActionCheckButton.check();
  }
}

void fadeOutMark(int markPin,unsigned int microDelay){
  for(unsigned char brightness = 255; brightness > 0; brightness--){
      timedActionCheckButton.check();
      if (STATE==STATE_IDLE){ //If status change after push button, abort fade and return
        ShiftPWM.SetOne(markPin, 0);
        return;
      }      
      ShiftPWM.SetOne(markPin, brightness);
      delayMicroseconds(microDelay);
    }
    ShiftPWM.SetOne(markPin, 0);
}

void fadeInMark(int markPin,unsigned int microDelay){
  for(unsigned char brightness = 0; brightness < 255; brightness++){
      timedActionCheckButton.check();
      if (STATE==STATE_IDLE){ //If status change after push button, abort fade and return
        ShiftPWM.SetOne(markPin, 0);
        return;
      }
      ShiftPWM.SetOne(markPin, brightness);
      delayMicroseconds(microDelay);
    }
}


void restTick(){
  
  if (STATE!=STATE_RESTTIME)return;
  
  fadeInMark(restMarkIndex,1000);
  restMarkIndex++;
  if (restMarkIndex==LAST_TICK_LED+1){
    restMarkIndex=0;
    fadeOutAllBlue(1000);
    STATE=STATE_IDLE;
  }
  
}

void tick(){
  
  
  
  if (STATE!=STATE_COUNTDOWN)return;
  
  Serial.print("TICK ");
  Serial.println(markIndex);
  
  if (markIndex>=0){
    fadeOutMark(markIndex,1000);
    if (markIndex==0){
      //start from beggining
      STATE=STATE_TIMEOUT;
      //STATE=STATE_IDLE;
      markIndex=LAST_TICK_LED;
     
       //Red marks management 
      
      fadeInMark(markIndexRedLed,2000);
      markIndexRedLed++;
      
      return;
    }
    markIndex--;
     
  }
  
  
  
}

void CicleIn(unsigned int microsec){
  //Fill dial
  for(int pin = 0; pin <= 11; pin++){
    for(unsigned char brightness = 0; brightness < 255; brightness++){
      ShiftPWM.SetOne(pin, brightness);
      delayMicroseconds(microsec);
    }
  }
}

void demoStart(){

}


void beep(unsigned long hz, unsigned long ms) {  
    // reference: 440hz = 2273 usec/cycle, 1136 usec/half-cycle  
    //unsigned int delay_offset=11;
    unsigned int delay_offset=100;
    
    unsigned long us = (500000 / hz) - delay_offset;  
    unsigned long rep = (ms * 500L) / (us + delay_offset);  
    
    for (int i = 0; i < rep; i++) {  
        digitalWrite(BEEP_PIN, HIGH);  
        delayMicroseconds(us);  
        digitalWrite(BEEP_PIN, LOW);  
        delayMicroseconds(us);
    }  
}  

void sleepNow()
{
  
  // Main http://playground.arduino.cc/Learning/arduinoSleepCode
  // From http://rubenlaguna.com/wp/2008/10/15/arduino-sleep-mode-waking-up-when-receiving-data-on-the-usart/
  
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep modus.
     *
     * In the avr/sleep.h file, the call names of these sleep modus are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     *  the power reduction management <avr/power.h>  is described in 
     *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
     */  
     
  set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 
  attachInterrupt(1, wakeUp, CHANGE);
  
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  
  
  sleep_mode();            // here the device is actually put to sleep!!
  

 
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...

  power_all_enable();
   
}

void wakeUp(){
  detachInterrupt(1);
  sleep_disable();
  power_all_enable();
  Serial.println("Good Morning!");
  delay(300);
  //pin2_interrupt_flag = 1;
}

void demo(){
  
    ShiftPWM.SetAll(0);
    
  // For every led
  for(int pin = 0; pin <= 11; pin++){
   
    // Fade in
    for(unsigned char brightness = 0; brightness < 255; brightness++){
      ShiftPWM.SetOne(pin, brightness);
      delayMicroseconds(2000);
    }
    
    /*
    // Fade out
    for(unsigned char brightness = 255; brightness > 0; brightness--){
      ShiftPWM.SetOne(pin, brightness);
      delayMicroseconds(2000);
    }*/
        
  }
  
  for(int pin = 11; pin >=0; pin--){
    
    // Fade out
    for(unsigned char brightness = 255; brightness > 0; brightness--){
      ShiftPWM.SetOne(pin, brightness);
      delayMicroseconds(2000);
    }
    ShiftPWM.SetOne(pin, 0);
    
  }
  
  delay(200);  
}
