/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13. 
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead(). 
 
 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/AnalogInput
 
 */

#include "Adafruit_NeoPixel.h"
#include <EEPROM.h>
#include "RotaryEncoder.h"

#define HUE_PIN 9 //B1
#define SAT_PIN 10 //B2
#define BUT_PIN 11 //B3
//rotary encoder pins
#define ENC0 A0 //C8
#define ENC1 A1 //C9
#define NUM_LEDS 8*8
#define ADJ_TIMEOUT 60000 //amount of time you have after hitting the button to adjust the hue, milliseconds
#define ROLL_AVG_N 50
#define VALUE_INC 4 //amount to increment/decrement value while moving toward target_value
#define EFFECT_SPEED 2 //number of LED updates between incrementing the "effect" along the strands
#define EFFECT_BRIGHTNESS 2 //birghtness of the effect, 1/n. eg, 4 would mean that the effect is 1/4 as bright as the set brightness
#define EFFECT_TIME_MIN 10*1000//minimum amount of time between doing an effect (ms)
#define EFFECT_TIME_MAX 30*1000//Maximum time between effects (ms)

int sensorPin = A0;    // select the input pin for the potentiometer

static uint8_t effect[] = {13, 27, 40, 54, 67, 81, 94, 107, 121, 134, 148, 161, 174, 188, 201, 215, 228, 242, 255, 242, 228, 215, 201, 188, 174, 161, 148, 134, 121, 107, 94, 81, 67, 64, 40, 27, 13};

//LED control pins
uint8_t spins[] = {4, 5, 6, 7, 8};
//leds per each strand
//uint8_t snum[] = {88, 121, 119, 116, 71};
uint8_t snum[] = {71, 116, 119, 121, 88};

//Rotary encoder null object
RotaryEncoder *encoder = nullptr;


//initialize our pixel strand objects
Adafruit_NeoPixel strand[] = {
  Adafruit_NeoPixel(snum[0], spins[0], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[1], spins[1], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[2], spins[2], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[3], spins[3], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[4], spins[4], NEO_GRB + NEO_KHZ800),
};

typedef enum {
  MODE_NORM,
  MODE_HUE,
  MODE_SAT
} mode_adj_t;

//flag for enabling "easter egg" mode
bool easter_egg = 0;

//global value for the brightness being displayed
uint8_t value = 128;
uint8_t brightness = 8; //value to convert to Value (ie, HSV value)
//adjustment (hue and sat) timeout
uint32_t adj_timeout = 0;
//track the current LED strand for hue/sat adjustment
uint8_t strand_index=0;
//track what mode we are in (normal, hue adj, sat adj)  
mode_adj_t mode = MODE_NORM;

//hue and sat values for each strand
uint16_t hue[] = {0, 0, 0, 0, 0};
uint8_t sat[] = {0, 0, 0, 0, 0};
volatile uint8_t hue_press;
volatile uint8_t sat_press;
volatile uint8_t but_press;

void load_eeprom()
{
  //loop through each value of hue, and load it from a corresponding address in eeprom
  for(int i=0; i<sizeof(hue); i++)
  {
    ((uint8_t*)hue)[i] = EEPROM.read(i);
  }
  for(int i = 0; i<sizeof(sat); i++)
  {
     sat[i] = EEPROM.read(i + sizeof(hue));
  }
  for(int i=0; i<sizeof(value); i++)
  {
    sat[i] = EEPROM.read(i + sizeof(hue)+sizeof(sat));
  }
}

//same as above, opposite direction.
void save_eeprom()
{
  for(int i=0; i<sizeof(hue); i++)
  {
    EEPROM.write(i, ((uint8_t*)hue)[i]);
  }
  for(int i = 0; i<sizeof(sat); i++)
  {
     EEPROM.write(i + sizeof(hue), sat[i]);
  }
  for(int i=0; i<sizeof(value); i++)
  {
    EEPROM.write(i+sizeof(hue)+sizeof(sat), value);
  }
}


// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3.
  ISR(PCINT1_vect) {
    encoder->tick(); // just call tick() to check the state.
  }


// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3.
  ISR(PCINT0_vect) {
    static unsigned long last = 0;
    if(micros() - last < 10000)
      return;
    last = micros();
    hue_press |= !digitalRead(HUE_PIN);
    sat_press |= !digitalRead(SAT_PIN);
    but_press |= !digitalRead(BUT_PIN);
    PCIFR &= ~PCIF0;
//    PCMSK0 &= ~(1<<PCINT1 | 1<<PCINT2 | 1<<PCINT3) ;
  }

//convenience function to give me a usable color value from HSB
uint32_t get_color(uint16_t h, uint8_t s, uint8_t v)
{
  return strand[0].gamma32(strand[0].ColorHSV(h, s, v));
}

uint16_t range_checked_update(uint16_t val, uint16_t max, bool wrap)
{
  
  int16_t enc_pos = encoder->getPosition();
  if( enc_pos == 0)
    return val;
  uint16_t speed = encoder->getRPM();
  encoder->setPosition(0);
  if(speed > 3 && speed < 10)
    enc_pos *= (max / 128) ;
  else if(speed >=10 && speed < 100)
    enc_pos *= (max / 64);
  else if(speed >= 100)
    enc_pos *= (max / 32);
  if(wrap)
    val += enc_pos & max;
  else
    if(((int32_t)val + (int32_t)enc_pos) > (int32_t)max)
      val = max;
    else if(((int32_t)val + (int32_t)enc_pos) < 0)
      val = 0;
    else
      val += enc_pos; 
 
  return val;
}

//this does a lot of the heavy lifting of setting colors and running animations
void update()
{
  static uint8_t effect_location[] = {0xff,0xff,0xff,0xff,0xff}; //keeps track of where the effect starts
  static uint8_t effect_counter=0; //counts frames between an effect change
  static uint32_t next_run[] = {0,0,0,0,0}; //time index of when we should start the next effect
  static uint8_t easter_location = 0;
  //adjust the brightness value (do something here)

  //do easter egg mode
  if(easter_egg)
  {
    
    for(int i=0; i<sizeof(snum); i++)
    {
      for(int j=0; j<sizeof(effect); j++)
      {
        strand[i].setPixelColor(easter_location-sizeof(effect)+j, get_color((hue[i]+effect[j]*256) & 0xefff, 255, value)); 
      }
      //pump out new values to all the pixels
      strand[i].show();
    }
    //increment the index of where the easter egg "effect" starts
    easter_location++;
    if(easter_location > 121)
    {
      easter_egg = 0;
      easter_location = 0;
    }
    delay(15);
  }
  else
  {
    //go through each strand, and apply effect, if applicable, and update value/hue/sat
    for(int i=0; i<sizeof(snum); i++)
    { 
    	//fill the entire strand memory buffer with the right HSV
      strand[i].fill(get_color(hue[i], sat[i], value),  0, snum[i]); 
      //if were at, or later than the right time to apply the effect, and the effect loaction is out
      //of bounds, indicating that it has not been set
      if(next_run[i] > millis() && effect_location[i] == 0xff)
      {
        //then set it to zero, which is where it will start
        effect_location[i] = 0;
      }
      //if the effect location is valid, then apply the effect
      if(effect_location[i] != 0xff)
      {
        //iterate over all elements of the "effect" values
        for(int j=0; j<sizeof(effect); j++)
        {
        	//calculate the value for the pixel affected by the effect
          uint32_t eff_bright = value + effect[j]/EFFECT_BRIGHTNESS*value/255;
  
          //if the calculated brightness exceeds max, then set it to max
          if(eff_bright & 0xffffff00)
            eff_bright = 0xff; //255 (0xff) is max brightness
          //set the appropriate pixel value to the brightness it should be for the location of the effect
          strand[i].setPixelColor(effect_location[i]-sizeof(effect)+j, get_color(hue[i], sat[i], (uint8_t)eff_bright)); //this will mess up at the upper end of brightnes
        }
                  //check to see if we should increment the location of the effect (we only want to do this every
        // EFFECT_SPEED frames)
        if(!(effect_counter % EFFECT_SPEED))
        {
          //increment the index of where the "effect" starts
          effect_location[i]++;
        }
      }
  
      //check to see if the effect location is out of bounds of the LED strand. If so, set it to 255 
      //(invalid value)
      if((snum[i] + sizeof(effect)) < effect_location[i])
      {
        //again, 255 represents an invalid location, and means an effect is not running
        effect_location[i] = 0xff;
        //calculate a random time to start the next effect.
        next_run[i] = millis()+random(EFFECT_TIME_MIN, EFFECT_TIME_MAX);
      }
      //pump out new values to all the pixels
      strand[i].show();
    }
  }
  //increment our frame counter
  effect_counter++;
}

//setup our pins and whatnot. This runs once
void setup() {
  //set GRPIO direction and pullup
  pinMode(HUE_PIN, INPUT_PULLUP);
  pinMode(SAT_PIN, INPUT_PULLUP);
  pinMode(BUT_PIN, INPUT_PULLUP);
  //load saved values from EEPROM
  load_eeprom();

  //set each strand to the appropriate color
  for(int i=0; i<sizeof(snum); i++)
  {
  	//initialize the strand
    strand[i].begin();
    //clear it
    strand[i].clear();
    //set pixel values
    strand[i].fill(strand[i].ColorHSV(hue[i], sat[i], value), 0, snum[i]); 
    //pump out the values to LEDs
    strand[i].show();
  }
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(ENC0, ENC1, RotaryEncoder::LatchMode::FOUR0);
//  attachInterrupt(digitalPinToInterrupt(ENC0), checkPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENC1), checkPosition, CHANGE);
  // Setup flags to activate the ISR PCINT1.
// You may have to modify the next 2 lines if using other pins than A2 and A3

  pinMode(ENC0, INPUT_PULLUP);
  pinMode(ENC1, INPUT_PULLUP);

  //Port C interrupt
  PCICR = 1<<PCIE1 | 1<<PCIE0; // set int on port C and B
  PCIFR = 0x00; //clear interrupt flags 
  PCMSK1 = 1<<PCINT8 | 1<<PCINT9;
  PCMSK0 = 1<<PCINT1 | 1<<PCINT2 | 1<<PCINT3 ;
  
//  EICRA = 0x03;
//  attachInterrupt(digitalPinToInterrupt(ENC0), checkPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENC1), checkPosition, CHANGE);

  

}

void reset_timeout()
{
  adj_timeout = millis() + ADJ_TIMEOUT;
}

void enter_mode(mode_adj_t new_mode)
{
      if(new_mode == MODE_NORM)
      {
        save_eeprom();
        //make sure our timeout is zeroed out
        adj_timeout = 0;
      }
      else
      {
        if(new_mode == mode)
          strand_index = (strand_index + 1) % sizeof(spins);
        //blank the currently controlled strand for a bit
        strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
        strand[strand_index].show();  
        delay(100);
        reset_timeout();
      }
      mode = new_mode;
}

//this runs over and over... Forever.
void loop() {  
  if(hue_press)
  {
    enter_mode(MODE_HUE);
    hue_press = 0;
  }
  if(sat_press)
  {
    enter_mode(MODE_SAT);
    sat_press = 0;
  }
  if(but_press)
  {
    if(mode == MODE_NORM)
      easter_egg =1;
    enter_mode(MODE_NORM);
    but_press = 0;
  }

  //give us more time if we're in an adjustment mode, and there is encoder movement.
  if(mode != MODE_NORM)
  {
    if(adj_timeout < millis())
      enter_mode(MODE_NORM);
    else if(encoder->getPosition())
      reset_timeout();
  }

  switch(mode)
  {
    case MODE_NORM: value = range_checked_update(value, 255, 0); break;
    case MODE_HUE: hue[strand_index] = range_checked_update(hue[strand_index], 65535, 1); break;
    case MODE_SAT: sat[strand_index] = range_checked_update(sat[strand_index], 255, 0); break;
  }
  //update the pixels
  update();
}
