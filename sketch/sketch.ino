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

//gamma corrected lookup tables from Adafruit lib
extern const uint8_t gamma8[];

#include "Adafruit_NeoPixel.h"
#include <EEPROM.h>

#define HUE_PIN 2
#define SAT_PIN 3
#define NUM_LEDS 8*8
#define HUE_ADJ_TIMEOUT 2000 //amount of time you have after hitting the button to adjust the hue, milliseconds
#define ROLL_AVG_N 50
#define VALUE_INC 4 //amount to increment/decrement value while moving toward target_value
#define EFFECT_SPEED 2 //number of LED updates between incrementing the "effect" along the strands
#define EFFECT_BRIGHTNESS 2 //birghtness of the effect, 1/n. eg, 4 would mean that the effect is 1/4 as bright as the set brightness
#define EFFECT_TIME_MIN 1*1000//minimum amount of time between doing an effect (ms)
#define EFFECT_TIME_MAX 2*1000//Maximum time between effects (ms)

int sensorPin = A0;    // select the input pin for the potentiometer

static uint8_t effect[] = {13, 27, 40, 54, 67, 81, 94, 107, 121, 134, 148, 161, 174, 188, 201, 215, 228, 242, 255, 242, 228, 215, 201, 188, 174, 161, 148, 134, 121, 107, 94, 81, 67, 64, 40, 27, 13};

//LED control pins
uint8_t spins[] = {4, 5, 6, 7, 8};
//leds per each strand
//uint8_t snum[] = {88, 121, 119, 116, 71};
uint8_t snum[] = {71, 116, 119, 121, 88};

Adafruit_NeoPixel strand[] = {
  Adafruit_NeoPixel(snum[0], spins[0], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[1], spins[1], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[2], spins[2], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[3], spins[3], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[4], spins[4], NEO_GRB + NEO_KHZ800),
};


uint8_t value = 128;
uint8_t target_value;
uint16_t hue[] = {0, 0, 0, 0, 0};
uint8_t sat[] = {0, 0, 0, 0, 0};

uint32_t value_sum; //used for running average of value

void load_eeprom()
{
  for(int i=0; i<sizeof(hue); i++)
  {
    ((uint8_t*)hue)[i] = EEPROM.read(i);
  }
  for(int i = 0; i<sizeof(sat); i++)
  {
     sat[i] = EEPROM.read(i + sizeof(hue));
  }
}

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
}

uint32_t get_color(uint16_t h, uint8_t s, uint8_t v)
{
  return strand[0].gamma32(strand[0].ColorHSV(h, s, v));
}

void update()
{
  static uint8_t effect_location[] = {0,0,0,0,0}; //keeps track of where the effect starts
  static uint8_t effect_counter=0; //counts frames between an effect change
  static uint32_t next_run[] = {0,0,0,0,0};
  if(target_value != value) //only adjust balue if they are different
  {
    if(abs(target_value - value) < VALUE_INC) //if they need to adjusted less than our increment value
      value = target_value; //then just set them equal
    else if(target_value - value > 0) //find out if we need to increment or decrement value
      value += VALUE_INC;
    else
      value -= VALUE_INC;
  }
  
  for(int i=0; i<sizeof(snum); i++)
  { 
    strand[i].fill(get_color(hue[i], sat[i], value),  0, snum[i]); 
    if(next_run[i] < millis() && effect_location[i] == 0xff)
    {
      effect_location[i] = 0;
    }
    if(effect_location[i] != 255)
    {
      for(int j=0; j<sizeof(effect); j++)
      {
        //uint32_t eff_bright = (uint32_t)value + ((uint32_t)value * (uint32_t)effect[j])/0xff/(uint32_t)EFFECT_BRIGHTNESS;
        uint32_t eff_bright = value + effect[j]/EFFECT_BRIGHTNESS*value/255;
        if(eff_bright & 0xffffff00)
          eff_bright &= 0xff;
        strand[i].setPixelColor(effect_location[i]-sizeof(effect)+j, get_color(hue[i], sat[i], (uint8_t)eff_bright)); //this will mess up at the upper end of brightness
        //strand[i].setPixelColor(effect_location+j, get_color(hue[i], sat[i], value + 20)); //this will mess up at the upper end of brightness
      }
    }
    if(!(effect_counter % EFFECT_SPEED))
    {
      effect_location[i]++;
    }
    if((snum[i] + sizeof(effect)) < effect_location[i])
    {
      effect_location[i] = 255;
      next_run[i] = millis()+random(EFFECT_TIME_MIN, EFFECT_TIME_MAX);
    }
    //strand[i].fill(get_color(hue[i], sat[i], 0),  effect_location, snum[i]); 
    strand[i].show();
  }
  effect_counter++;
}

int read_pot()
{
  return analogRead(sensorPin) & 0b1111111111;
}

uint8_t read_button(uint8_t pin)
{
  if(!digitalRead(pin)) //look for active low
  {
    uint16_t start = millis();
    delayMicroseconds(3000); //delay to debounce
    while(!digitalRead(pin) && ((millis() - start) < 250)); //waste time until the button is released, or we time out
    if(!digitalRead(pin))
    {
      while(!digitalRead(pin)); //wait until switch lifted
      return 0;
    }
    else
      return 1;
  }
  return 0; //no button press
}


void setup() {
  pinMode(HUE_PIN, INPUT_PULLUP);
  pinMode(SAT_PIN, INPUT_PULLUP);
  load_eeprom();
  value = read_pot()>>2;
  value_sum = value*ROLL_AVG_N;
  for(int i=0; i<sizeof(snum); i++)
  {
    strand[i].begin();
    strand[i].clear();
    strand[i].fill(strand[i].ColorHSV(hue[i], 255, value), 0, snum[i]); 
    strand[i].show();
  }
}

void loop() {  
  uint16_t pot;
  static uint32_t hue_adj_end = 0;
  static uint32_t sat_adj_end = 0;
  static uint16_t last_pot;
  static uint8_t strand_index=0;
  static uint16_t pot_hue_offset;
  static uint8_t pot_sat_offset;
  uint8_t hue_button;
  uint8_t sat_button;
  hue_button = !digitalRead(HUE_PIN);
  sat_button = !digitalRead(SAT_PIN);
  
  pot = read_pot(); //read the pot value *once* at the top of the loop
  value_sum -= value_sum/ROLL_AVG_N; //subtract the average value from the rolling sum of the pot value
  value_sum += pot>>2 & 0xff; //add the actual pot value to the rolling sum. ADC is 10 bits, so we shift right 2 to make the right range for the LED brightness of 0-255
  
  //if the hue button is down, OR our hue adjustment timeout has not elapsed
  if(hue_button == 1 || hue_adj_end > millis()) 
  {
    //set a brightness target of a fixed (arbitrary) value 
    target_value = 64;
    //if our timeout has NOT elapsed, and the hue button has been pressed
    if(hue_adj_end > millis() && hue_button == 1)
    {
      //increment the index of the strand we are adjusting
      strand_index = (strand_index + 1) % sizeof(snum);
      //set that strand to all black for a short amount of time, so we can identify it.
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      
      //reset our timeout to some time in the future
      hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    //if we do not have a timeout set
    if(hue_adj_end == 0)
    {
      /* save our current pot location, so we can subtract it out. This will allow us to adjust the
      hue RELATIVE to the current pot position, instead of just using the ABSOLUTE value of the pot
      reading, which would instantly change the color when we changed to adjust that strand.
      Also, the Hue controls are 16-bits, hence the shift left */
      pot_hue_offset = pot << 6;
      //blank the currently controlled strand for a bit
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      //reset our tiemout
      hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    //this was an attempt to add some noise immunity. If there's too much noise, the adjustment 
    //mode will never time out
    if ( abs(pot - last_pot) > 20)
    {
       hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
       last_pot = pot;
    }
    //use our pot offset to get a new hue value, using the pot value
    hue[strand_index] = hue[strand_index] + ((pot << 6) - pot_hue_offset);
    //reset the pot offset (we always want to subtract it out)
    pot_hue_offset = pot << 6;
  }
  //this block is almost exactly the same as the above if-block. See comments above
  else if(hue_button == 1 || sat_adj_end > millis())
  {
    target_value = 64;
    if(sat_adj_end > millis() && hue_button == 1)
    {
      strand_index = (strand_index + 1) % sizeof(snum);
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      sat_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    if(sat_adj_end == 0)
    {
      //pot_sat_offset = pot >> 2;
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      sat_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    if ( abs(pot - last_pot) > 20)
    {
       sat_adj_end = millis() + HUE_ADJ_TIMEOUT;
       last_pot = pot;
    }
    //update the 8-bit saturation target
    sat[strand_index] = pot >> 2; 
  }
  else
  {
  	//adjust the target value. See the update function for why that is important.
    target_value = value_sum/ROLL_AVG_N;
  }
  
  //if our hue adjustment timeout has elapsed, and hue adjustment timeout is non-zero (non-zero 
  // means there is not one set). Same logic applies for the saturation timeout
  if((hue_adj_end < millis() && hue_adj_end != 0) ||  (sat_adj_end < millis() && sat_adj_end != 0))
  { 
  	//save the hue and saturation values to EEPROM
    save_eeprom();
    //set target value - This may be redundant to the above setting operation.
    target_value = value_sum/ROLL_AVG_N;

    //make sure our timeouts are zeroed out
    hue_adj_end = 0;
    sat_adj_end = 0;
  }
  //update the pixels
  update();
}
