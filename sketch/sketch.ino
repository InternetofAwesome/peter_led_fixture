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

// Hysteresis value for reading the pot
#define POT_HYS 4
#define POT_SW_PIN 2
#define POT_SW_POL 0 //active low
#define BONUS_SW_PIN 3
#define BONUS_SW_POL 0 //active low
#define NUM_LEDS 8*8
#define HUE_ADJ_TIMEOUT 2000 //amount of time you have after hitting the button to adjust the hue, milliseconds
#define LEDS_PER_STRAND 8
#define NUM_STRANDS 5
#define ROLL_AVG_N 32

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

//LED control pins
uint8_t spins[] = {4, 5, 6, 7, 8};
//leds per each strand
uint8_t snum[] = {88, 121, 119, 116, 71};

Adafruit_NeoPixel strand[] = {
  Adafruit_NeoPixel(snum[0], spins[0], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[1], spins[1], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[2], spins[2], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[3], spins[3], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(snum[4], spins[4], NEO_GRB + NEO_KHZ800),
};

typedef enum {
  NORMAL = 0, //normal pot operation. Controls brightness
  POT_OFF,
  POT_ON,
  COLOR_CONTROL,
} pot_state_e;

typedef enum {
  OFF,
  DEBOUNCE,
  ON,
} button_state_e;

typedef struct {
  uint8_t pin;
  uint8_t pol;
  button_state_e state;
  uint16_t debounce_millis;
} button_t;

button_t pot_sw = {POT_SW_PIN, POT_SW_POL, OFF, 50};
button_t bonus_sw = {BONUS_SW_PIN, BONUS_SW_POL, OFF, 50};

pot_state_e pot_state = NORMAL;

uint8_t value = 128;
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

void set_value(uint8_t val)
{
  int inc;
  if(val == value)
  {
    for(int i=0; i<NUM_STRANDS; i++)
    { 
      strand[i].fill(get_color(hue[i], sat[i], value),  0, snum[i]); 
    }
    return;
  }
  if(val < value)
    inc = -1;
  else
    inc = 1;
    
  inc *= 4;
  
  while(abs(value - val) > 4)
  {
    value += inc;
    for(int i=0; i<NUM_STRANDS; i++)
    {
      strand[i].fill(get_color(hue[i], sat[i], value),  0, snum[i]); 
      strand[i].show();
    }  
//    delay(5);
  }
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
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
//  clock_prescale_set(clock_div_1);
  //setup our gpios
  pinMode(POT_SW_PIN, INPUT_PULLUP);
  pinMode(BONUS_SW_PIN, INPUT_PULLUP);
  load_eeprom();
  value_sum = value*ROLL_AVG_N;
  for(int i=0; i<NUM_STRANDS; i++)
  {
    strand[i].begin();
    strand[i].clear();
    strand[i].fill(strand[i].ColorHSV(hue[i], 255, value), 0, snum[i]); 
    strand[i].show();
  }
}

void loop() {
//  strand[0].clear();
//  for(int j=1; j<150; j++)
//  {
//    strand[0].fill(strand[0].ColorHSV(0, 0, 0), 0, j-1);
//    strand[0].fill(strand[0].ColorHSV(0, 0, 255), j, j);
//    strand[0].fill(strand[0].ColorHSV(0, 0, 0), j+1, 150);
//    strand[0].show();
////  delay(1);
//  }
//  return;
  
  
  uint16_t pot;
  static uint32_t hue_adj_end = 0;
  static uint32_t sat_adj_end = 0;
  static uint16_t last_pot;
  static uint8_t strand_index=0;
  static uint16_t pot_hue_offset;
  static uint8_t pot_sat_offset;
  uint8_t button;
  uint8_t button2;
  button = !digitalRead(POT_SW_PIN);
  button2 = !digitalRead(3);
  
  pot = read_pot(); 
  value_sum -= value_sum/ROLL_AVG_N;
  value_sum += pot>>2 & 0xff;
  
  if(button == 1 || hue_adj_end > millis())
  {
    set_value(64);
    if(hue_adj_end > millis() && button == 1)
    {
      strand_index = (strand_index + 1) % NUM_STRANDS;
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    if(hue_adj_end == 0)
    {
      pot_hue_offset = pot << 6;
      strand[strand_index].fill(strand[strand_index].Color(0,0,0), 0, snum[strand_index]);
      strand[strand_index].show();  
      delay(100);
      hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
    }
    if ( abs(pot - last_pot) > 20)
    {
       hue_adj_end = millis() + HUE_ADJ_TIMEOUT;
       last_pot = pot;
    }
    hue[strand_index] = hue[strand_index] + ((pot << 6) - pot_hue_offset);
    pot_hue_offset = pot << 6;
    strand[strand_index].fill(get_color(hue[strand_index], sat[strand_index], value), 0, snum[strand_index]);
    strand[strand_index].show(); 
  }
  else if(button2 == 1 || sat_adj_end > millis())
  {
    set_value(64);
    if(sat_adj_end > millis() && button2 == 1)
    {
      strand_index = (strand_index + 1) % NUM_STRANDS;
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
    //sat[strand_index] = sat[strand_index] + ((pot >> 2) - pot_sat_offset);
    //pot_sat_offset = pot >> 2;
    sat[strand_index] = pot >> 2;
    strand[strand_index].fill(get_color(hue[strand_index], sat[strand_index], value), 0, snum[strand_index]);
    strand[strand_index].show(); 
  }
  else
  {
    //value = c_lin[pot>>2 & 0xff];
    set_value(value_sum/ROLL_AVG_N);
    //value = value_sum/8;
  }
  
  if((hue_adj_end < millis() && hue_adj_end != 0) ||  (sat_adj_end < millis() && sat_adj_end != 0))
  { 
     save_eeprom();
//     for(int i=0; i<NUM_STRANDS; i++)
//     {
//       strand[i].fill(strand[i].Color(0,0,0), 0, snum[i]);
//       strand[i].show();  
//     }
     set_value(0);
     delay(100);
//     value = 0;
     set_value(value_sum/ROLL_AVG_N);
     hue_adj_end = 0;
     sat_adj_end = 0;
  }
  //pixels.show?
}
