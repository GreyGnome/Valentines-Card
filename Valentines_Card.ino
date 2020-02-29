/*
   Copyright [2019] [Michael Anthony Schwager]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   A copy of the license is also provided in a file alongside this source code
   at https://github.com/GreyGnome/IndividualLED/+***SOMETHING***
 */
// === DESCRIPTION =====================================================================
// ATMEL ATMEGA8A / ATMEGA328P / ARDUINO
// s == seven segment display pin, l == led strip pin, u == up button, d == down button,
// h == hours button, x=debug signal, y=debug clock.
//
//                           +-\/-+
//     RESET   (D 22)  PC6  1|    |28  PC5  (D 19  A5  SCL ADC5)
//             (D  0)  PD0  2|    |27  PC4  (D 18  A4  SDA ADC4)
//             (D  1)  PD1  3|    |26  PC3  (D 17  A3  ADC3)
//       INT0  (D  2)  PD2  4|    |25  PC2  (D 16  A2  ADC2)
//  OC2B INT1  (D  3)  PD3  5|    |24  PC1  (D 15  A1  ADC1)
//             (D  4)  PD4  6|    |23  PC0  (D 14  A0  ADC0)
//                     VCC  7|    |22  GND
//                     GND  8|    |21  AREF
//      TOSC1  (D 20)  PB6  9|    |20  AVCC
//             (D 21)  PB7 10|    |19  PB5  (D 13  SCK) 
//             (D  5)  PD5 11|    |18  PB4  (D 12  MISO) 
//  AIN0       (D  6)  PD6 12|x   |17  PB3  (D 11  MOSI OC2A)
//  AIN1       (D  7)  PD7 13|clk |16  PB2  (D 10  SS OC1A)
//             (D  8)  PB0 14|    |15  PB1  (D  9     OC1B)
//                           +----+
#include <avr/interrupt.h>
#include "digitalWriteFast.h"

#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
#define OC2A_PIN 11
#define OC2B_PIN  3

/* DEBUGGING //////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
const uint8_t EXT_CLK_IN = 20; // Pin 20 NOT ON CRYSTAL ARDUINOS!!!
                               //Only if the internal oscillator is used as a clock
const uint8_t INDICATOR = 15;
// These must be on the same port.
const uint8_t DEBUG_X = 6; // bit indicator for debugging
const uint8_t DEBUG_CLK = 7; // Clock for the above

const uint8_t x_bit = digitalPinToBit(DEBUG_X);
const uint8_t y_bit = digitalPinToBit(DEBUG_CLK);
const uint8_t debug_hi = (1 << digitalPinToBit(DEBUG_X)) | (1 << digitalPinToBit(DEBUG_CLK));
const uint8_t debug_lo = (1 << digitalPinToBit(DEBUG_CLK));
const uint8_t debug_rst = (uint8_t (~debug_hi)) & (uint8_t (~debug_lo));

// Send a bit out with a clock. The bit's value depends on V. The clock triggers always.
#define DEBUG_CLK(V) ((V) != 0) ? *(digitalPinToPortReg(DEBUG_X)) |= debug_hi : (*(digitalPinToPortReg(DEBUG_X)) |= debug_lo); *(digitalPinToPortReg(DEBUG_X)) &= debug_rst
// Set the value of the debug X port, based on V. It is not reset.
#define DEBUG_BIT(V) ((V) != 0) ? *(digitalPinToPortReg(DEBUG_X)) |= debug_hi : (*(digitalPinToPortReg(DEBUG_X)) |= debug_lo)
// To use:
/*digitalWriteFast(DEBUG_X, HIGH);
__asm__ __volatile__ ("nop");
__asm__ __volatile__ ("nop");
digitalWriteFast(DEBUG_X, LOW);*/
static inline void isr_display_value(uint8_t value) {
  uint8_t i;
  for (i=0b10000000; i > 0; i = i>>1) {
    DEBUG_CLK(i & value);
  }
}
/* DEBUGGING //////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/


/* LED VALUES /////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
// Requires pgm_read_byte() to get it out. Costs 1 extra cycle. Not worth it on slower CPUs.
// const uint8_t PROGMEM segment_pins[] = {0, 1, 2, 3, 4, 5, 6, 7};
// 20 is TOSC1!!!
// WHITE: 0, 1, 2, 3
// RED:   16, 17, 18, 19
#define WHITE_1 0
#define WHITE_2 1
#define WHITE_3 2
#define WHITE_4 3
#define RED_1 4
#define RED_2 5
#define RED_3 6
#define RED_4 7
// WHITE_1, WHITE_2, WHITE_3, WHITE_4, RED_1, RED_2, RED_3, RED_4};
const uint8_t led_pins[] = {0, 1, 2, 3, 16, 17, 18, 19};
//const uint8_t led_pins[] = {0, 1, 19};
//const uint8_t led_pins[] = {18, 19}; //2, 4, 16, 17, 18, 19};
//Why is an FF at any position, replicated at the next position?
//Not only FF, but FD, FE, FF. FC seems safe to use. This is odd behavior.
//It must have to do with the double-buffered nature of the OCR2A. It takes a
//couple of cycles before it will actually take effect, which isn't good enough
//for me.
volatile uint8_t led_brightness[] = {0xFC, 0xFC, 0xFC, 0xFC, 0x08, 0x08, 0x08, 0x08};
//volatile uint8_t led_brightness[] = {0, 0};

volatile uint8_t current_led=0;
volatile uint8_t brightness=0;
// On is LOW...
#define LED_ON 0
#define LED_OFF 1

volatile uint8_t my_millis=0;
ISR(TIMER0_COMPB_vect) {
  my_millis++;
}

ISR(TIMER2_COMPB_vect) {
  //digitalWriteFast(OC2A_PIN, HIGH);
  //digitalWriteFast(OC2A_PIN, LOW);
  //digitalWriteFast(OC2B_PIN, HIGH);
  //digitalWriteFast(OC2B_PIN, LOW);
  //DEBUG_CLK(1);
  //DEBUG_CLK(1);
}

volatile boolean ovf_vect=false;
volatile boolean compa_vect=false;
volatile uint8_t this_led_pin = led_pins[0];
/*
 * During the Overflow Interrupt, we need to
 * - advance the active LED pin to the next
 * - retrieve the led_brightness
 * - when this ISR is hit, the LED should go low (unless it's 0; then just stay 1).
 */
ISR(TIMER2_OVF_vect) {
  this_led_pin=led_pins[current_led];
  // compensate for the short-pulse problem with FastPWM when OCR2A==0; 
  if (OCR2A != 0) { digitalWriteFast(this_led_pin, LED_ON); }
}

//ISR(timer2_ovf_vect) {
// COMPA (pin 11) goes high to turn the LED on, for the duration of its PWM period.
// At the end of its PWM period, the LED (-) pin should go LOW
// to thus shut off the LED.
// LED + goes to pin 11, OC2A. LED - goes to the individual LED pins.
ISR(TIMER2_COMPA_vect) {
  uint8_t bright;

  digitalWriteFast(this_led_pin, LED_OFF);
  current_led++;
  if (current_led == ARRAY_SIZE(led_pins)) current_led = 0;
  bright = led_brightness[current_led];
  // See https://forum.arduino.cc/index.php?topic=665510.0 .
  // Because of double-buffered OCR2A, setting this is a bit problematic.
  if (bright > 0xFC) bright = 0xFC;
  OCR2A = bright; // DOUBLE BUFFERED, SET ON BOTTOM!
  // NOOP ************************ (for reference)
  // __asm__ __volatile__ ("nop");
}

void set_all_pins_input() {
  for (uint8_t i=0; i < ARRAY_SIZE(led_pins); i++) {
    pinMode(led_pins[i], INPUT);
  }
}

inline void indicate_flash(uint8_t counter) {
  uint8_t indicator=INDICATOR;

  pinMode(indicator, OUTPUT);
  if (counter == 0) {
    counter=10;
    for (uint8_t j=0; j < counter; j++) {
      digitalWriteFast(indicator, LED_ON); // on
      _delay_ms (75);
      digitalWriteFast(indicator, LED_OFF); // off
      _delay_ms (75);
    }
  } else {
    for (uint8_t j=0; j < counter; j++) {
      digitalWriteFast(indicator, LED_ON); // on
      _delay_ms (200);
      digitalWriteFast(indicator, LED_OFF); // off
      _delay_ms (200);
    }
  }
}

// Flash a few times (value) at the indicator pin.
void indicate (uint8_t value) {
  // debugging
  uint8_t local_value=value;
  uint8_t to_display;
  
  if (value < 10) {
    indicate_flash(value);
    return;
  }
  to_display=local_value / 100;
  local_value=local_value - (to_display * 100);
  indicate_flash(to_display);
  _delay_ms (500);
  to_display=local_value / 10;
  local_value=local_value - (to_display * 10);
  indicate_flash(to_display);
  _delay_ms (500);
  indicate_flash(local_value);
  _delay_ms (500);
  //
}

void set_pin_directions(void) {
  uint8_t i;
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    pinMode(led_pins[i], OUTPUT);
  }
  // pinMode(INDICATOR, OUTPUT); // Debugging
  // pinMode(DEBUG_X, OUTPUT);   // Debugging
  // pinMode(DEBUG_CLK, OUTPUT); // Debugging
  pinMode(OC2A_PIN, OUTPUT);
  //pinMode(OC2B_PIN, OUTPUT);
  //pinMode(EXT_CLK_IN, INPUT);  // TOSC1 for external clock
}

void set_timer2_ctc(void) {
  // Set up timer 2
  TCCR2A = 0;
  TCCR2B = 0;
  // 0x7B == *approximately* 1024 times per second at 8MHz clock
  OCR2A = 0xFF;  // 8 MHz clock: f = 8,000,000 / (2 * prescalar * (1 + OCR2)
  //OCR2B = 0xF1;
  //  One could set bits this way:
  //  sbi(TCCR2A, WGM20);
  //  Or clear them this way:
  //  cbi(TCCR2A, COM2A0);
  TCCR2A &= ~(1 << COM2A0); // shut off output pin
  TCCR2A &= ~(1 << COM2A1); // shut off output pin
  TCCR2A |= (1 << COM2A0); // turn on output pin, set to toggle.
  TCCR2A &= ~(1 << COM2B0); // shut off output pin
  TCCR2A &= ~(1 << COM2B1); // shut off output pin
  //TCCR2A |= (1 << COM2B0); // turn on output pin, set to toggle.
  TCCR2A |= (1 << WGM21);  /* CTC mode */
  TIMSK2 |= (1 << TOIE2); // enable timer2 overflow interrupt- not reached if OCIE2A enabled
  //TIMSK2 |= (1 << OCIE2B); /* enable timer2 compare B interrupt */
  TIMSK2 |= (1 << OCIE2A); /* enable timer2 compare A interrupt */
}

void set_timer2_fastpwm(void) {
  // Note that when OCR2 == 0, there will be a spike when the TCNT2 hits BOTTOM.
  // Set up timer 2
  // 8 MHz clock: f = 8,000,000 / (prescalar * 256)
  TCCR2A = 0;
  TCCR2B = 0; // Shut off clock entirely (CS2{2,1,0} == 000)
  // 0x7B == *approximately* 1024 times per second at 8MHz clock
  //OCR2A = 0x17;  // 8 MHz clock: f = 8,000,000 / (prescalar * 256)
  OCR2A = led_brightness[0];    //
  //OCR2B = 0xF1;
  //  One could set bits this way: // if wiring_private.h is included.
  //  sbi(TCCR2A, WGM20);
  //  Or clear them this way:
  //  cbi(TCCR2A, COM2A0);
  /* Not on 16 MHz crystal controlled chips
  ASSR |= (1 << EXCLK); // Enable External clock
  ASSR |= (1 << AS2);   // External clock from TOSC1
  */
  //TCCR2A &= ~(1 << COM2A0 | 1 << COM2A1); // shut off output pin
  TCCR2A |= (1 << COM2A1); // output pin, non-inverting (clear on match, set at bottom)
  TCCR2A &= ~(1 << COM2B0); // shut off output pin
  TCCR2A &= ~(1 << COM2B1); // shut off output pin
  //TCCR2A |= (1 << COM2B0); // output pin, non-inverting (clear on match, set at bottom)
  TCCR2A |= (1 << WGM21 | 1 << WGM20);  /* Fast PWM mode, TOP=0xFF */
  TIMSK2 |= (1 << TOIE2); // enable timer2 overflow interrupt.
  //TIMSK2 |= (1 << OCIE2B); /* enable timer2 compare B interrupt */
  TIMSK2 |= (1 << OCIE2A); /* enable timer2 compare A interrupt */
  TCNT2=0;
  //ASSR |= (1 << EXCLK);    // external clock
  //ASSR |= (1 << AS2);
}
inline void turn_on_clock() {
  //TCCR2B |= (1 << CS20);   // x1 prescaler
  TCCR2B |= (1 << CS22);   // x64 prescaler 
  //TCCR2B |= (1 << CS21);   // x8 prescaler, f=3906 Hz (measured 258 uS, == 3875 Hz)
  //TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // x1024 prescaler, f=61 Hz
}

// the setup function runs once when you press reset or power the board
void setup() {
  //delay(100);
  uint8_t i=0;
  set_timer2_fastpwm();
  sei();
  set_pin_directions();
  //Serial.begin(230400);
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    digitalWriteFast(led_pins[i], LED_OFF);
  }
  digitalWriteFast(OC2A_PIN, HIGH);
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    digitalWriteFast(11, HIGH);
    digitalWriteFast(led_pins[i], LED_ON);
    _delay_ms(200);
    digitalWriteFast(led_pins[i], LED_OFF);
  }
  digitalWriteFast(OC2A_PIN, LOW);
  //TIFR2 |= (1 << TOV2);    /* clear interrupt flag */
  // 800000000000000000000000000000000000000000000000000000000000000000000000000000
  ADCSRA &= ~(1<<ADEN); // Disable ADC for better power consumption (ATmega8)
                        // Might need PRR |= (1 << PRADC) 
  // indicate(2); Sanity check. Make sure: set INDICATOR to output in set_pin_directions
  // Start out sanely
  digitalWriteFast(OC2A_PIN, HIGH);
  turn_on_clock();
  // Utilize the timer0, which is already running for millis().
  OCR0B=0x00;  // 0xFF is constant high. 0x00 shows a single pulse.
               // This takes place every 1 ms as measured on a scope, so using
               // the compare interrupt as ms timer is good.
  TIMSK0 |= (1 << OCIE0B);
}


const uint8_t ALL = 0xFF;
void off(uint8_t led) {
  if (led == ALL) {
    for (uint8_t i=0; i < ARRAY_SIZE(led_pins); i++) {
      led_brightness[i] = 0;
    }
    return;
  }
  led_brightness[led] = 0;
}

void on(uint8_t led, uint8_t level) {
  if (led == ALL) {
    for (uint8_t i=0; i < ARRAY_SIZE(led_pins); i++) {
      led_brightness[i] = level;
    }
    return;
  }
  led_brightness[led] = level;
}

const uint8_t UP=1;
const uint8_t DOWN=0;
uint8_t light_level = 0;
uint8_t direction = UP;
void twinkle(uint8_t led, uint8_t level) {
  uint8_t light_level_difference = 0;

  if (light_level < 50) light_level_difference = 5;
  else if (light_level < 80) light_level_difference = 10;
  else light_level_difference = 20;
  if (direction == UP)
    light_level += light_level_difference;
  else
    light_level -= light_level_difference;

  led_brightness[led] = light_level;
  if ((light_level >= level) || (light_level > 234)) direction=DOWN;
  if (light_level < 6) direction=UP;
}

uint32_t last_ms = 0;
uint8_t last_cnt = 0;
boolean loop_first_half = true;
// the loop function runs forever
void loop() {
  uint16_t ms_counter;
  uint8_t current_millis;

  off(ALL);
  _delay_ms(500);
  on(ALL, 0xFF);
  _delay_ms(500);
  off(ALL);

  // twinkle it
  ms_counter = 0;
  current_millis=my_millis;
  light_level = 0;
  direction = UP;
  while(ms_counter < 200) {
    _delay_ms(50);
    ms_counter++;
    twinkle(WHITE_2, 0xFF);
  }
}

