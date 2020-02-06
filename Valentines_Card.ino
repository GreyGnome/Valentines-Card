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
//             (D 20)  PB6  9|    |20  AVCC
//             (D 21)  PB7 10|    |19  PB5  (D 13  SCK) 
//             (D  5)  PD5 11|    |18  PB4  (D 12  MISO) 
//  AIN0       (D  6)  PD6 12|x   |17  PB3  (D 11  MOSI OC2A)
//  AIN1       (D  7)  PD7 13|clk |16  PB2  (D 10  SS OC1A)
//             (D  8)  PB0 14|    |15  PB1  (D  9     OC1B)
//                           +----+
#include <avr/interrupt.h>
#include "digitalWriteFast.h"

// Requires pgm_read_byte() to get it out. Costs 1 extra cycle. Not worth it on slower CPUs.
// const uint8_t PROGMEM segment_pins[] = {0, 13, 12, 11, 1, 4, 20, 10};
const uint8_t led_pins[] = {0, 1, 2, 3, 16, 17, 18, 19};
volatile uint8_t led_mux_sequence=0;
volatile uint8_t led_pin=0, current_led=0;
// On is HIGH...
#define LED_ON 1
#define LED_OFF 0

/* DEBUGGING //////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// These must be on the same port. */
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

static inline void isr_display_value(uint8_t value) {
  uint8_t i;
  for (i=0b10000000; i > 0; i = i>>1) {
    DEBUG_CLK(i & value);
  }
}

/* DEBUGGING //////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

//ISR(timer2_ovf_vect) {
ISR(TIMER2_COMPA_vect) {
  DEBUG_CLK(1);
  DEBUG_CLK(1);
  /*
  digitalWriteFast(DEBUG_CLK, HIGH);
  digitalWriteFast(DEBUG_CLK, LOW);
  return;

  // NOOP ************************
  // __asm__ __volatile__ ("nop");

  // do seven segment display
  digitalWriteFast(led_pin, LED_OFF); // reset old pin for multiplexing
  led_pin = led_pins[current_led];
  digitalWriteFast(led_pin, LED_OFF); // reset new pin for multiplexing

  if (led_mux_sequence < 20) {
    digitalWriteFast(led_pin, LED_ON); // Need logic here.
  }
  current_led++;
  if (current_led == sizeof(led_pins)) {
    current_led = 0;
  }
  led_mux_sequence++; */
}

void set_all_pins_input() {
  for (uint8_t i=0; i < 8; i++) {
    pinMode(led_pins[i], INPUT);
  }
}

// Flash a few times (value) at the indicator pin.
void indicate (uint8_t value) {
  // debugging
  uint8_t indicator=16;

  if (value==0) value=10;
  pinMode(indicator, OUTPUT);
  for (uint8_t j=0; j < value; j ++) {
    digitalWriteFast(indicator, LED_ON); // on
    delay (200);
    digitalWriteFast(indicator, LED_OFF); // off
    delay (200);
  }
  delay(500);
  //
}

void set_pin_directions(void) {
  uint8_t i;
  for (i=0; i <= 7; i++) {
    // pinMode(led_pins[i], OUTPUT);
    // digitalWriteFast(led_pins[i], LED_OFF);
  }
  pinMode(DEBUG_X, OUTPUT);
  pinMode(DEBUG_CLK, OUTPUT);
  pinMode(11, OUTPUT);
}

// the setup function runs once when you press reset or power the board
void setup() {
  //delay(100);
  // Set up timer 2
  TCCR2A = 0;
  TCCR2B = 0;
      // 0x7B == *approximately* 1024 times per second at 8MHz clock
  OCR2A = 0x17;  // 8 MHz clock: f = 8,000,000 / (2 * prescalar * (1 + OCR2)
                // then divide by 256.
  //  One could set bits this way:
  //  sbi(TCCR2A, WGM20);
  //  Or clear them this way:
  //  cbi(TCCR2A, COM2A0);
  sei();
  TCCR2A &= ~(1 << COM2A0); // shut off output pin
  //TCCR2A |= (1 << COM2A0); // turn on output pin, set to toggle.
  TCCR2A &= ~(1 << COM2A1); // shut off output pin
  TCCR2B &= ~(1 << COM2B0); // shut off output pin
  TCCR2B &= ~(1 << COM2B1); // shut off output pin
  TCCR2A |= (1 << WGM21);  /* CTC mode */
  //TIMSK2 |= (1 << TOIE2); /* enable timer2 overflow interrupt */
  TIMSK2 |= (1 << OCIE2A); /* enable timer2 compare interrupt */
  //TIMSK2 |= 0b10; /* enable timer2 compare interrupt */
  //TCCR2 |= (1 << CS20);   // No prescaler 
  TCCR2B |= (1 << CS22);   // x64 prescaler 
  TIFR2 |= (1 << TOV2);    /* clear interrupt flag */
  // 800000000000000000000000000000000000000000000000000000000000000000000000000000
  set_pin_directions();
  pinMode(DEBUG_X, OUTPUT);
  ADCSRA &= ~(1<<ADEN); // Disable ADC for better power consumption (ATmega8)
                        // Might need PRR |= (1 << PRADC) 
}

// the loop function runs forever
void loop() {
}
