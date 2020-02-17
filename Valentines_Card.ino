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
const uint8_t MY_EXT_CLK = 13;
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

static inline void isr_display_value(uint8_t value) {
  uint8_t i;
  for (i=0b10000000; i > 0; i = i>>1) {
    DEBUG_CLK(i & value);
  }
}
/* DEBUGGING //////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/


/* LED VALUES /////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
// Requires pgm_read_byte() to get it out. Costs 1 extra cycle. Not worth it on slower CPUs.
// const uint8_t PROGMEM segment_pins[] = {0, 13, 12, 11, 1, 4, 20, 10};
// 20 is TOSC1!!!
// WHITE: 0, 1, 2 4
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
//const uint8_t led_pins[] = {0, 1, 2, 4, 16, 17, 18, 19};
const uint8_t led_pins[] = {2, 3, 19};
//const uint8_t led_pins[] = {18, 19}; //2, 4, 16, 17, 18, 19};
volatile uint8_t led_brightness[] = {0x01, 0x01, 0x01, 0, 0, 0, 0, 0};
//volatile uint8_t led_brightness[] = {0, 0};

volatile uint8_t current_led=0;
volatile uint8_t brightness=0;
// On is LOW...
#define LED_ON 0
#define LED_OFF 1

ISR(TIMER2_COMPB_vect) {
  //digitalWriteFast(OC2A_PIN, HIGH);
  //digitalWriteFast(OC2A_PIN, LOW);
  //digitalWriteFast(OC2B_PIN, HIGH);
  //digitalWriteFast(OC2B_PIN, LOW);
  //DEBUG_CLK(1);
  //DEBUG_CLK(1);
}

volatile uint8_t interrupt_count=0;
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
  //digitalWriteFast(OC2A_PIN, HIGH);
  //digitalWriteFast(OC2A_PIN, LOW);
  //digitalWriteFast(OC2A_PIN, HIGH);
  //digitalWriteFast(OC2B_PIN, LOW);
  current_led++;
  if (current_led == ARRAY_SIZE(led_pins)) current_led = 0;
  this_led_pin=led_pins[current_led];
  brightness=led_brightness[current_led];
  OCR2A=brightness;
  // compensate for the short-pulse problem with FastPWM when OCR2A==0; 
  if (brightness != 0) { digitalWriteFast(this_led_pin, LED_ON); }
  //else { digitalWriteFast(this_led, LED_ON); }
  //ovf_vect=true;
  interrupt_count++;
}

//ISR(timer2_ovf_vect) {
// COMPA (pin 11) goes high to turn the LED on, for the duration of its PWM period.
// At the end of its PWM period, the LED (-) pin should go LOW
// to thus shut off the LED.
// LED + goes to pin 11, OC2A. LED - goes to the individual LED pins.
ISR(TIMER2_COMPA_vect) {
  uint8_t next_led;
  //digitalWriteFast(11, LOW);
  digitalWriteFast(this_led_pin, LED_OFF);
  next_led=current_led+1;
  if (next_led == ARRAY_SIZE(led_pins)) next_led = 0;
  OCR2A = led_brightness[next_led]; // DOUBLE BUFFERED, SET ON BOTTOM!
  interrupt_count--;
  //compa_vect=true;
  //digitalWriteFast(OC2A_PIN, HIGH);
  //digitalWriteFast(OC2A_PIN, LOW);
  //digitalWriteFast(OC2B_PIN, HIGH);
  //digitalWriteFast(OC2B_PIN, LOW);
  //DEBUG_CLK(1);
  //DEBUG_CLK(1);
  //PORTD = 0xC0;
  /*
  // NOOP ************************
  // __asm__ __volatile__ ("nop");

  // do
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
  for (uint8_t i=0; i < ARRAY_SIZE(led_pins); i++) {
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
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    pinMode(led_pins[i], OUTPUT);
  }
  pinMode(DEBUG_X, OUTPUT);
  pinMode(DEBUG_CLK, OUTPUT);
  pinMode(OC2A_PIN, OUTPUT);
  //pinMode(OC2B_PIN, OUTPUT);
  pinMode(EXT_CLK_IN, INPUT);  // TOSC1 for external clock
  pinMode(MY_EXT_CLK, OUTPUT); // External clock source
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
}

void set_timer2_ctc(void) {
  // Set up timer 2
  TCCR2A = 0;
  TCCR2B = 0;
  // 0x7B == *approximately* 1024 times per second at 8MHz clock
  OCR2A = 0x17;  // 8 MHz clock: f = 8,000,000 / (2 * prescalar * (1 + OCR2)
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
  TCCR2B |= (1 << CS22);   // x64 prescaler 
}

void set_timer2_fastpwm(void) {
  // Note that when OCR2 == 0, there will be a spike when the TCNT2 hits BOTTOM.
  // Set up timer 2
  // 8 MHz clock: f = 8,000,000 / (prescalar * 256)
  TCCR2A = 0;
  TCCR2B = 0;
  // 0x7B == *approximately* 1024 times per second at 8MHz clock
  //OCR2A = 0x17;  // 8 MHz clock: f = 8,000,000 / (prescalar * 256)
  OCR2A = 0xFF;    // start at max.
  //OCR2B = 0xF1;
  //  One could set bits this way:
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
  //TCCR2B |= (1 << CS21);   // x8 prescaler, f=3906 Hz (measured 258 uS, == 3875 Hz)
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // x1024 prescaler, f=61 Hz
}

// the setup function runs once when you press reset or power the board
void setup() {
  //delay(100);
  uint8_t i=0;
  sei();
  set_pin_directions();
  Serial.begin(9600);
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    digitalWriteFast(led_pins[i], LED_OFF);
  }
  digitalWriteFast(OC2A_PIN, HIGH);
  for (i=0; i < ARRAY_SIZE(led_pins); i++) {
    digitalWriteFast(13, HIGH);
    digitalWriteFast(8, HIGH);
    digitalWriteFast(led_pins[i], LED_OFF);
    digitalWriteFast(led_pins[i], LED_ON);
    delay(500);
    digitalWriteFast(led_pins[i], LED_OFF);
  }
  digitalWriteFast(OC2A_PIN, LOW);
  digitalWriteFast(13, LOW);
  digitalWriteFast(8, LOW);
  //TIFR2 |= (1 << TOV2);    /* clear interrupt flag */
  // 800000000000000000000000000000000000000000000000000000000000000000000000000000
  set_timer2_fastpwm();
  ADCSRA &= ~(1<<ADEN); // Disable ADC for better power consumption (ATmega8)
                        // Might need PRR |= (1 << PRADC) 
}

uint32_t last_ms = 0;
uint8_t last_cnt = 0;
boolean loop_first_half = true;
// the loop function runs forever
void loop() {
  uint32_t current_ms = millis();
  uint32_t delta_ms = current_ms - last_ms;
  /*
  digitalWriteFast(0, LED_ON);
  digitalWriteFast(1, LED_ON);
  delay(1000);
  digitalWriteFast(0, LED_OFF);
  digitalWriteFast(1, LED_OFF);
  delay(1000);
  return;
  */
  //uint8_t test_brightness=0x00;
  //digitalWriteFast(MY_EXT_CLK, LOW);
  if (ovf_vect) {
    //Serial.print("O");
    ovf_vect = false;
  }
  /*
    Serial.print("--O ");
    Serial.print("Cnt x");
    Serial.print(TCNT2, HEX);
    Serial.print(" OCR2A x");
    Serial.print(OCR2A, HEX);
    Serial.print(" LED ");
    Serial.print(current_led, HEX);
    Serial.print(" pin ");
    Serial.println(led_pins[current_led], DEC);
    last_cnt = TCNT2;
    ovf_vect = false;
  }
  */
  if (compa_vect) {
    //Serial.println("");
    compa_vect = false;
  }
  /*
    Serial.print(" a ");
    Serial.print("Cnt x");
    Serial.print(TCNT2, HEX);
    Serial.print(" OCR2A x");
    Serial.print(OCR2A, HEX);
    Serial.print(" LED ");
    Serial.print(current_led, HEX);
    Serial.print(" pin ");
    Serial.println(led_pins[current_led], DEC);
    last_cnt = TCNT2;
    compa_vect = false;
  }
  */
    /*
    Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxx");
    if (loop_first_half) {
      led_brightness[0]=0xFF;
      led_brightness[1]=0xFF;
      led_brightness[2]=0xFF;
      loop_first_half = false;
    }
    */
  /*
  led_brightness[WHITE_1]=0xFF;
  led_brightness[WHITE_2]=0xFF;
  led_brightness[WHITE_3]=0xFF;
  led_brightness[WHITE_4]=0xFF;
  led_brightness[RED_1]=0xFF;
  led_brightness[RED_2]=0xFF;
  led_brightness[RED_3]=0xFF;
  led_brightness[RED_4]=0xFF;
  */
  //for (uint8_t i=0; i < ARRAY_SIZE(led_pins); i++) {
    //led_brightness[i]=test_brightness;
  //}
  //digitalWriteFast(MY_EXT_CLK, HIGH);
  /*
    else {
      led_brightness[0]=0xFF; // flashes!
      led_brightness[1]=0xFF;
      led_brightness[2]=0xFF;
      loop_first_half = true;
    }
    last_ms = current_ms;
  */
  /*
  if (delta_ms > 1) {
    if (last_cnt != TCNT2) {
      Serial.print("C ");
      Serial.print(TCNT2, DEC);
      Serial.print(" 2 ");
      Serial.print(OCR2A, DEC);
      Serial.print(" L ");
      Serial.println(current_led, DEC);
      last_cnt = TCNT2;
    }
  }
  */
  delay(10);
}
