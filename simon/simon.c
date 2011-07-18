// Simon clone
// nick.lewis@gmail.com

// no touchy
#define F_CPU 14745600

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "../libnerdkits/delay.h"
#include "../libnerdkits/lcd.h"

// 74HC164 stuff
#define LED_RESET PC0
#define LED_DATA PC1
#define LED_CLOCK PC2

// SRCLK high shifts register by one
#define SRCLK PC3

// RCLK high latches register to output pins
#define RCLK PC4

// DATA is input on 7HC595
#define DATA PC5

// buttons
#define BUTTON_WHITE PB1
#define BUTTON_YELLOW PB2
#define BUTTON_GREEN PB3
#define BUTTON_RED PB4

// speaker
#define SPEAKER PB5

// notes for piezo test
// Frequencies from http://www.phy.mtu.edu/~suits/notefreqs.html
// converted to half-periods (us) by calculating
// 1000000/2/frequency
// where frequency is in Hz
#define D5 851
#define E5 758
#define G5 637
#define A5 568
#define B5 506
#define C6 477
#define D6 425
#define G2 5102

// LEDs
#define white   0b00001000
#define yellow  0b00000100
#define green   0b00000010
#define red     0b00000001
#define all     0b00001111
#define blank   0b00000000


void init() {

  // every pin on port C used as output
  DDRC = 0xff;

  // buttons as input, left to right on breadboard
  DDRB &= ~(1<<BUTTON_WHITE);
  DDRB &= ~(1<<BUTTON_YELLOW);
  DDRB &= ~(1<<BUTTON_GREEN);
  DDRB &= ~(1<<BUTTON_RED);

  // use speaker as a... speaker
  DDRB |= (1<<SPEAKER);

  // enable pull up resistors for buttons
  PORTB |= (1<<BUTTON_WHITE);
  PORTB |= (1<<BUTTON_YELLOW);
  PORTB |= (1<<BUTTON_GREEN);
  PORTB |= (1<<BUTTON_RED);

  // OK, initialize shift registers
  // low is ready to shift
  PORTC &= ~(1<<SRCLK);

  // nothing to output
  PORTC &= ~(1<<RCLK);

  // LED reset is active low
  PORTC |= (1<<LED_RESET);

  // LED clock active high
  PORTC &= ~(1<<LED_CLOCK);

  lcd_init();
  lcd_clear_and_home();

}

void play_tone(uint16_t note_delay, uint8_t duration) {
  uint16_t delaysm = note_delay / 50;
  uint16_t cycles = (100 * duration) / delaysm;

  while(cycles > 0) {
    PORTB |= (1<<SPEAKER);
    delay_us(note_delay);
    PORTB &= ~(1<<SPEAKER);
    delay_us(note_delay);
    cycles--;
  }
}

void light_led(unsigned char bits) {

  uint8_t i;
  for(i=0; i<=7; i++) {
    if (bits & (1<<i)) {
      PORTC |= (1<<LED_DATA);
    }
    else {
      PORTC &= ~(1<<LED_DATA);
    }
    PORTC |= (1<<LED_CLOCK);
    PORTC &= ~(1<<LED_CLOCK);
  }

}

void clear_leds() {
  PORTC &= ~(1<<LED_RESET);
  PORTC |= (1<<LED_RESET);
}

void display_number(unsigned char bits) {

  uint8_t i;
  for(i=0;i<=7;i++) {
    if (bits & (1<<i)) {
      PORTC |= (1<<DATA);
    }
    else {
      PORTC &= ~(1<<DATA);
    }

    PORTC |= (1<<SRCLK);
    PORTC &= ~(1<<SRCLK);

  }
  PORTC |= (1<<RCLK);
  PORTC &= ~(1<<RCLK);

}

void do_white() {
  light_led(white);
  int i;
  for(i=0;i<5;i++) {
    play_tone(D5,10);
  }
  clear_leds();
}

void do_yellow() {
  light_led(yellow);
  int i;
  for(i=0;i<5;i++) {
    play_tone(E5,10);
  }
  clear_leds();
}

void do_green() {
  light_led(green);
  int i;
  for(i=0;i<5;i++) {
    play_tone(A5,10);
  }
  clear_leds();
}

void do_red() {
  light_led(red);
  int i;
  for(i=0;i<5;i++) {
    play_tone(B5,10);
  }
  clear_leds();
}

int main() {

  init();

  // MSB is segment A
  // LSB unused, represents decimal point
  unsigned char zero  = 0b11111100;
  unsigned char one   = 0b01100000;
  unsigned char two   = 0b11011010;
  unsigned char three = 0b11110010;
  unsigned char four  = 0b01100110;
  unsigned char five  = 0b10110110;
  unsigned char six   = 0b10111110;
  unsigned char seven = 0b11100000;
  unsigned char eight = 0b11111110;
  unsigned char nine  = 0b11100110;

  uint8_t numbers[] = {zero,one,two,three,four,five,six,seven,eight,nine};
  static uint8_t sequence[99];

  lcd_line_one();

  int i;
  for(i=0;i<100;i++) {
    sequence[i]=rand() % 4;
  }

  uint8_t turn = 0;

  while(1) {
    int ones = turn % 10;
    int tens = (turn / 10) % 10;
    display_number(numbers[tens]);
    display_number(numbers[ones]);

    // play current sequence up to current turn
    uint8_t i;
    for(i=0;i<=turn;i++) {
      lcd_line_two();
      lcd_write_int16(i);
      delay_ms(200);
      uint8_t this_turn = sequence[i];
      lcd_line_four();
      lcd_write_int16(this_turn);
      if (this_turn == 0) {
        do_white();
      }
      else if (this_turn == 1) {
        do_yellow();
      }
      else if (this_turn == 2) {
        do_green();
      }
      else if (this_turn == 3) {
        do_red();
      }
    }

    // read inputs, compare with sequence
    uint8_t guess;
    uint8_t any_pressed,white_pressed,yellow_pressed,green_pressed,red_pressed;

    for(i=0;i<=turn;i++) {

      any_pressed = 0;

      while(!any_pressed) {
        white_pressed = !((PINB & (1<<BUTTON_WHITE)) >> BUTTON_WHITE);
        yellow_pressed = !((PINB & (1<<BUTTON_YELLOW)) >> BUTTON_YELLOW);
        green_pressed = !((PINB & (1<<BUTTON_GREEN)) >> BUTTON_GREEN);
        red_pressed = !((PINB & (1<<BUTTON_RED)) >> BUTTON_RED);

        any_pressed = white_pressed || yellow_pressed || green_pressed || red_pressed;
      }

      if (white_pressed) {
        guess = 0;
        do_white();
      }
      else if (yellow_pressed) {
        guess = 1;
        do_yellow();
      }
      else if (green_pressed) {
        guess = 2;
        do_green();
      }
      else if (red_pressed) {
        guess = 3;
        do_red();
      }

      // wrong answer, play a fart sound and blink all LEDs
      // TODO: restart game, maybe add a 'new game' button?
      // TODO: save high score in NVRAM
      if(sequence[i] != guess) {
        int i;
        int j;
        for(i=0; i<4; i++) {
          light_led(all);
          for(j=0;j<5;j++) {
            play_tone(G2,10);
          }
          clear_leds();
          for(j=0;j<5;j++) {
            play_tone(G2,10);
          }
        }
        delay_ms(10000);
      }
    }
    delay_ms(150);
    turn++;

    lcd_line_three();
    lcd_write_int16(turn);
    light_led(blank);

  }

  return 0;

}

