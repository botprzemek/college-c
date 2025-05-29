#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define F_CPU 16000000UL

#include <util/delay.h>

#define BUTTON_START_STOP  PD2
#define BUTTON_RESET       PD3

#define SEGMENT_DDR        DDRC
#define SEGMENT_PORT       PORTC

#define DIGIT_DDR          DDRB
#define DIGIT_PORT         PORTB

volatile uint16_t distance = 0;
volatile bool measuring = false;
volatile bool show = true;
volatile bool update_display = false;
volatile uint8_t second_counter = 0;
volatile uint8_t blink_flag = 0;

uint8_t digits[6] = {0};

const uint8_t segment_codes[10] = {
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111
};

void init_ADC() {
    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2);
}

void start_ADC() {
    ADCSRA |= (1<<ADSC);
}

void init_timer0() {
    TCCR0 = (1<<CS01)|(1<<CS00);
    TIMSK |= (1<<TOIE0);
}

void init_timer1() {
    TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);
    OCR1A = 15624;
    TIMSK |= (1<<OCIE1A);
}

void init_IO() {
    SEGMENT_DDR = 0xFF;
    SEGMENT_PORT = 0x00;
    DIGIT_DDR = 0xFF;
    DIGIT_PORT = 0x00;

    DDRD &= ~((1<<BUTTON_START_STOP)|(1<<BUTTON_RESET));
    PORTD |= (1<<BUTTON_START_STOP)|(1<<BUTTON_RESET);
}

void display_digits() {
    static uint8_t pos = 0;

    DIGIT_PORT = 0x00;
    SEGMENT_PORT = 0x00;

    if (show || measuring) {
        SEGMENT_PORT = digits[pos];
    }

    DIGIT_PORT = (1 << pos);
    pos = (pos + 1) % 6;
}

ISR(TIMER0_OVF_vect) {
    display_digits();

    static uint8_t debounce = 0;

    if (++debounce < 10) {
        return;
    }

    debounce = 0;
    
    if (!(PIND & (1 << BUTTON_START_STOP))) {
        measuring = !measuring;
        _delay_ms(200);
    }

    if (!(PIND & (1 << BUTTON_RESET))) {
        distance = 0;
        _delay_ms(200);
    }
}

ISR(TIMER1_COMPA_vect) {
    static uint8_t blink = 0;

    update_display = true;

    if (measuring) {
        start_ADC();
        return;
    }

    blink ^= 1;
    show = blink;
}

ISR(ADC_vect) {
    distance = ADC / 2;
    show = true;
}

void update_display_buffer() {
    uint16_t val = distance;

    digits[0] = segment_codes[(val / 1000) % 10];
    digits[1] = segment_codes[(val / 100) % 10];
    digits[2] = segment_codes[(val / 10) % 10];
    digits[3] = segment_codes[val % 10];

    digits[4] = 0x0 + 'c';
    digits[5] = 0x0 + 'm';
}

int main(void) {
    cli();

    init_IO();
    init_ADC();

    init_timer0();
    init_timer1();
    
    sei();

    while (1) {
        if (!update_display) {
            continue;
        }
        
        update_display = false;
        update_display_buffer();
    }
}
