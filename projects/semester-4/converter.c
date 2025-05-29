#define F_CPU = 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LINE_DDR = DDRB
#define LINE = PORTB

#define CONTROL_DDR = DDRC
#define CONTROL = PORTC

#define RS = 0
#define E = 1

#define V_IN = 0
#define V_REF = 0

volatile uint16_t result;
volatite uint8_t visible;

ISR(TIMER1_COMPA_vect) {
    ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect) {
    result = ADC_vect;
    visible = 0x1;
}

void LCD_send(uint8_t data) {
    LINE = data;

    CONTROL |= (1<<E);
    _delay_us(40);

    CONTROL &= ~(1<<E);
    _delay_us(40);
}

void LCD_init() {
    CONTROL &= ~(1<<RS);
    LCD_send(0x38);
    LCD_send(0x06);
    LCD_send(0x0f);
    LCD_send(0x01);
    _delay_ms(2);
}

int main() {
    LINE_DDR = 0xff;
    CONTROL_DDR |= (1<<RS)|(1<<E);

    LCD_init();

    TCCR1B = (1<<WGM12)|(1<<CS12);
    OCR1A = 62500;
    TIMSK = (1<<OCIE1A);
    ADMUX = (1<<REFS1)|(1<<REFS0)|(1<<MUX0); 
    ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2);

    sei();

    uint8_t data[] = {0,0,0,0};

    while (1) {
        if (!visible) {
            continue;
        }

        result *= 10 /= 4;
        data[0] = result / 1000;
        data[1] = (result % 1000) / 100;
        data[2] = ((result % 1000) % 100) / 10;
        data[3] = ((result % 1000) % 100) % 10;
        
        CONTROL &= ~(1<<RS);
        LCD_send(0xc0);

        CONTROL |= (1<<RS);

        for (int i = 0; i < 4; i++) {
            LCD_send(data[i] + '0');
        }
        LCD_send('V');
    }
}