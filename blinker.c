#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <math.h>


#define UC_SC PB2
#define UC_MOSI PB3
#define UC_MISO PB4
#define UC_SCK PB5

unsigned char spi_transfer(unsigned char data) {

	SPDR = data;
	while (!(SPSR & (1<<SPIF)))
	{
	};
	return SPDR;

}

void dac_write(uint16_t value, uint8_t dac) {

	// DAC Variables
	uint8_t dacRegister = 0b00110000;
	uint16_t dacSecondaryByteMask = 0b0000000011111111;
	
	uint8_t dacPrimaryByte;
	uint8_t dacSecondaryByte;
	
	dacPrimaryByte = (value >> 8) | dacRegister;
	dacSecondaryByte = value & dacSecondaryByteMask;

	// DAC A
	if (dac == 0) {
		
		dacPrimaryByte &= ~(1 << 7);
		
		// DAC B
		} else {
		
		dacPrimaryByte |= (1<<7);
		
	}
	
	// Pull latch LOW
	PORTB &= ~(1 << 2);

	// Write value to DAC
	spi_transfer(dacPrimaryByte);
	spi_transfer(dacSecondaryByte);
	
	// Pull latch HIGH
	PORTB |= (1 << 2);
	
}


void adc_read_10(uint16_t *value, uint8_t channel){
	//Set Alignment
	//Sets ADLAR to 0
	ADMUX &= 0b11000000;
	
	//Sets the current multiplexer channel
	ADMUX = (0xf0 & ADMUX) | channel;
	//Starts Conversion
	ADCSRA |= (1 << ADSC);
	//Loops until ADSC bit is Clear.
	do{}while(ADCSRA & (1<<ADSC));
	
	uint8_t lowByte = ADCL;
	uint8_t highByte = ADCH;
	*value = (highByte << 8) | lowByte;

	return;
}

typedef struct 
{
    uint32_t counter;
} metro;

void metro_init(metro *m) 
{
    m->counter = 0;
}

int main(void) {

    DDRD |= (1 << 1);
    DDRD |= (1 << 2);
    
    uint16_t val = 0;
    int i;
    uint32_t t = 0;
    uint32_t x = 1;


	ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADPS2) | ( 1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);	

    /* init SPI */


    DDRB = 0b100;
    DDRB |= (1 << UC_MOSI) | (1 <<UC_SCK) | (1 << UC_SC);
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = (1 << SPI2X);

    while(1) {
        adc_read_10(&x, 1);

        //x %= 0xff;       
 
        val = t*(((t>>12)|(t>>8))&(x&(t>>4)));

        val %= 0xfff;


        dac_write(val, 0);

        if(val > 0xff) {         
            PORTD |= (1 << 1);
            PORTD |= (1 << 2);
        } else {
            PORTD &= ~(1 << 1);
            PORTD &= ~(1 << 2);
        }

        t = (t + 1) % 0xffffffff;
        _delay_ms(4);

    }
    return 0;
}
