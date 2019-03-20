/*
 * lipoCcounter.c
 *
 * Created: 19.03.2019 15:51:47
 * Author : Tobias
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "util.h"
#include "usi_i2c_slave.h"

#define ADC_SAMPLES 32

volatile uint16_t adc_cnt;
volatile int16_t adcs;
double dU_millivolt;

volatile struct {
	uint8_t adc:1;
} flags;

struct UCI_ISC_Reg reg0;

struct UCI_ISC_Reg* regs[1] = {
	&reg0,
};

int main(void)
{
	// Init differential ADC
	ADMUX  = BIT(REFS1) | BIT(MUX2) | BIT(MUX1) | BIT(MUX0);
	ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADIE) | BIT(ADATE) | BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2);
	ADCSRB = BIT(BIN);
	DIDR0  = BIT(ADC2D) | BIT(ADC3D);
/*
	flags.adc = 0;
	adc_cnt = 0;
	adcs = 0;
*/
	USI_I2C_Init(0x42, regs, sizeof(regs) / sizeof(*regs));
	sei();
    /* Replace with your application code */
    while (1) 
    {
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_enable();
		sleep_cpu();
		if(flags.adc) {
			flags.adc = 0;
			dU_millivolt = adcs / 20.0 * 1100.0 / 512.0 / ADC_SAMPLES;
			adcs = 0;
//			ADCSRA |= BIT(ADATE) | BIT(ADSC);
		}
    }
}

int16_t adc_val_bipo() {
	int16_t raw = ADCL | (ADCH << 8);
	if(raw & BIT(9)) {
		raw = -((~raw) & 0b111111111);
	}
	return raw;
}

ISR(ADC_vect) {
	adcs += adc_val_bipo();
	adc_cnt++;
	if(adc_cnt >= ADC_SAMPLES) {
		adc_cnt = 0;
		flags.adc = 1;	
		ADCSRA &= ~BIT(ADATE);
	}
}
