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

#define ADC_SAMPLES 32UL
#define SHUNT_RESITANCE 950UL
#define CURRENT_GAIN 20UL
#define REF_VOLTAGE 1100UL

#define TIMER_TICK_NS 8000000UL 
#define TIMER_COUNTER_NS 64000UL
#define SEC_NSECS 1000000000UL
#define SEC_USECS 1000000UL


volatile uint16_t adc_cnt;
volatile uint16_t adc;
volatile int16_t adcs;

int64_t current_uA;
int16_t voltage_mV;

struct timeval_t {
	uint64_t secs;
	uint64_t nsecs;
};

struct timeval_t past;
struct timeval_t last_measure;

int32_t uws_count;

volatile struct {
	uint8_t adc_I:1;
	uint8_t adc_U:1;
} flags;

struct UCI_ISC_Reg reg_Ih;
struct UCI_ISC_Reg reg_Il;

struct UCI_ISC_Reg reg_Uh;
struct UCI_ISC_Reg reg_Ul;

struct UCI_ISC_Reg reg_uWsh;
struct UCI_ISC_Reg reg_uWsmh;
struct UCI_ISC_Reg reg_uWsml;
struct UCI_ISC_Reg reg_uWsl;

struct UCI_ISC_Reg reg_mWh; 
struct UCI_ISC_Reg reg_mWl;

struct UCI_ISC_Reg* regs[] = {
	&reg_Ih,
	&reg_Il,
	&reg_Uh,
	&reg_Ul,
	&reg_uWsh,
	&reg_uWsmh,
	&reg_uWsml,
	&reg_uWsl,
	&reg_mWh,
	&reg_mWl
};

enum {
	ADC_STATE_I = 0,
	ADC_STATE_U,
};

uint8_t adc_state = 0;

void now(struct timeval_t* t) {
	*t = past;
	t->nsecs += TIMER_COUNTER_NS * TCNT1;
	if(t->nsecs >= SEC_NSECS) {
		t->secs++;
		t->nsecs -= SEC_NSECS;
	}
}

void timedelta(struct timeval_t* pre, struct timeval_t* post, struct timeval_t* delta) {
	delta->secs = post->secs - pre->secs;
	if(pre->nsecs > post->nsecs) {
		delta->secs--;
		delta->nsecs = SEC_NSECS - (pre->nsecs - post->nsecs);
	} else {
		delta->nsecs = post->nsecs - pre->nsecs;		
	}
}

void update_uwh_count() {
	int32_t power_uW;
	int32_t power_mW;
	struct timeval_t current;
	struct timeval_t delta;
	now(&current);
	timedelta(&last_measure, &current, &delta);
	power_uW = voltage_mV * current_uA / 1000L;
	power_mW = power_uW / (int32_t)1000L;
	reg_mWh.data = (power_mW >> 8) & 0xFF;
	reg_mWl.data = power_mW & 0xFF;

	uws_count += ((int64_t)delta.secs) * power_uW;
	uws_count += ((int64_t)delta.nsecs) * power_mW / ((int64_t)SEC_USECS);
	reg_uWsh.data = (uws_count >> 24) & 0xFF;
	reg_uWsmh.data = (uws_count >> 16) & 0xFF;
	reg_uWsml.data = (uws_count >> 8) & 0xFF;
	reg_uWsl.data = (uws_count >> 0) & 0xFF;
	last_measure = current;
}

int main(void)
{
	// Init differential ADC
	ADMUX  = BIT(REFS1) | BIT(MUX2) | BIT(MUX1) | BIT(MUX0);
	ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADIE) | BIT(ADATE) | BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2);
	ADCSRB = BIT(BIN);
	DIDR0  = BIT(ADC2D) | BIT(ADC3D);
	
	// Set up timer (CLKDIV /512, compare match)
	TCCR1 = BIT(CTC1) | BIT(CS11) | BIT(CS13);
	TIMSK = BIT(OCIE1A);
	// One compare match interrupt each 8 ms
	OCR1A = 125;
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
		int64_t tmp;
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_enable();
		sleep_cpu();
		if(flags.adc_I) {
			flags.adc_I = 0;
			tmp = adcs;
			tmp = tmp * REF_VOLTAGE * SHUNT_RESITANCE / CURRENT_GAIN / 512UL / ADC_SAMPLES;
			current_uA = tmp;
			tmp /= 10;
			adcs = tmp;
			reg_Ih.data = (adcs >> 8) & 0xFF;
			reg_Il.data = adcs & 0xFF;
//			dU_millivolt = adcs / 20.0 * 1100.0 / 512.0 / ADC_SAMPLES;
			adcs = 0;
			adc_state = ADC_STATE_U;
			ADMUX  = BIT(MUX3) | BIT(MUX2);
			ADCSRB = 0;
			ADCSRA |= BIT(ADATE) | BIT(ADSC);
		}
		if(flags.adc_U) {
			flags.adc_U = 0;
			tmp = adc;
			tmp = REF_VOLTAGE * 1024UL * ADC_SAMPLES / tmp;
			voltage_mV = tmp;
			adc = tmp;
			reg_Uh.data = (adc >> 8) & 0xFF;
			reg_Ul.data = adc & 0xFF;
			adc = 0;
			update_uwh_count();
			_delay_ms(500);
			adc_state = ADC_STATE_I;
			ADMUX  = BIT(REFS1) | BIT(MUX2) | BIT(MUX1) | BIT(MUX0);
			ADCSRB = BIT(BIN);
			ADCSRA |= BIT(ADATE) | BIT(ADSC);
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

uint16_t adc_val() {
	return ADCL | (ADCH << 8);
}

ISR(ADC_vect) {
	switch(adc_state) {
		case ADC_STATE_I:
			adcs += adc_val_bipo();
			adc_cnt++;
			if(adc_cnt >= ADC_SAMPLES) {
				adc_cnt = 0;
				flags.adc_I = 1;
				ADCSRA &= ~BIT(ADATE);
			}
			break;
		case ADC_STATE_U:
			adc += adc_val();
			adc_cnt++;
			if(adc_cnt >= ADC_SAMPLES) {
				adc_cnt = 0;
				flags.adc_U = 1;
				ADCSRA &= ~BIT(ADATE);
			}
			break;
	}
}

ISR(TIMER1_COMPA_vect) {
	past.nsecs += TIMER_TICK_NS;
	if(past.nsecs >= SEC_NSECS) {
		past.secs++;
		past.nsecs -= SEC_NSECS;
	}
}
