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
#include <string.h>

#include "time.h"
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
#define TIMER1_LIMIT 125
#define WDT_TICK_NS 125000000UL
#define MEASURE_INTERVAL_NS 500000000UL

#define ADC_CONVERSION_IN_PROGRESS (ADCSRA & BIT(ADSC))
#define ADC_CONVERSION_IS_BIPOLAR (ADCSRB & BIT(BIN))

volatile uint16_t adc_cnt;
volatile uint16_t adc;
volatile int16_t adcs;
volatile int16_t adc_diff_cal;

int64_t current_uA;
int16_t voltage_mV;

struct timeval_t past;
struct timeval_t last_measure;

int32_t uws_count;

volatile struct {
	uint8_t adc_cal:1;
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

struct UCI_ISC_Reg reg_uptimeh;
struct UCI_ISC_Reg reg_uptimel;

struct UCI_ISC_Reg reg_adc_calh;
struct UCI_ISC_Reg reg_adc_call;

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
	&reg_mWl,
	&reg_uptimeh,
	&reg_uptimel,
	&reg_adc_calh,
	&reg_adc_call,
};

enum {
	ADC_STATE_IDLE = 0,
	ADC_STATE_CAL,
	ADC_STATE_I,
	ADC_STATE_U,
};

enum {
	CLOCK_NONE = 0,
	CLOCK_TIMER,
	CLOCK_WDT,	
};

#define ATOMIC_BEGIN do { cli();
#define ATOMIC_END   sei(); } while(0);

struct {
	uint8_t adc:2;
	uint8_t timesource:2;
} state;

void timeval_add_nsec(struct timeval_t* t, uint64_t nsecs) {
	t->nsecs += nsecs;
	if(t->nsecs >= SEC_NSECS) {
		t->secs++;
		t->nsecs -= SEC_NSECS;
	}
}


void now_fast(struct timeval_t* t) {
	*t = past;
}

void now(struct timeval_t* t) {
	*t = past;
	timeval_add_nsec(t, TIMER_COUNTER_NS * TCNT1);
}

int8_t timecmp(struct timeval_t* a, struct timeval_t* b) {
	if(a->secs > b->secs) {
		return GT;
	}
	if(a->secs < b->secs) {
		return LT;
	}
	if(a->nsecs > b->nsecs) {
		return GT;
	}
	if(a->nsecs < b->nsecs) {
		return LT;
	}
	return EQ;
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

void setup_timer1()  {
	PRR &= ~BIT(PRTIM1);
	// Set up timer (CLKDIV /512, compare match)
	TIMSK = BIT(OCIE1A);
	// One compare match interrupt each 8 ms
	OCR1A = TIMER1_LIMIT;
	TCNT1 = 0;
	TCCR1 = BIT(CTC1) | BIT(CS11) | BIT(CS13);
}

void shutdown_timer1() {
	TCCR1 = 0;
	PRR |= BIT(PRTIM1);
}

void setup_wdt() {
	// Set up watchdog timer, ~8 interrupts per second
	WDTCR = BIT(WDIE) | BIT(WDCE) | BIT(WDP1) | BIT(WDP0);	
}

void shutdown_wdt() {
	WDTCR = BIT(WDCE);
}

void set_time_source(uint8_t source) {
	if(source == state.timesource) {
		return;
	}
	ATOMIC_BEGIN
	switch(source) {
		case CLOCK_TIMER:
			shutdown_wdt();
			// No recovery of partial WDT counter overflow possible, accuracy issue?
			setup_timer1();
			break;
		case CLOCK_WDT:
			shutdown_timer1();
			now(&past);
			setup_wdt();
	}
	ATOMIC_END
	state.timesource = source;
}

void setup_adc() {
	PRR &= ~BIT(PRADC);
	ADCSRA |= BIT(ADEN);
}

void shutdown_adc() {
	ADCSRA &= ~BIT(ADEN);
	PRR |= BIT(PRADC);	
}

void adc_start_measure() {
	setup_adc();
	state.adc = ADC_STATE_CAL;
	// Setup ADC for differential offset measurement
	ADMUX  = BIT(REFS1) | BIT(MUX2) | BIT(MUX0);
	ADCSRA = BIT(ADEN) | BIT(ADIE) | BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2);
	ADCSRB = BIT(BIN);
}

void adc_process() {
	int64_t tmp;
	if(flags.adc_cal) {
		flags.adc_cal = 0;
		adc_diff_cal = adcs;
		reg_adc_calh.data = (adcs >> 8) & 0xFF;
		reg_adc_call.data = adcs & 0xFF;
		state.adc = ADC_STATE_I;
		ADMUX = BIT(REFS1) | BIT(MUX2) | BIT(MUX1) | BIT(MUX0);
	}
	if(flags.adc_I) {
		flags.adc_I = 0;
		tmp = adcs - adc_diff_cal;
		tmp = tmp * REF_VOLTAGE * SHUNT_RESITANCE / CURRENT_GAIN / 512UL / ADC_SAMPLES;
		current_uA = tmp;
		tmp /= 10;
		adcs = tmp;
		reg_Ih.data = (adcs >> 8) & 0xFF;
		reg_Il.data = adcs & 0xFF;
		adcs = 0;
		state.adc = ADC_STATE_U;
		ADMUX  = BIT(MUX3) | BIT(MUX2);
		ADCSRB = 0;
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
		shutdown_adc();
		state.adc = ADC_STATE_IDLE;
	}
}

int main(void)
{
	uint8_t i;
	
	struct timeval_t next_measure;
	next_measure.nsecs = 0;
	next_measure.secs = 0;

	past.nsecs = 0;
	past.secs = 0;

	// Disable input buffer on ADC pins
	DIDR0  = BIT(ADC2D) | BIT(ADC3D);
	
	// Disable Timer 0 via PRR
	PRR = BIT(PRTIM0);
	
	for(i = 0; i < sizeof(regs) / sizeof(*regs); i++) {
		memset(regs[i], 0, sizeof(struct UCI_ISC_Reg));		
	}
	USI_I2C_Init(0x42, regs, sizeof(regs) / sizeof(*regs));

	sei();
    while (1) {
		struct timeval_t time;
		// Actions
		now(&time);
		
		// Update uptime
		reg_uptimeh.data = (time.secs >> 8) & 0xFF;
		reg_uptimel.data = time.secs & 0xFF;
		
		// Check if measurement is due
		if(timecmp(&time, &next_measure) != LT && state.adc == ADC_STATE_IDLE) {
			adc_start_measure();
			next_measure = time;
			timeval_add_nsec(&next_measure, MEASURE_INTERVAL_NS);
		}
		
		// Sleep
		if(USI_I2C_Busy()) {
			set_time_source(CLOCK_TIMER);
			/* According to appnote we should not need this?
			ATOMIC_BEGIN
			if(state.adc != ADC_STATE_IDLE && !ADC_CONVERSION_IN_PROGRESS) {
				ADCSRA |= BIT(ADSC);
			}
			ATOMIC_END
			*/
			set_sleep_mode(SLEEP_MODE_IDLE);
		} else {
			set_time_source(CLOCK_WDT);
			// We could probably go into an even deeper sleep mode if the ADC was not running
			set_sleep_mode(SLEEP_MODE_ADC);
		}
		sleep_enable();
		sleep_cpu();
		
		// Data processing
		adc_process();
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
	if(ADC_CONVERSION_IS_BIPOLAR) {
		adcs += adc_val_bipo();
	} else {
		adc += adc_val();
	}
	adc_cnt++;
	if(adc_cnt >= ADC_SAMPLES) {
		adc_cnt = 0;
		switch(state.adc) {
			case ADC_STATE_CAL:
				flags.adc_cal = 1;
				break;
			case ADC_STATE_I:
				flags.adc_I = 1;
				break;
			case ADC_STATE_U:
				flags.adc_U = 1;
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	timeval_add_nsec(&past, TIMER_TICK_NS);
}

ISR(WDT_vect) {
	timeval_add_nsec(&past, WDT_TICK_NS);	
}
