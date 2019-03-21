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

#define CUTOFF_MV 2900UL

#define TIMER_TICK_NS 8000000UL 
#define TIMER_COUNTER_NS 64000UL
#define SEC_NSECS 1000000000UL
#define SEC_USECS 1000000UL
#define TIMER1_LIMIT 125
#define WDT_TICK_NS 125000000UL
#define MEASURE_INTERVAL_NS 500000000UL

struct {
	uint64_t tick_ns;
	uint64_t counter_ns;
} timer1_cal;

struct {
	uint64_t tick_ns;
} wdt_cal;

#define ADC_CONVERSION_IN_PROGRESS (ADCSRA & BIT(ADSC))
#define ADC_CONVERSION_IS_BIPOLAR (ADCSRB & BIT(BIN))

#define DEBUG_LO (PORTB &= ~BIT(PB1))
#define DEBUG_HI (PORTB |= BIT(PB1))
#define DEBUG_BLIP do { DEBUG_HI; asm("nop\n"); DEBUG_LO; } while(0)

#define REG64_CHANGED(reg) (\
	(reg##_56)->attr.changed &&\
	(reg##_48)->attr.changed &&\
	(reg##_40)->attr.changed &&\
	(reg##_32)->attr.changed &&\
	(reg##_24)->attr.changed &&\
	(reg##_16)->attr.changed &&\
	(reg##_8)->attr.changed &&\
	(reg##_0)->attr.changed)

#define REG64_READ(reg) (\
	((uint64_t)(reg##_0)->data) |\
	(((uint64_t)(reg##_8)->data) << 8) |\
	(((uint64_t)(reg##_16)->data) << 16) |\
	(((uint64_t)(reg##_24)->data) << 24) |\
	(((uint64_t)(reg##_32)->data) << 32) |\
	(((uint64_t)(reg##_40)->data) << 40) |\
	(((uint64_t)(reg##_48)->data) << 48) |\
	(((uint64_t)(reg##_56)->data) << 56))

#define REG64_WRITE(reg, val) do {\
	(reg##_0)->data = val & 0xFF;\
	(reg##_8)->data = (val >> 8) & 0xFF;\
	(reg##_16)->data = (val >> 16) & 0xFF;\
	(reg##_24)->data = (val >> 24) & 0xFF;\
	(reg##_32)->data = (val >> 32) & 0xFF;\
	(reg##_40)->data = (val >> 40) & 0xFF;\
	(reg##_48)->data = (val >> 48) & 0xFF;\
	(reg##_56)->data = (val >> 56) & 0xFF; } while(0)

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

struct USI_I2C_Reg reg_Ih;
struct USI_I2C_Reg reg_Il;

struct USI_I2C_Reg reg_Uh;
struct USI_I2C_Reg reg_Ul;

struct USI_I2C_Reg reg_uWsh;
struct USI_I2C_Reg reg_uWsmh;
struct USI_I2C_Reg reg_uWsml;
struct USI_I2C_Reg reg_uWsl;

struct USI_I2C_Reg reg_mWh; 
struct USI_I2C_Reg reg_mWl;

struct USI_I2C_Reg reg_uptimeh;
struct USI_I2C_Reg reg_uptimel;

struct USI_I2C_Reg reg_adc_calh;
struct USI_I2C_Reg reg_adc_call;

enum {
	CALIBRATE_NONE = 0,
	// Calibration of timer1 should be done regularly since 
	// internal RC oscillator is highly temperature dependent
	CALIBRATE_TIMER1,
	// WDT calibration should be performed at start but is 
	// less critical after that since it depends mainly on
	// supply voltage
	CALIBRATE_WDT
};

struct USI_I2C_Reg reg_cal_flags;

// IMPORTANT: All registers must be written to for calibration!
struct USI_I2C_Reg reg_cal_56;
struct USI_I2C_Reg reg_cal_48;
struct USI_I2C_Reg reg_cal_40;
struct USI_I2C_Reg reg_cal_32;
struct USI_I2C_Reg reg_cal_24;
struct USI_I2C_Reg reg_cal_16;
struct USI_I2C_Reg reg_cal_8;
struct USI_I2C_Reg reg_cal_0;

struct USI_I2C_Reg* regs[] = {
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
	&reg_cal_flags, // 14
	&reg_cal_56,    // 15
	&reg_cal_48,    // 16
	&reg_cal_40,    // 17
	&reg_cal_32,    // 18
	&reg_cal_24,    // 19
	&reg_cal_16,    // 20
	&reg_cal_8,     // 21
	&reg_cal_0,     // 22
};

enum {
	MEASURE_BIPOL_CAL = 0,
	MEASURE_CURRENT,
	MEASURE_VOLTAGE,
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
	timeval_add_nsec(t, timer1_cal.counter_ns * TCNT1);
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

uint64_t timedelta_ns(struct timeval_t* pre, struct timeval_t* post) {
	struct timeval_t delta;
	timedelta(pre, post, &delta);
	return SEC_NSECS * delta.secs + delta.nsecs;
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

#define TIMER1_CAL_IN_PROGRESS (reg_cal_flags.data == CALIBRATE_TIMER1)

int main(void)
{
	uint8_t i;
	
	struct timeval_t next_measure;
	next_measure.nsecs = 0;
	next_measure.secs = 0;

	past.nsecs = 0;
	past.secs = 0;
	
	state.timesource = CLOCK_NONE;
	state.adc = ADC_STATE_IDLE;
	
	timer1_cal.counter_ns = TIMER_COUNTER_NS;
	timer1_cal.tick_ns = TIMER_TICK_NS;
	
	wdt_cal.tick_ns = WDT_TICK_NS;
	
	struct timeval_t clock_cal_start;

	// Disable input buffer on ADC pins
	DIDR0 = BIT(ADC2D) | BIT(ADC3D);
	
	// Enable debug IO
	DDRB |= BIT(PINB1);
	
	// Disable Timer 0 via PRR
	PRR = BIT(PRTIM0);
	
	for(i = 0; i < sizeof(regs) / sizeof(*regs); i++) {
		memset(regs[i], 0, sizeof(struct USI_I2C_Reg));		
	}
	USI_I2C_Init(0x42, regs, sizeof(regs) / sizeof(*regs));

	sei();
	while (1) {
		uint8_t sleep_mode;
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
		// USI counter overflows wake the controller up from IDLE only
		if(USI_I2C_Busy() || TIMER1_CAL_IN_PROGRESS) {
			set_time_source(CLOCK_TIMER);
			sleep_mode = SLEEP_MODE_IDLE;
		} else {
			set_time_source(CLOCK_WDT);
			sleep_mode = SLEEP_MODE_PWR_DOWN;
			if(state.adc == ADC_STATE_IDLE) {
				sleep_mode = SLEEP_MODE_ADC;
			}
		}
		if(sleep_mode == SLEEP_MODE_PWR_DOWN) {
			DEBUG_HI;
		}
		set_sleep_mode(sleep_mode);
		sleep_enable();
		sleep_cpu();
		DEBUG_LO;
		
		// Process calibration requests
		ATOMIC_BEGIN
		if(TIMER1_CAL_IN_PROGRESS) {
			if(reg_cal_flags.attr.changed) {
				now(&clock_cal_start);
			}
			// Ouch, this block will need a LOT of CPU time
			if(REG64_CHANGED(&reg_cal)) {
				DEBUG_BLIP;
				uint64_t delta_local;
				uint64_t delta_cal;
				struct timeval_t cal_end;
				now(&cal_end);
				reg_cal_flags.data = CALIBRATE_NONE;
				// Reduce resolution to us to allow for longer calibration runs
				delta_local = timedelta_ns(&clock_cal_start, &cal_end) / 1000;
				delta_cal = REG64_READ(&reg_cal) / 1000;
				timer1_cal.counter_ns = (timer1_cal.counter_ns * delta_cal) / delta_local;
				timer1_cal.tick_ns = (timer1_cal.tick_ns * delta_cal) / delta_local;
				REG64_WRITE(&reg_cal, 0ULL);
				DEBUG_BLIP;
			}
			reg_cal_flags.attr.changed = 0;
		}
		ATOMIC_END

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
	timeval_add_nsec(&past, timer1_cal.tick_ns);
}

ISR(WDT_vect) {
	timeval_add_nsec(&past, wdt_cal.tick_ns);	
}
