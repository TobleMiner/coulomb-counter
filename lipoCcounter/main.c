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
#include <string.h>
#include <avr/eeprom.h>

#include "time.h"
#include "util.h"
#include "usi_i2c_slave.h"
#include "eeprom.h"

#define ADC_SKIP_SAMPLES 2
#define ADC_SAMPLES 32ULL
#define SHUNT_RESISTANCE 1200ULL
#define CURRENT_GAIN 20ULL
#define REF_VOLTAGE 1100ULL

#define TIMER_TICK_NS 8000000UL 
#define TIMER_COUNTER_NS 64000UL
#define SEC_NSECS 1000000000UL
#define SEC_USECS 1000000UL
#define TIMER1_LIMIT 125
#define WDT_TICK_NS 125000000UL
#define MEASURE_INTERVAL_ACTIVE_NS  100000000UL // Each 100 ms
// Occasional wakeup is also important while passive since attaching of a charger must be detected
#define MEASURE_INTERVAL_PASSIVE_NS 5000000000UL // Each 5000 ms
#define LOG_INTERVAL_SEC 60UL
#define EPSILON_CHARGE_UA 1000UL
#define EPSILON_CHARGE (EPSILON_CHARGE_UA * CURRENT_GAIN * 512UL * ADC_SAMPLES / (REF_VOLTAGE * SHUNT_RESITANCE))

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
volatile int16_t adc_shunt_cal;

int64_t current_uA;
int16_t voltage_mV;

struct timeval_t past;
struct timeval_t last_measure;

int32_t uws_count;

extern struct eeprom_log_data log_data;
extern struct eeprom_device_block dev_block;


volatile struct {
	uint8_t adc_I:1;
	uint8_t adc_U:1;
	uint8_t log_loaded:1;
} flags;

#define REG64(name) \
	struct USI_I2C_Reg name##_56;\
	struct USI_I2C_Reg name##_48;\
	struct USI_I2C_Reg name##_40;\
	struct USI_I2C_Reg name##_32;\
	struct USI_I2C_Reg name##_24;\
	struct USI_I2C_Reg name##_16;\
	struct USI_I2C_Reg name##_8;\
	struct USI_I2C_Reg name##_0;

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

REG64(reg_cal);

struct USI_I2C_Reg reg_output;

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
	&reg_output,    // 23
};

enum {
	MEASURE_BIPOL_CAL = 0,
	MEASURE_CURRENT,
	MEASURE_VOLTAGE,
};

enum {
	ADC_STATE_IDLE = 0,
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
	uint8_t output_on:1;
	uint8_t charging:1;
} state;

//#define INVERT_OUTPUT

void output_on() {
	state.output_on = 1;
#ifndef INVERT_OUTPUT
	PORTB |= BIT(PINB1);
#else
	PORTB &= ~BIT(PINB1);
#endif
}

void output_off() {
	state.output_on = 0;
#ifndef INVERT_OUTPUT
	PORTB &= ~BIT(PINB1);
#else
	PORTB |= BIT(PINB1);
#endif
}

void set_output_state(uint8_t val) {
	if(val) {
		output_on();
	} else {
		output_off();
	}
	reg_output.data = val;
}

void timeval_add_nsec(struct timeval_t* t, uint64_t nsecs) {
	t->nsecs += nsecs;
	if(t->nsecs >= SEC_NSECS) {
		t->secs++;
		t->nsecs -= SEC_NSECS;
	}
}

void timeval_add_sec(struct timeval_t* t, uint64_t secs) {
	t->secs += secs;
}


void now_fast(struct timeval_t* t) {
	*t = past;
}

void now(struct timeval_t* t) {
	*t = past;
	if(state.timesource == CLOCK_TIMER) {
		timeval_add_nsec(t, timer1_cal.counter_ns * TCNT1);
	}
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
	TCCR1 = BIT(CTC1) | BIT(CS10) | BIT(CS13);
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
	state.adc = ADC_STATE_I;
	// Setup ADC for differential measurement
	ADMUX = BIT(REFS1) | BIT(MUX2) | BIT(MUX1) | BIT(MUX0);
	ADCSRA = BIT(ADEN) | BIT(ADIE) | BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2);
	ADCSRB = BIT(BIN);
}

#define DEVICE_ACTIVE (state.output_on)

uint8_t adc_process() {
	int64_t tmp;
	if(flags.adc_I) {
		flags.adc_I = 0;
		// Transmission gate prevents device from being charged/discharged
		if(!DEVICE_ACTIVE) {
			adc_shunt_cal = adcs;
			// Short path. Target device is inactive, no need to perform further measurements
			reg_adc_calh.data = (adcs >> 8) & 0xFF;
			reg_adc_call.data = adcs & 0xFF;
			adcs = 0;
			shutdown_adc();
			state.adc = ADC_STATE_IDLE;
			return 1;
		} else {
			tmp = adcs - adc_shunt_cal;
			tmp = (tmp * (int64_t)REF_VOLTAGE * 1000000LL) / (int64_t)CURRENT_GAIN / (int64_t)512LL / (int64_t)ADC_SAMPLES / (int64_t)SHUNT_RESISTANCE;
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
	return 0;
}

#define TIMER1_CAL_IN_PROGRESS (reg_cal_flags.data == CALIBRATE_TIMER1)
#define WDT_CAL_IN_PROGRESS (reg_cal_flags.data == CALIBRATE_WDT)
#define CLOCK_CAL_IN_PROGRESS (TIMER1_CAL_IN_PROGRESS || WDT_CAL_IN_PROGRESS)
#define LAST_CAL_TIMER1 (last_cal == CALIBRATE_TIMER1)
#define LAST_CAL_WDT (last_cal == CALIBRATE_WDT)
#define LAST_CAL_CLOCK (LAST_CAL_TIMER1 || LAST_CAL_WDT)

#define ADC_IDLE (state.adc == ADC_STATE_IDLE)
#define OUTPUT_ON (state.output_on)

int main(void)
{
	uint8_t i;
	
	struct timeval_t next_measure;
	next_measure.nsecs = 0;
	next_measure.secs = 0;

	struct timeval_t next_log_write;
	next_log_write.nsecs = 0;
	next_log_write.secs = 0;

	past.nsecs = 0;
	past.secs = 0;
	
	flags.adc_I = 0;
	flags.adc_U = 0;
	flags.log_loaded = 0;

	state.timesource = CLOCK_NONE;
	state.adc = ADC_STATE_IDLE;
	state.output_on = 0;
	state.charging = 0;
	
	timer1_cal.counter_ns = TIMER_COUNTER_NS;
	timer1_cal.tick_ns = TIMER_TICK_NS;
	
	wdt_cal.tick_ns = WDT_TICK_NS;
	
	struct timeval_t clock_cal_start;
	uint8_t last_cal = CALIBRATE_NONE;

	// Disable input buffer on ADC pins
	DIDR0 = BIT(ADC2D) | BIT(ADC3D);
	
	// Enable power switch IO
	DDRB |= BIT(PINB1);
	
	// Disable Timer 0 via PRR
	PRR = BIT(PRTIM0);
	
	// Initialize eeprom log writer
	eeprom_init();
	
	// Read device description block
	eeprom_read_device_block();
	// Read last log block
	if(eeprom_find_log_block()) {
		flags.log_loaded = 1;
	}
	set_output_state(log_data.flags.output_on);
	
	// Load data from log block
	uws_count = log_data.uWs;
	
	for(i = 0; i < sizeof(regs) / sizeof(*regs); i++) {
		memset(regs[i], 0, sizeof(struct USI_I2C_Reg));		
	}
	USI_I2C_Init(0x42, regs, sizeof(regs) / sizeof(*regs));

	sei();
	
	// Run calibration if output is off
	if(flags.log_loaded) {
		adc_shunt_cal = log_data.adc_shunt_cal;
	} else {
		uint8_t calibrated = 0;
		output_off();
		set_time_source(CLOCK_WDT);
		adc_start_measure();
		while (!calibrated) {
			set_sleep_mode(SLEEP_MODE_ADC);
			sleep_enable();
			sleep_cpu();
			calibrated = adc_process();
		}
		set_output_state(log_data.flags.output_on);
	}

	while (1) {
		uint8_t sleep_mode;
		struct timeval_t time;
		// Actions
		now(&time);
		
		// Update uptime
		reg_uptimeh.data = (time.secs >> 8) & 0xFF;
		reg_uptimel.data = time.secs & 0xFF;
		
		// Check if measurement is due
		if(timecmp(&time, &next_measure) != LT && ADC_IDLE) {
			adc_start_measure();
			next_measure = time;
			timeval_add_nsec(&next_measure, DEVICE_ACTIVE ? MEASURE_INTERVAL_ACTIVE_NS : MEASURE_INTERVAL_PASSIVE_NS);
		}

		// Check if we should write a log entry
		if(timecmp(&time, &next_log_write) != LT && !eeprom_busy()) {
			log_data.flags.output_on = state.output_on;
			log_data.uWs = uws_count;
			log_data.adc_shunt_cal = adc_shunt_cal;
			eeprom_write_log_block();
			next_log_write = time;
			timeval_add_sec(&next_log_write, LOG_INTERVAL_SEC);
		}
		
		// Sleep
		// USI counter overflows wake the controller up from IDLE only
		if(USI_I2C_Busy() || TIMER1_CAL_IN_PROGRESS) {
			set_time_source(CLOCK_TIMER);
			sleep_mode = SLEEP_MODE_IDLE;
		} else {
			set_time_source(CLOCK_WDT);
			sleep_mode = SLEEP_MODE_ADC;
			if(ADC_IDLE && !eeprom_busy()) {
				sleep_mode = SLEEP_MODE_PWR_DOWN;
			}
		}
		
		set_sleep_mode(sleep_mode);
		sleep_enable();
		sleep_cpu();
		
		// Process calibration requests
		ATOMIC_BEGIN
		if(CLOCK_CAL_IN_PROGRESS) {
			if(reg_cal_flags.attr.changed) {
				now(&clock_cal_start);
			}
		}
		else if (LAST_CAL_CLOCK) {
			uint64_t delta_local;
			uint64_t delta_cal;
			struct timeval_t cal_end;
			now(&cal_end);
			// Reduce resolution to us to allow for longer calibration runs
			delta_local = timedelta_ns(&clock_cal_start, &cal_end) / 1000;
			delta_cal = REG64_READ(&reg_cal) / 1000;
			if(LAST_CAL_TIMER1) {
				timer1_cal.counter_ns = (timer1_cal.counter_ns * delta_cal) / delta_local;
				timer1_cal.tick_ns = (timer1_cal.tick_ns * delta_cal) / delta_local;
			}
			else if(LAST_CAL_WDT) {
				wdt_cal.tick_ns = (wdt_cal.tick_ns * delta_cal) / delta_local;
			}
			REG64_WRITE(&reg_cal, 0ULL);
		}
		reg_cal_flags.attr.changed = 0;
		last_cal = reg_cal_flags.data;
		ATOMIC_END
		
		// Process power-up/power-down requests
		if(reg_output.data != state.output_on) {
			set_output_state(reg_output.data);
		}

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
	if(adc_cnt >= ADC_SKIP_SAMPLES) {
		if(ADC_CONVERSION_IS_BIPOLAR) {
			adcs += adc_val_bipo();
		} else {
			adc += adc_val();
		}
	}
	adc_cnt++;
	if(adc_cnt >= ADC_SKIP_SAMPLES + ADC_SAMPLES) {
		adc_cnt = 0;
		switch(state.adc) {
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

ISR(PCINT0_vect) {
	if(!DEVICE_ACTIVE) {
		// Do something with the value of SDA/SCL
	}
}