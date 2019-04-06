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
#include <avr/wdt.h>

#include "time.h"
#include "util.h"
#include "usi_i2c_slave.h"
#include "debug.h"

#define ADC_SKIP_SAMPLES 2
#define ADC_SAMPLES 32ULL
#define SHUNT_RESISTANCE 500ULL
#define CURRENT_GAIN 10ULL
#define REF_VOLTAGE 1100ULL

#define BATTERY_MIN_VOLTAG_MV 2900
#define BATTERY_MAX_VOLTAG_MV 4200

#define BATTERY_STATE_SAMPLES 8

#define TIMER1_LIMIT 100

// Time interval definitions
#define SEC_PSECS                1000000000000ULL
#define SEC_NSECS                   1000000000ULL
#define SEC_USECS                      1000000ULL
#define DEBOUNCE_NS                   10000000ULL      // 10ms between button presses
#define TIMER_TICK_NS                  3200000ULL      // 3.2 ms per interrupt
#define TIMER_COUNTER_NS                 32000ULL      // 32 us per counter tick
#define WDT_TICK_NS                   16000000ULL      // Each 16 ms
#define MEASURE_INTERVAL_NS          100000000ULL      // Each 100 ms
#define LOG_INTERVAL_SEC 60UL

#define EPSILON_CHARGE_UA 1000UL
#define HYSTERESIS_CHARGE_UA 300UL

#define HYSTERESIS_CHARGE_MV 50UL

#define POWER_REQUEST_TIMEOUT_NS (BATTERY_STATE_SAMPLES * MEASURE_INTERVAL_NS * 2)

const int64_t current_div = (int64_t)CURRENT_GAIN * 512LL * (int64_t)ADC_SAMPLES * (int64_t)SHUNT_RESISTANCE;

#define OUTPUT_DDR  DDRA
#define OUTPUT_PORT PORTA
#define OUTPUT_PIN  PINA7

#define CHARGE_SENSE_DDR   DDRB
#define CHARGE_SENSE_PORT  PORTB
#define CHARGE_SENSE_PIN   PINB2
#define CHARGE_SENSE_PINR  PINB
#define CHARGE_SENSE_PCBIT PCINT3
#define CHARGE_SENSE_PCREG PCMSK0
#define CHARGE_SENSE_PCIE  PCIE0

#define POWER_REQUEST_DDR   DDRA
#define POWER_REQUEST_PORT  PORTA
#define POWER_REQUEST_PIN   PINA3
#define POWER_REQUEST_PINR  PINA
#define POWER_REQUEST_PCBIT PCINT10
#define POWER_REQUEST_PCREG PCMSK1
#define POWER_REQUEST_PCIE  PCIE1


//#define INVERT_OUTPUT
//#define INVERT_CHARGE

struct timeval_t power_request_debounce;

#define REG32_WRITE(reg, val) do {\
	(reg##_0).data = val & 0xFF;\
	(reg##_8).data = (val >> 8) & 0xFF;\
	(reg##_16).data = (val >> 16) & 0xFF;\
	(reg##_24).data = (val >> 24) & 0xFF; } while(0)

#define REG16_WRITE(reg, val) do {\
	(reg##_l).data = val & 0xFF;\
	(reg##_h).data = (val >> 8) & 0xFF; } while(0)

volatile uint16_t adc_cnt;
volatile uint16_t adc;
volatile int16_t adcs;
volatile int16_t adc_diff_cal;
volatile int16_t adc_shunt_cal;

int64_t current_uA;
int64_t current_uA_avg;
int16_t voltage_mV;

struct timeval_t past;
struct timeval_t last_measure;

volatile struct {
	uint8_t adc_I:1;
	uint8_t adc_U:1;
	uint8_t power_request:1;
	uint8_t do_calibrate:1;
} flags;

struct battery_flag {
	uint8_t value:1;
	uint8_t set:1;
	uint8_t clear:1;
	uint8_t counter:5;
};

struct {
	struct battery_flag overvoltage;
	struct battery_flag undervoltage;
	struct battery_flag charging;
} battery;

#define REG64(name) \
	struct USI_I2C_Reg name##_56 = {{ 0 }};\
	struct USI_I2C_Reg name##_48 = {{ 0 }};\
	struct USI_I2C_Reg name##_40 = {{ 0 }};\
	struct USI_I2C_Reg name##_32 = {{ 0 }};\
	struct USI_I2C_Reg name##_24 = {{ 0 }};\
	struct USI_I2C_Reg name##_16 = {{ 0 }};\
	struct USI_I2C_Reg name##_8 = {{ 0 }};\
	struct USI_I2C_Reg name##_0 = {{ 0 }};

#define REG32(name) \
	struct USI_I2C_Reg name##_24 = {{ 0 }};\
	struct USI_I2C_Reg name##_16 = {{ 0 }};\
	struct USI_I2C_Reg name##_8 = {{ 0 }};\
	struct USI_I2C_Reg name##_0 = {{ 0 }};

#define REG16(name) \
	struct USI_I2C_Reg name##_h = {{ 0 }};\
	struct USI_I2C_Reg name##_l = {{ 0 }};

REG16(reg_I);
REG16(reg_U);
REG16(reg_cal);
REG16(reg_I_avg);
REG16(reg_uptime);
struct USI_I2C_Reg reg_flags = {
	.write_mask = 1,
};

struct USI_I2C_Reg* regs[] = {
	&reg_I_h,
	&reg_I_l,
	&reg_U_h,
	&reg_U_l,
	&reg_cal_h,
	&reg_cal_l,
	&reg_I_avg_h,
	&reg_I_avg_l,
	&reg_uptime_h,
	&reg_uptime_l,
	&reg_flags
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

enum {
	POWER_REQUEST_IDLE,
	POWER_REQUEST_MEASURE
};

struct {
	uint8_t adc:2;
	uint8_t timesource:2;
	uint8_t output_on:1;
	uint8_t charge_on:1;
	uint8_t charger_detect:1;
	uint8_t power_request_state:1;
	uint8_t power_requested:1;
} state;

#define ADC_CONVERSION_IN_PROGRESS (ADCSRA & BIT(ADSC))
#define ADC_CONVERSION_IS_BIPOLAR (state.adc == ADC_STATE_I)

void output_on() {
	reg_flags.data |= 1;
	state.output_on = 1;
#ifndef INVERT_OUTPUT
	OUTPUT_PORT |= BIT(OUTPUT_PIN);
#else
	OUTPUT_PORT &= ~BIT(OUTPUT_PIN);
#endif
}

void output_off() {
	reg_flags.data &= ~1;
	state.output_on = 0;
#ifndef INVERT_OUTPUT
	OUTPUT_PORT &= ~BIT(OUTPUT_PIN);
#else
	OUTPUT_PORT |= BIT(OUTPUT_PIN);
#endif
}

void set_output_state(uint8_t val) {
	if(val) {
		output_on();
	} else {
		output_off();
	}
}


void timeval_add_nsec(struct timeval_t* t, uint64_t nsecs) {
	t->nsecs += nsecs;
	while(t->nsecs >= SEC_NSECS) {
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
		timeval_add_nsec(t, TIMER_COUNTER_NS * TCNT1L);
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

void setup_timer1()  {
	PRR &= ~BIT(PRTIM1);
	// Set up timer (CLKDIV /16, compare match)
	TIMSK1 = BIT(OCIE1A);
	// One compare match interrupt each 8 ms
	OCR1AL = TIMER1_LIMIT;
	OCR1AH = 0;
	TCNT1L = 0;
	TCNT1H = 0;
	TCCR1B = BIT(WGM12) | BIT(CS12);
}

void shutdown_timer1() {
	TCCR1B = 0;
	PRR |= BIT(PRTIM1);
}

#define POWER_ACTIVE (state.output_on)

void setup_wdt() {
	// Set up watchdog timer, ~1 interrupt per second
	uint8_t	prescaler_bits = BIT(WDIE);
	MCUSR &= ~BIT(WDRF);
	WDTCSR = BIT(WDCE) | BIT(WDE);
	WDTCSR = prescaler_bits;
}

void shutdown_wdt() {
	MCUSR &= ~BIT(WDRF);
	WDTCSR = BIT(WDCE) | BIT(WDE);
	WDTCSR = 0;
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
			break;
		case CLOCK_NONE:
			shutdown_timer1();
			shutdown_wdt();
	}
	ATOMIC_END
	state.timesource = source;
}

void setup_adc() {
	PRR &= ~BIT(PRADC);
	ADCSRA |= BIT(ADEN);
}

void shutdown_adc() {
	state.adc = ADC_STATE_IDLE;
	ADCSRA &= ~BIT(ADEN);
	PRR |= BIT(PRADC);
}

void adc_start_measure() {
	setup_adc();
	state.adc = ADC_STATE_I;
	adcs = 0;
	adc = 0;
	// Setup ADC for differential measurement
	ADMUX = BIT(REFS1) | BIT(MUX3) | BIT(MUX0);
	ADCSRA = BIT(ADEN) | BIT(ADIE);
	ADCSRB = BIT(BIN);
}

#define RESET_FLAG_COUNTER(flag) ((flag).counter = 0)

#define GET_FLAG(flag) ((flag).value)

#define PROCESS_FLAG(flag, val, limit, hysteresis, invert) \
	do {\
		if(((invert) ? (val) + (hysteresis) < (limit) : (val) > (limit) + (hysteresis)) && !(flag).value) {\
			if((flag).clear) {\
				(flag).clear = 0;\
				(flag).counter = 0;\
			}\
			(flag).set = 1;\
			if((flag).counter >= BATTERY_STATE_SAMPLES) {\
				(flag).value = 1;\
			} else {\
				(flag).counter++;\
			}\
		}\
		if(((invert) ? (val) > (limit) + (hysteresis) : (val) + (hysteresis) < (limit)) && (flag).value) {\
			if((flag).set) {\
				(flag).set = 0;\
				(flag).counter = 0;\
			}\
			(flag).clear = 1;\
			if((flag).counter >= BATTERY_STATE_SAMPLES) {\
				(flag).value = 0;\
			} else {\
				(flag).counter++;\
			}\
		}\
	} while(0)

#define PROCESS_FLAG_UPPER(flag, val, limit, hysteresis)\
	PROCESS_FLAG(flag, val, limit, hysteresis, 0)

#define PROCESS_FLAG_LOWER(flag, val, limit, hysteresis)\
	PROCESS_FLAG(flag, val, limit, hysteresis, 1)
	
uint8_t adc_process() {
	int64_t tmp;
	if(flags.adc_I) {
		flags.adc_I = 0;
		if(flags.do_calibrate) {
			adc_shunt_cal = adcs;
			REG16_WRITE(reg_cal, adcs);
			shutdown_adc();
			return 1;
		} else {
			tmp = adcs - adc_shunt_cal;
			tmp = (tmp * (int64_t)REF_VOLTAGE * 1000000LL) / current_div;
			current_uA = tmp;
			adcs = tmp / 10LL;
			REG16_WRITE(reg_I, adcs);
			current_uA_avg = ((current_uA_avg * 900LL) + (current_uA * 100LL)) / 1000LL;
			adcs = current_uA_avg / 10LL;
			REG16_WRITE(reg_I_avg, adcs);
			
			// Process current flags
			PROCESS_FLAG_UPPER(battery.charging, current_uA, EPSILON_CHARGE_UA, HYSTERESIS_CHARGE_UA);
		}
		adcs = 0;
		state.adc = ADC_STATE_U;
		ADMUX  = BIT(MUX5) | BIT(MUX0);
		ADCSRB = 0;
	}
	if(flags.adc_U) {
		flags.adc_U = 0;
		// Calculate voltage
		tmp = adc;
		tmp = REF_VOLTAGE * 1024UL * ADC_SAMPLES / tmp;
		voltage_mV = tmp;
		adc = tmp;
		reg_U_h.data = (adc >> 8) & 0xFF;
		reg_U_l.data = adc & 0xFF;
		adc = 0;

		// Process voltage flags
		PROCESS_FLAG_LOWER(battery.undervoltage, voltage_mV, BATTERY_MIN_VOLTAG_MV, HYSTERESIS_CHARGE_MV);

		shutdown_adc();
	}
	return 0;
}

#define POWER_REQUESTED (state.power_requested)
#define POWER_REQUEST_PENDING (state.power_request_state == POWER_REQUEST_MEASURE)
#define CHARGER_DETECTED (state.charger_detect)

struct timeval_t power_request_timeout;

void digital_io_process(struct timeval_t* time) {
	state.charger_detect = !!(CHARGE_SENSE_PINR & BIT(CHARGE_SENSE_PIN));
	state.power_requested = ((~POWER_REQUEST_PINR) & BIT(POWER_REQUEST_PIN)) >> POWER_REQUEST_PIN;
		
	ATOMIC_BEGIN
	if(flags.power_request) {
		if(!POWER_REQUEST_PENDING) {
			state.power_request_state = POWER_REQUEST_MEASURE;
			power_request_timeout = *time;
			timeval_add_nsec(&power_request_timeout, POWER_REQUEST_TIMEOUT_NS);
		}
		power_request_debounce = *time;
		timeval_add_nsec(&power_request_debounce, DEBOUNCE_NS);
	}
	flags.power_request = 0;
	ATOMIC_END

	if(!state.power_requested) {
		if(timecmp(time, &power_request_debounce) != LT) {
			state.power_request_state = POWER_REQUEST_IDLE;
			output_off();
		}
	}
	
	if(CHARGER_DETECTED) {
		POWER_REQUEST_PORT |= BIT(POWER_REQUEST_PIN);		
	}
}

struct {
	uint8_t undervoltage:2;
} last_bat_state = {
	.undervoltage = 3,
};

void battery_process() {
	if(GET_FLAG(battery.undervoltage) && !CHARGER_DETECTED) {
		POWER_REQUEST_PORT &= ~BIT(POWER_REQUEST_PIN);
		output_off();
	}
	
	if(GET_FLAG(battery.undervoltage) != last_bat_state.undervoltage) {
		if(!GET_FLAG(battery.undervoltage) && state.power_requested) {
			output_on();
		}
	}
	last_bat_state.undervoltage = GET_FLAG(battery.undervoltage);
}

#define ADC_IDLE (state.adc == ADC_STATE_IDLE)
#define OUTPUT_ON (state.output_on)
#define REG_OUTPUT_ON (reg_flags.data & 1)

struct timeval_t power_request_timeout;

int main(void)
{
	uint8_t i;
	
	struct timeval_t next_measure;
	next_measure.nsecs = 0;
	next_measure.secs = 0;

	past.nsecs = 0;
	past.secs = 0;
	
	memset((void*)&flags, 0, sizeof(flags));

	memset(&battery, 0, sizeof(battery));

	memset(&state, 0, sizeof(state));
		
	// Disable input buffer on ADC pins
	DIDR0 = BIT(ADC2D) | BIT(ADC1D);
	
	// Enable power switch IO
	OUTPUT_DDR |= BIT(OUTPUT_PIN);
	
	// Enable pullup on power request input
	POWER_REQUEST_PORT |= BIT(POWER_REQUEST_PIN);
	
	// Enable pin change interrupt on charger sense pin
	CHARGE_SENSE_PCREG |= BIT(CHARGE_SENSE_PCBIT);
	GIMSK |= BIT(CHARGE_SENSE_PCIE);

	// Enable pin change interrupt on power request pin
	POWER_REQUEST_PCREG |= BIT(POWER_REQUEST_PCBIT);
	GIMSK |= BIT(POWER_REQUEST_PCIE);

	// Disable Timer 0 via PRR
	PRR = BIT(PRTIM0);

	for(i = 0; i < sizeof(regs) / sizeof(*regs); i++) {
		memset(regs[i], 0, sizeof(struct USI_I2C_Reg));
	}
	USI_I2C_Init(0x42, regs, sizeof(regs) / sizeof(*regs));

	sei();
	
	// Debugging
	DEBUG_INIT(PB0);
	DEBUG_INIT(PB1);
	
	// Run calibration
	flags.do_calibrate = 1;
	output_off();
	set_time_source(CLOCK_WDT);
	adc_start_measure();
	while (flags.do_calibrate) {
		set_sleep_mode(SLEEP_MODE_ADC);
		sleep_enable();
		sleep_cpu();
		flags.do_calibrate = !adc_process();
	}

	// Process digital inputs
	digital_io_process(&past);

	// Battery status processing
	battery_process();

	while (1) {
		uint8_t sleep_mode;
		struct timeval_t time;
		// Actions
		now(&time);
		
		// Update uptime
		REG16_WRITE(reg_uptime, time.secs);
		
		// Check if measurement is due
		if(timecmp(&time, &next_measure) != LT && ADC_IDLE) {
			adc_start_measure();
			next_measure = time;
			timeval_add_nsec(&next_measure, MEASURE_INTERVAL_NS);
		}

		if(POWER_REQUEST_PENDING) {
			// Check if power request timeout elapsed
			if(timecmp(&time, &power_request_timeout) != LT) {
				state.power_request_state = POWER_REQUEST_IDLE;
				if(!GET_FLAG(battery.undervoltage)) {
					output_on();
				}
			}
		}
		
		// Sleep
		// TWI data wakes the controller up from IDLE only
		if(USI_I2C_Busy()) {
			set_time_source(CLOCK_TIMER);
			sleep_mode = SLEEP_MODE_IDLE;
		} else {
			if(OUTPUT_ON || POWER_REQUEST_PENDING || CHARGER_DETECTED) {
				set_time_source(CLOCK_WDT);				
			} else {
				set_time_source(CLOCK_NONE);
				shutdown_adc();
			}
			sleep_mode = SLEEP_MODE_PWR_DOWN;
			if(!ADC_IDLE) {
				sleep_mode = SLEEP_MODE_ADC;
			}
		}
		
		set_sleep_mode(sleep_mode);
		sleep_enable();
		sleep_cpu();

		now(&time);

		// Data processing
		adc_process();

		// Process digital inputs
		digital_io_process(&time);
	
		// Process power-up/power-down requests
		if(REG_OUTPUT_ON != OUTPUT_ON) {
			set_output_state(REG_OUTPUT_ON);
			// Reset measurement time interval monitor
			if(state.output_on) {
				shutdown_adc();
				now(&last_measure);
				// Force a measurement
				next_measure = last_measure;
				// Clean slate, run full undervoltage detection cycle
				RESET_FLAG_COUNTER(battery.undervoltage);
			}
		}
				
		// Battery status processing
		battery_process();
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

ISR(TIM1_COMPA_vect) {
	DEBUG_BLIP(PB0);
	timeval_add_nsec(&past, TIMER_TICK_NS);
}

ISR(WDT_vect) {
	DEBUG_BLIP(PB0);
	timeval_add_nsec(&past, WDT_TICK_NS);
}

ISR(PCINT0_vect) {
	DEBUG_BLIP(PB1);
	flags.power_request = ((~POWER_REQUEST_PINR) & BIT(POWER_REQUEST_PIN)) >> POWER_REQUEST_PIN;
}

ISR(PCINT1_vect) {
	// NOP, just wake up
	DEBUG_BLIP(PB1);
}
