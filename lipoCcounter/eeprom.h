#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

struct eeprom_log_data {
	int32_t mAs;
	uint32_t last_capacity_mAs;
	// Shunt can't be calibrated while output is on,
	// thus we need to save the last calibration value
	int16_t adc_shunt_cal;
	uint16_t cycle_count;
	struct {
		uint8_t output_on:1;
		struct {
			uint8_t undervoltage:1;
			uint8_t overvoltage:1;
		} battery;
	} flags;
};

struct eeprom_log_data_priv {
	uint8_t serial;
	struct eeprom_log_data data;
};

struct eeprom_log_block {
	struct eeprom_log_data_priv data;
	// Writing the data block can not be an atomic
	// operation, crc ensures integrity
	uint16_t crc;
};

struct eeprom_device_block  {
	uint16_t design_capacity_mAh;
	uint16_t design_voltage_mV;
	uint16_t min_voltage_mV;  // Lower shutdown threshold
	uint16_t max_voltage_mV;  // Upper shutdown threshold
	struct {
		uint8_t auto_poweron:1;
	} flags;
};

#define EEPROM_SIZE 4096

void eeprom_init();
void eeprom_read_device_block();
uint8_t eeprom_find_log_block();
uint8_t eeprom_busy();
void eeprom_write_log_block();


#endif