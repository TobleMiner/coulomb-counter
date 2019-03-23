#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

struct eeprom_log_data {
	uint8_t  serial;
	int32_t uWs;
	struct {
		uint8_t output_on:1;
	} flags;	
};

struct eeprom_log_block {
	struct eeprom_log_data data;
	// Writing the data block can not be a atomic
	// operation, crc ensures integrity
	uint16_t crc;
};

struct eeprom_device_block  {
	uint16_t design_capacity;
	uint16_t design_voltage;
};

#define EEPROM_SIZE       512

void eeprom_init();
uint8_t eeprom_find_log_block();
uint8_t eeprom_busy();
void eeprom_write_log_block();


#endif