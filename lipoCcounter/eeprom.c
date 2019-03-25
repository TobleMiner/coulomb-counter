#include "eeprom.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "crc.h"
#include "util.h"

static uint16_t next_log_entry;

static unsigned char* write_data;
uint8_t write_len;
uint16_t write_addr;

#define LOG_ENTRY_LEN (sizeof(struct eeprom_log_block))
#define LOG_DATA_LEN (sizeof(struct eeprom_log_data_priv))
#define DEV_DATA_LEN (sizeof(struct eeprom_device_block))

#define LOG_ENTRIES CLAMPH(((EEPROM_SIZE - sizeof(struct eeprom_device_block)) / LOG_ENTRY_LEN), 255)

EEMEM struct eeprom_device_block devicedata = {
	.design_capacity_mAh = 2500
};
EEMEM struct eeprom_log_block logdata[LOG_ENTRIES];


static struct eeprom_log_block log_block;

struct eeprom_log_data log_data;
struct eeprom_device_block dev_block;

void eeprom_init() {
	next_log_entry = 0;
}

static void inc_log() {
	next_log_entry++;
	if(next_log_entry >= ARRAY_LEN(logdata)) {
		next_log_entry = 0;
	}
	log_block.data.serial++;
}

// For some bizarre reason eeprom_read_block reads data backwards...
static void eeprom_read_block_8(void* dst, void* src, uint8_t len) {
	while(len--) {
		*((unsigned char*)dst++) = eeprom_read_byte(src++);
	}
}

static uint8_t eeprom_read_log_block(struct eeprom_log_block* log, uint8_t entry) {
	eeprom_read_block_8(log, &logdata[entry], LOG_ENTRY_LEN);
	return log->crc == crc16_8((unsigned char*)&log->data, LOG_DATA_LEN);
}

uint8_t eeprom_find_log_block() {
	struct eeprom_log_block log;
	struct eeprom_log_block last_log;
	uint8_t entry = 0;
	uint8_t found = 0;
	uint8_t max_serial = 0;
	while(entry < ARRAY_LEN(logdata)) {
		if(eeprom_read_log_block(&log, entry)) {
			if(found) {
				if((last_log.data.serial == 0xFF) && (log.data.serial < last_log.data.serial)) {
					max_serial = 0;
				}
			}
			if(log.data.serial >= max_serial) {
				log_block = log;
				log_data = log_block.data.data;
				next_log_entry = entry;
				inc_log();
				found = 1;
			}
			last_log = log;
		}
		entry++;
	}
	return found;
}

void eeprom_read_device_block() {
	eeprom_read_block_8(&dev_block, &devicedata, DEV_DATA_LEN);
}

uint8_t eeprom_busy() {
	return write_len != 0;
}

void eeprom_write_next_byte() {
	EEARL = write_addr & 0xFF;
	EEARH = (write_addr >> 8);
	EEDR = *write_data;
	EECR |= BIT(EEMPE) | BIT(EERIE);
	EECR |= BIT(EEPE);
}

void eeprom_write_log_block() {
	log_block.data.data = log_data;
	write_addr = (uint16_t)&logdata[next_log_entry];
	write_len = LOG_ENTRY_LEN;
	write_data = (unsigned char*)&log_block;
	log_block.crc = crc16_8((unsigned char*)&log_block.data, LOG_DATA_LEN);
	eeprom_write_next_byte();
}

ISR(EE_READY_vect) {
	write_addr++;
	write_data++;
	write_len--;
	if(write_len) {
		// Enqueue next byte
		eeprom_write_next_byte();
	} else {
		// Transfer finished, disable interrupts and increment serial
		EECR &= ~BIT(EERIE);
		inc_log();
	}
}
