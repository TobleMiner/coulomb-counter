#include "twi_i2c_slave.h"
#include "util.h"
#include "time.h"
#include "debug.h"

#define F_CPU 8000000UL

#include <avr/interrupt.h>
#include <util/delay.h>

#define I2C_STATUS_START_W 0x60
#define I2C_STATUS_START_R 0xA8
#define I2C_STATUS_BYTE_R  0x80
#define I2C_STATUS_BYTE_W  0xB8
#define I2C_STATUS_NAK_W   0xC0
#define I2C_STATUS_STOP_W  0xA0
#define I2C_STATUS_STOP_R  0xC8

enum {
	I2C_STATE_IDLE = 0,	
	I2C_STATE_ADDRESS,
	I2C_STATE_DATA
};

struct {
	uint8_t state:2;
	struct timeval_t timeout;
	struct timeval_t last_xfer;
	struct TWI_I2C_Reg** regs;
	struct TWI_I2C_Reg* active_reg;
	uint8_t num_regs;
} i2c;

void TWI_I2C_Init(char address, struct TWI_I2C_Reg** regs, uint8_t num_regs) {
	i2c.state = I2C_STATE_IDLE;
	i2c.timeout.secs = 0;
	i2c.timeout.nsecs = 10000000UL;
	i2c.last_xfer.nsecs = 0;
	i2c.last_xfer.secs = 0;
	i2c.regs = regs;
	i2c.num_regs = num_regs;

	TWCR = BIT(TWIE) | BIT(TWEA) | BIT(TWEN);
	TWAR = (address << 1);
}

uint8_t i2c_state;

uint8_t TWI_I2C_Busy() {
	struct timeval_t time;
	struct timeval_t delta;
	if(i2c.state == I2C_STATE_IDLE) {
		return 0;
	}
	now_fast(&time);
	ATOMIC_BEGIN
	timedelta(&i2c.last_xfer, &time, &delta);
	if(timecmp(&delta, &i2c.timeout) != LT) {
		i2c.state = I2C_STATE_IDLE;
	}
	ATOMIC_END
	return i2c.state != I2C_STATE_IDLE;
}

ISR(TWI_vect) {
	uint8_t status = TWSR & 0b11111000;
	switch(status)
	{
		case I2C_STATUS_START_W:
			i2c.state = I2C_STATE_ADDRESS;
			i2c.active_reg = NULL;
			break;
		case I2C_STATUS_BYTE_R:
//					i2c.num_bytes++;
			switch(i2c.state) {
				case I2C_STATE_ADDRESS:
					status = TWDR;
					if(status < i2c.num_regs) {
						i2c.active_reg = i2c.regs[status];
					} else {
						i2c.active_reg = NULL;
					}
					i2c.state = I2C_STATE_DATA;
					break;
				case I2C_STATE_DATA:
					if(i2c.active_reg) {
						i2c.active_reg->data = TWDR;
						i2c.active_reg->attr.changed = 1;
					}
					break;
				default:
					i2c.state = I2C_STATE_IDLE;
			}
			break;
		case I2C_STATUS_STOP_W:
// Just ignore number of bytes written for now
//					if(i2c.num_bytes == 1) {
//					}
			break;
		case I2C_STATUS_START_R:
			i2c.state = I2C_STATE_DATA;
		case I2C_STATUS_BYTE_W:
			if(i2c.active_reg) {
				TWDR = i2c.active_reg->data;
			} else {
				TWDR = 0;
			}
			break;
		case I2C_STATUS_NAK_W:
		case I2C_STATUS_STOP_R:
		default:
			TWCR |= BIT(TWEA) | BIT(TWSTO);
			i2c.state = I2C_STATE_IDLE;
			i2c.active_reg = NULL;
	}
	now_fast(&i2c.last_xfer);
	TWCR |= BIT(TWINT);
}