#ifndef TWI_I2C_SLAVE
#define TWI_I2C_SLAVE

#include <stdint.h>

struct TWI_I2C_Reg {
	struct {
		uint8_t changed:1;
	} attr;
	uint8_t data;
};

//TWI I2C Initialize
//  address - If slave, this parameter is the slave address
void TWI_I2C_Init(char address, struct TWI_I2C_Reg** regs, uint8_t num_regs);
uint8_t TWI_I2C_Busy();


#endif
