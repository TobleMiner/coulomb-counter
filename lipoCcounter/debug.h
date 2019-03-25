#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG_DDR  DDRB
#define DEBUG_PORT PORTB

#define DEBUG_INIT(x) (DEBUG_DDR |= BIT((x)))
#define DEBUG_LO(x)   (DEBUG_PORT &= ~BIT((x)))
#define DEBUG_HI(x)   (DEBUG_PORT |= BIT((x)))
#define DEBUG_BLIP(x) do { DEBUG_HI((x)); asm("nop\n"); DEBUG_LO((x)); } while(0)
	
#define DEBUG_PARALLEL_DDR  DDRD
#define DEBUG_PARALLEL_PORT PORTD

#define DEBUG_PARALLEL_INIT (DEBUG_PARALLEL_DDR = 0xFF)
#define DEBUG_PARALLEL(x)   (DEBUG_PARALLEL_PORT = (x))

#endif