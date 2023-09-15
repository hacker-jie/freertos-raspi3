/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "demo.h"

#define DEVICE_BASE 		0x3F000000	
#define PBASE 			DEVICE_BASE

#define TIMER_CS        (PBASE+0x00003000)
#define TIMER_CLO       (PBASE+0x00003004)
#define TIMER_CHI       (PBASE+0x00003008)
#define TIMER_C0        (PBASE+0x0000300C)
#define TIMER_C1        (PBASE+0x00003010)
#define TIMER_C2        (PBASE+0x00003014)
#define TIMER_C3        (PBASE+0x00003018)

#define TIMER_CS_M0	(1 << 0)
#define TIMER_CS_M1	(1 << 1)
#define TIMER_CS_M2	(1 << 2)
#define TIMER_CS_M3	(1 << 3)

#define SYSTEM_TIMER_IRQ_0_BIT (1 << 0)
#define SYSTEM_TIMER_IRQ_1_BIT (1 << 1)
#define SYSTEM_TIMER_IRQ_2_BIT (1 << 2)
#define SYSTEM_TIMER_IRQ_3_BIT (1 << 3)
#define AUX_IRQ_BIT	       (1 << 29)

#define IRQ_PENDING_1		(PBASE+0x0000B204)
#define ENABLE_IRQS_1		(PBASE+0x0000B210)

const unsigned int interval = 10000;
unsigned int curVal = 0;

void vClearTickInterrupt( void )
{
	put32(TIMER_C1, get32(TIMER_CLO) + interval);
	put32(TIMER_CS, TIMER_CS_M1);
}

void init_timer(void)
{
	put32(TIMER_C1, get32(TIMER_CLO) + interval);
}

void vConfigureTickInterrupt(void)
{
	uint32_t reg;

	init_timer();

	reg = get32(ENABLE_IRQS_1);
	reg |= SYSTEM_TIMER_IRQ_1_BIT;
	put32(ENABLE_IRQS_1, reg);
}

void handle_timer1_irq(void)
{
	FreeRTOS_Tick_Handler();
}

void vApplicationIRQHandler(void)
{
	unsigned int irq = get32(IRQ_PENDING_1);

	if (irq & SYSTEM_TIMER_IRQ_1_BIT) {
		handle_timer1_irq();
	}

	if (irq & AUX_IRQ_BIT) {
		irq_handler();
	}
}

