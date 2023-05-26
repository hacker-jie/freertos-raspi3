/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "demo.h"

#if 0
/* ARM Generic Timer */
#define CORE0_TIMER_IRQCNTL    ((volatile uint32_t *)(0x40000040))
static uint32_t timer_cntfrq = 0;
static uint32_t timer_tick = 0;

void enable_cntv(void)
{
	uint32_t cntv_ctl;
	cntv_ctl = 1;
	asm volatile ("msr cntv_ctl_el0, %0" :: "r" (cntv_ctl));
}
/*-----------------------------------------------------------*/

void write_cntv_tval(uint32_t val)
{
	asm volatile ("msr cntv_tval_el0, %0" :: "r" (val));
	return;
}
/*-----------------------------------------------------------*/

uint32_t read_cntfrq(void)
{
	uint32_t val;
	asm volatile ("mrs %0, cntfrq_el0" : "=r" (val));
	return val;
}
/*-----------------------------------------------------------*/

void init_timer(void)
{
	timer_cntfrq = timer_tick = read_cntfrq();
	write_cntv_tval(timer_cntfrq);    // clear cntv interrupt and set next 1 sec timer.
	return;
}
/*-----------------------------------------------------------*/

void timer_set_tick_rate_hz(uint32_t rate)
{
	timer_tick = timer_cntfrq / rate ;
	write_cntv_tval(timer_tick);
}
/*-----------------------------------------------------------*/

void vConfigureTickInterrupt( void )
{
	/* init timer device. */
	init_timer();

	/* set tick rate. */
	timer_set_tick_rate_hz(configTICK_RATE_HZ);

	/* timer interrupt routing. */
	*CORE0_TIMER_IRQCNTL = 1 << 3; /* nCNTVIRQ routing to CORE0.*/

	/* start & enable interrupts in the timer. */
	enable_cntv();
}
/*-----------------------------------------------------------*/

void vClearTickInterrupt( void )
{
	write_cntv_tval(timer_tick);    // clear cntv interrupt and set next timer.
	return;
}
/*-----------------------------------------------------------*/

void vApplicationIRQHandler( uint32_t ulCORE0_INT_SRC )
{
	uint32_t ulInterruptID;
	ulInterruptID = ulCORE0_INT_SRC & 0x0007FFFFUL;

	/* call handler function */
	if(ulInterruptID & (1 << 3))
	{
		/* Generic Timer */
		FreeRTOS_Tick_Handler();
	}
	if(ulInterruptID & (1 << 8))
	{
		/* Peripherals */
		irq_handler();
	}
}
#endif

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

const unsigned int interval = 1000;
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
	//uart_send('a');
	//uart_puts("irq");

	unsigned int irq = get32(IRQ_PENDING_1);

	if (irq & SYSTEM_TIMER_IRQ_1_BIT) {
		handle_timer1_irq();
		//uint32_t tick = (uint32_t)xTaskGetTickCount();
		//put32(TIMER_C3, tick);
		//uart_send('a');
	}

	if (irq & AUX_IRQ_BIT) {
		irq_handler();
//		uart_send('b');
	}
}

