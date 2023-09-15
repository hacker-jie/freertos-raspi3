#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "demo.h"

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );

/*-----------------------------------------------------------*/

void TaskA(void *pvParameters)
{
	(void) pvParameters;

    for( ;; )
    {
		for (char i = '1'; i <= '5'; i++)
		    uart_putchar(i);

		vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void TaskB(void *pvParameters)
{
	(void) pvParameters;

    for( ;; )
    {
		for (char i = 'A'; i <= 'E'; i++)
		    uart_putchar(i);

		vTaskDelay(30 / portTICK_RATE_MS);
    }
}

/*-----------------------------------------------------------*/

void main(void)
{
	TaskHandle_t task_a;
	TaskHandle_t task_b;

	uart_init();

	xTaskCreate(TaskA, "Task A", 512, NULL, tskIDLE_PRIORITY, &task_a);
	xTaskCreate(TaskB, "Task B", 512, NULL, tskIDLE_PRIORITY, &task_b);

	vTaskStartScheduler();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
