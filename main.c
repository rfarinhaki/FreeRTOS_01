#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
//#include "led_task.h"
//#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void initLedGPIO()
{
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

void vLedTask( void *pvParameters )
{
    uint8_t val = 0x0;
    for( ;; )
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, val);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        if(val==0x0)
            val=GPIO_PIN_3;
        else
            val=0x0;
    }

        /* Tasks must not attempt to return from their implementing
        function or otherwise exit.  In newer FreeRTOS port
        attempting to do so will result in an configASSERT() being
        called if it is defined.  If it is necessary for a task to
        exit then have the task call vTaskDelete( NULL ) to ensure
        its exit is clean. */
    vTaskDelete( NULL );
}

/**
 * main.c
 */
int main(void)
{
    uint32_t clk;

    // Set the clocking to run at 50 MHz from the PLL.
    //-5
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    initLedGPIO();

    clk = SysCtlClockGet();

    if(xTaskCreate(vLedTask, (const portCHAR *)"LED", 256, NULL, 1, NULL) != pdTRUE)
    {
        return(1);
    }

    vTaskStartScheduler();
    while(true){
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        //SysCtlDelay(SysCtlClockGet()/3);
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
        //SysCtlDelay(SysCtlClockGet()/3);
    };

	return 0;
}
