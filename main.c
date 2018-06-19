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

SemaphoreHandle_t xSemaphore;

void ButtonInterrupt()
{
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) {
            // PF4 was interrupt cause
            GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag
            xSemaphoreGiveFromISR(xSemaphore,0);

    }

}

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
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);


}

void initButtonGPIO(){
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,
            GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);        // Disable interrupt for PF4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);      // Clear pending interrupts for PF4
    GPIOIntRegister(GPIO_PORTF_BASE, ButtonInterrupt);     // Register our handler function for port F
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4,
        GPIO_FALLING_EDGE);             // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);     // Enable interrupt for PF4

}

void vLedTask( void *pvParameters )
{
    uint8_t val = 0x0;
    for( ;; )
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, val);
    //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, val);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        if(val==0x0)
            val=GPIO_PIN_3;
        else
            val=0x0;
    }

    vTaskDelete( NULL );
}

void vRedLedTask(void *pvParameters)
{
    uint8_t val = 0x0;
    for(;;)
    {
        if( xSemaphore != NULL )
            {
                // Obtain the semaphore - don't block if the semaphore is not
                // immediately available.
                if( xSemaphoreTake( xSemaphore, ( TickType_t ) 999999 ) )
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, val);

                                if(val==0x0)
                                    val=GPIO_PIN_2;
                                else
                                    val=0x0;
                 }


            }

    }

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
    initButtonGPIO();
    vSemaphoreCreateBinary(xSemaphore);
    xSemaphoreTake(xSemaphore, 0);

    clk = SysCtlClockGet();

    if(xTaskCreate(vLedTask, (const portCHAR *)"LED", 256, NULL, 1, NULL) != pdTRUE)
    {
        return(1);
    }

    if(xTaskCreate(vRedLedTask, (const portCHAR *)"RED", 256, NULL, 2, NULL) != pdTRUE)
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
