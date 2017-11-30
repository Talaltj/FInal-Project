#include "weather.h"
#include <stdbool.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"


void Rain_Sensor(void)  // Function which is called for rain sensor
{
			UARTprintf("\n It is raining, speed limit is 50mph\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
	

}
void Snow_Sensor(void)  // Function which is called for Snow sensor
{
		UARTprintf("\nIt is snowing, Speed limit is 40mph\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)


}


void Ice_Sensor(void)  // Function which is called for Ice sensor
{
			UARTprintf("\nIt is Icy, Speed Limit is 30mph\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
	
			

}


void Fog_Sensor(void)  // Function which is called for fog sensor
{
			UARTprintf("\nIt is Foggy, Speed Limit is 35mph\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
	

}
