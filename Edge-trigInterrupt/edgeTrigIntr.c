
//*****************************************************************************
//Edge-triggered Interrupt on PORTF pin 4 (left side swich on launchpad board)
// --flashes blue LED when edge-trigerred interrupt occurs on PF4
//*****************************************************************************
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
#include "weather.h"

#define RED_LED	  GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define MS_DELAY_500 375000
#define MS_DELAY_1000 750000   //Set up longer delay
#define SYS_CLOCK 4000000

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
//#define PF0       (*((volatile uint32_t *)0x40025004)) // SW2
//#define PF4       (*((volatile uint32_t *)0x40025040)) // SW1
//#define SWITCHES  (*((volatile uint32_t *)0x40025044))  
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define RED       0x02 //PF1
#define BLUE      0x04 //PF2
#define GREEN     0x08  //PF3

#define NVIC_EN0_INT30          0x40000000  // Interrupt 30 enable
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
//NVIC_EN0_R   IRQ 0 to 31 Set Enable Register
//NVIC_PRI7_R  IRQ 28 to 31 Priority Register

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode




void InitConsole(void); //Used for UART display

static int8_t tmp, tmp2;; //Global variables

void SysTick_Init(void){ 
	NVIC_ST_CTRL_R = 0; // 1) disable SysTick during setup 
  NVIC_ST_RELOAD_R = 0x00FFFFFF; // 2) maximum reload value 
  NVIC_ST_CURRENT_R = 0; // 3) any write to CURRENT clears it 
  NVIC_ST_CTRL_R = 0x00000005; // 4) enable SysTick with core clock } // The delay parameter is in units of the 80 MHz core clock(12.5 ns)
}	

void SysTick_Wait(unsigned long delay){ 
  NVIC_ST_RELOAD_R = delay-1; // number of counts 
  NVIC_ST_CURRENT_R = 0; // any value written to CURRENT clears 
  while((NVIC_ST_CTRL_R&0x00010000)==0){ } 
}

void SysTick_Wait10ms(unsigned long delay){ unsigned long i; 
	for(i=0; i<delay; i++){ 
  SysTick_Wait(800000); // wait 10ms 
	}
}

void SystemInit(){
	// set system clock ( 4 Mhz )
	SysCtlClockSet(SYSCTL_SYSDIV_50 | 
	SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

void  GPIOPortF_Handler(){
	 GPIO_PORTF_ICR_R = GPIO_PIN_4;   // acknowledge flag4
	
	 UARTprintf("\nWeather Warning Issued, Drive Carefully !!!!!.\n");
			
		if(tmp == 0x01)	//Switch 1		
		{
			UARTprintf("\nIt is snowing heavily, Speed limit is 20\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
		}
		
		else if(tmp == 0x02)	//Switch 2		
		{
			UARTprintf("\n It is raining heavily, speed limit is 35\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
		}
		
		else if(tmp == 0x04)	//Switch 3		
		{
			UARTprintf("\nIt is very Icy, Speed Limit is 20\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
		}
		
		else if(tmp == 0x08)	//Switch 4
		{
			UARTprintf("\nIt is Very Foggy, Speed Limit is 25\n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
		}	
		
		else
			{
		  	UARTprintf("\nThe Weather is Perfect, Speed Limit is 70 \n");
				SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
			}
	
}

void alternate_gpioInit(){
	// **** Using Tivaware Peripheral Driver Library Function Calls ****
	// init port f ( use pin 4 for switch 1 )
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// wait for port f enable
	 while ( !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) ) {}
	// configure port f pin 4 as input
   GPIO_PORTF_DIR_R &= ~GPIO_PIN_4;
	// enable weak pull ups on pin 4
	 GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, 
										0x00, GPIO_PIN_TYPE_STD_WPU);
	// register interrupt for port f pin 4
	  GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortF_Handler);
	// set interrupt type ( rising edge )
   GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
	// enable interrupts on port f ( rising edge )
  	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	// initialize pin2 for the blue LED
	  GPIO_PORTF_DIR_R |= GPIO_PIN_2;
    GPIO_PORTF_DEN_R |= GPIO_PIN_2;
}
	
void gpioInit(){ uint32_t fallingedges;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
	fallingedges = 0;  // short wait
	GPIO_PORTF_LOCK_R = 0x4C4F434B;  //  unlock Port F
	GPIO_PORTF_CR_R |= (GPIO_PIN_4 + GPIO_PIN_2);      //allow changes to PF0 and PF2
  GPIO_PORTF_DIR_R &= ~GPIO_PIN_2;    //  make PF4 input (built-in button)
	GPIO_PORTF_DIR_R |= 0x04;			//     and PF2 output (Blue LED)
  GPIO_PORTF_AFSEL_R &= ~(GPIO_PIN_4 + GPIO_PIN_2);  //     disable alt funct on PF4 and PF2
  GPIO_PORTF_DEN_R |= (GPIO_PIN_4 + GPIO_PIN_2);    //     enable digital I/O on PF4 and PF2   
  GPIO_PORTF_PCTL_R &= ~0x000F0F00; // configure PF4 and PF2 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //  disable analog functionality on PF
	GPIO_PORTF_PUR_R |= GPIO_PIN_4;     //  enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~GPIO_PIN_4;     //  PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~GPIO_PIN_4;    //  PF4 is not both edges
  GPIO_PORTF_IEV_R |= GPIO_PIN_4;     // PF4 rising edge event
  GPIO_PORTF_ICR_R = GPIO_PIN_4;      //  clear flag0
  GPIO_PORTF_IM_R |= GPIO_PIN_4;      //  arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = NVIC_EN0_INT30;  // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // 
}

void PortF_Init(void){ volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void PortD_Init(void){ unsigned long volatile delay;
//Set port D pins 0-3 as digital input
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3 ; // activate port D Run Mode Clock
  delay = 100;
  delay = 100;
  GPIO_PORTD_DIR_R &= ~0x0000000F;
	GPIO_PORTD_AFSEL_R &= ~0x0000000F;
	GPIO_PORTD_AMSEL_R &= ~0x0000000F; // disable analog mode PF0-7
	GPIO_PORTD_DEN_R |= 0x0000000F;  // enable digital I/O PF0-7
}

uint32_t PortD_Input(void){  uint32_t operand;
	//Read pins 0-3 on port D
	return GPIO_PORTD_DATA_R & 0x0000000F;
}

uint32_t PortF_Input(void){ 
	SysTick_Wait10ms(4); //Debounce delay 4 * 10ms
  return (GPIO_PORTF_DATA_R&0x11);  // read PF4,PF0 inputs SW2 SW1 resp.
}

void PortF_Output(uint32_t data){ // write PF3-PF1 outputs
  GPIO_PORTF_DATA_R = data;       // PF1 red PF2 blue   PF3 green
}


int main()
{
	gpioInit(); //Interrupt setup and enable
	InitConsole();
	
	SysTick_Init();  // initialize Systick
 
	PortF_Init();   // initialize PF0 and PF4 and make them inputs
									// make PF3-1 out (PF3-1 built-in LEDs)
	
	PortD_Init();  //Set port D pins 0-3 as digital input
	
	//Entry Mode
	UARTprintf("Welcome to Intelligent Vehicle System\n");
	UARTprintf("*********************************\n");
	UARTprintf("Please select switches as follows:\n");
	UARTprintf("Sensor1: 											Snow\n");
	UARTprintf("Sensor2: 											Rain\n");
	UARTprintf("Sensor3: 											Icy\n");
	UARTprintf("Sensor4: 											Fog\n");
	UARTprintf("*********************************\n");
	SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec);
	while ( 1 )
	{
				
		tmp = PortD_Input();		//Read in psuh buttons connected to Port D
					
		if(tmp == 0x01)	//Switch 1		
		{
			Snow_Sensor();
		}
		
		else if(tmp == 0x02)	//Switch 2		
		{
			Rain_Sensor();
			}
		
		else if(tmp == 0x04)	//Switch 3		
		{
			Ice_Sensor();
		}
		
		else if(tmp == 0x08)	//Switch 4
		{
			Fog_Sensor();
		}	
		
		else
		{
			UARTprintf("\nThe Weather is Perfect, Speed Limit is 70 \n");
			SysTick_Wait10ms(100); //Selection delay (100 * 10 ms = 1 sec)
		}
		
	}	; // wait for interrupts
	
}


//


//


void InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
   

		// Select the alternate (UART) function for these pins.   
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O. 9600 BAUD
    UARTStdioConfig(0, 9600, 16000000);
}

//EOF

