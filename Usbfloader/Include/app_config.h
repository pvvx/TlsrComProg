#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

//------------- Project Config ---------------//
// USE_USB_CDC or USE_INT_UART (!)
#define USE_USB_CDC			1
#define USE_INT_UART		0 
#if (USE_USB_CDC)
#define DATA_BUFF_SIZE 		4096
#else
#define DATA_BUFF_SIZE 		1024
#endif
//-------------- UART Config -----------------//
#define UART_BAUD 			230400 	// 115200 or 230400

//-------- External Flash Config -------------//
#define USE_EXT_FLASH 0
#define CS_EXT_FLASH GPIO_PB5

#define CHIP_TYPE MCU_CORE_8266

#define MCU_CORE_TYPE CHIP_TYPE

/////////////////// Clock  /////////////////////////////////

#define CLOCK_FHS_TYPE		FHS_SEL_PLL // FHS_SEL_PLL, FHS_SEL_48M, FHS_SEL_RC, FHS_SEL_PAD
#define CLOCK_SYS_TYPE  	CLK_SEL_RC32M // one of the following: CLK_SEL_HSDIV, CLK_SEL_RC32M, CLK_SEL_PADM, CLK_SEL_PADK

#define CLOCK_SYS_CLOCK_HZ  	32000000

#define	QUARTZ_16MHZ	16
#define	QUARTZ_12MHZ	12

#define SET_PLL 		QUARTZ_12MHZ	// QUARTZ_16MHZ, QUARTZ_12MHZ

/////////////////// watchdog  //////////////////////////////

#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		100  //ms


/////////////////// IRQ  /////////////////////////////////
#define USE_IRQ_SAVE 	USE_USB_CDC

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
