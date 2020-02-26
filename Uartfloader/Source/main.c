/*
 * Poject TlsrTool FLOADER
 * pvvx 09/2019
 *
 * Poject TlsrComProg
 * pvvx 02/2020
 */
#include "common.h"
#include "gpio.h"
#include "analog.h"
#include "spi_i.h"
#include "flash.h"
#include "clock.h"
#include "crc.h"

/*
 * Calculator CRC-16/MODBUS: https://crccalc.com/
 */
//-------------------------------
#define VERSION_BCD 0x01 // 0x12 -> Ver 1.2
#if CHIP_TYPE == MCU_CORE_8266
#define CHIP_ID 0x8266
#else
#define CHIP_ID 0x8269
#endif
//------------------------------- Init UART ---
#define UART_BAUD 230400 // 115200 or 230400

#if UART_BAUD == 115200
#define bwpc 		8  // 32000000/(8+1)/(30+1)=114695.340502
#define uartCLKdiv	30
#elif UART_BAUD == 230400
#define bwpc 		9  // 32000000/(9+1)/(13+1)=228571.428571
#define uartCLKdiv	13
#endif

#define DATA_BUFF_SIZE 		1024
#define UART_DMA_BUFF_SIZE (DATA_BUFF_SIZE+16)
#define UART_RX_BUFF_SIZE	UART_DMA_BUFF_SIZE
#define UART_TX_BUFF_SIZE	UART_DMA_BUFF_SIZE
//-------------------------------

#define SWIRE_OFF 1

enum{
	CMD_GET_VERSION = 0,		//0
	CMD_FLASH_READ,				//1
	CMD_FLASH_WRITE,			//2
	CMD_FLASH_SECT_ERASE,		//3
	CMD_FLASH_ALL_ERASE,		//4
	CMD_FLASH_GET_JEDEC_ID		//5
};

typedef struct __attribute__((packed)) _blk_head_t {
	u8 cmd;
	u8 addrl;
	u16 addrh;
}blk_head_t;

typedef struct __attribute__((packed)) _cmd_fread_t {
	u16 len;
}cmd_fread_t;

typedef struct __attribute__((packed)) _cmd_fwrite_t {
	u8  data[DATA_BUFF_SIZE];
}cmd_fwrite_t;

typedef struct __attribute__((packed)) _cmd_fserase_t {
	u16 cnt;
}cmd_fserase_t;

typedef struct __attribute__((packed)) _blk_rx_pkt_t{
	blk_head_t head;
	union __attribute__((packed)) {
		cmd_fread_t		fr;
		cmd_fwrite_t	fw;
		cmd_fserase_t	fs;
	};
} blk_rx_pkt_t;

typedef struct _dma_uart_buf_t {
	volatile u32 len;
	union __attribute__((packed)) {
		blk_rx_pkt_t pkt;
		u8 uc[UART_DMA_BUFF_SIZE-4];
		u16 uw[1];
		u32 ud[1];
	};
}dma_uart_buf_t;

dma_uart_buf_t urxb;
dma_uart_buf_t utxb;

//_attribute_ram_code_
inline void uart_init(void) {
	// reg_uart_clk_div/reg_uart_ctrl0
	REG_ADDR32(0x094) = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1)
		|	((MASK_VAL( FLD_UART_BWPC, bwpc) // set bit width
			| MASK_VAL(FLD_UART_STOP_BIT, 1) // 00: 1 bit, 01: 1.5bit 1x: 2bits;
			| FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN) // enable UART DMA mode
			<< 16);
	reg_uart_rx_timeout = MASK_VAL(FLD_UART_TIMEOUT_BW, (bwpc+1)*12) | FLD_UART_BW_MUL2;

	// reg_dma0_addr/reg_dma0_ctrl
	REG_ADDR32(0x500) = (unsigned short)((u32)(&urxb)) //set receive buffer address
		| 	((FLD_DMA_WR_MEM // set DMA0 mode to 0x01 for receive.write to memory
			| MASK_VAL(FLD_DMA_BUF_SIZE, UART_RX_BUFF_SIZE>>4))  //set rx buffer size
			<< 16);
	REG_ADDR32(0x504) = (unsigned short)((u32)(&utxb)) //set tx buffer address
		| 	(MASK_VAL(FLD_DMA_BUF_SIZE, UART_TX_BUFF_SIZE>>4) //set tx buffer size
			<< 16);
}

_attribute_ram_code_ void flash_write_sector(u32 addr, u32 len, u8 *buf) {
	u32 sz = 256;
	while(len) {
		if (len < sz) sz = len;
		flash_write_page(addr, sz, buf);
		addr += sz;
		buf += sz;
		len -= sz;
	}
}

_attribute_ram_code_ int main (void) {
	reg_irq_en = 0;
	// Open clk for MCU running
	REG_ADDR8(0x60) = 0x00;
	REG_ADDR8(0x61) = 0x00;
	REG_ADDR8(0x62) = 0x00;
	REG_ADDR8(0x63) = 0xff;
	REG_ADDR8(0x64) = 0xff;
	REG_ADDR8(0x65) = 0xff;
	REG_ADDR32(0x60) = 0x00000000 // reg_rst_clk0
	| FLD_CLK_SPI_EN
//	| FLD_CLK_I2C_EN
//	| FLD_CLK_USB_EN
//	| FLD_CLK_USB_PHY_EN
	| FLD_CLK_MCU_EN
	| FLD_CLK_MAC_EN
//	| FLD_CLK_ADC_EN	// ADC interface
	| FLD_CLK_ZB_EN
		;

	REG_ADDR32(0x64) = 0
	// reg_clk_sel:
    | (6 << 16) 	  	// reg_clk_sel CLOCK_TYPE_PLL : 192MHz/32MHz = 6
//	| (1 << (7+16))	  	// reg_clk_sel CLOCK_TYPE_OSC
    | (0 << (5+16)) 	// 0:32m clock from rc
//    | (1 << (5+16)) 	// 1:hs divider clk
    // reg_clk_en:
	| FLD_CLK_GPIO_EN
//	| FLD_CLK_ALGM_EN
	| FLD_CLK_DMA_EN
	| FLD_CLK_UART_EN
//	| FLD_CLK_PWM_EN
//	| FLD_CLK_AES_EN
//	| FLD_CLK_32K_TIMER_EN
#if (!SWIRE_OFF)
	| FLD_CLK_SWIRE_EN
#endif
//	| FLD_CLK_32K_QDEC_EN
//	| FLD_CLK_AUD_EN
//	| FLD_CLK_DIFIO_EN
//	| FLD_CLK_KEYSCAN_EN
	| FLD_CLK_MCIC_EN
//	| FLD_CLK_QDEC_EN
	;
	reg_fhs_sel = FHS_SEL_192M_PLL;
	// enable system tick ( clock_time() )
	reg_system_tick_ctrl = FLD_SYSTEM_TICK_START; //	REG_ADDR8(0x74f) = 0x01;

	crcInit();
	uart_init();
	// sws off and enable uart function and enable input
	if(reg_mcu_id == MCU_PROD_ID__8266) { // CHIP_TYPE == MCU_CORE_8266
		//  ART_GPIO_CFG_PC6_PC7: CPGIO PC6/PC7 enable uart function and enable input
		analog_write(0x10, (analog_read(0x10) & 0xf0) | PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
#if (SWIRE_OFF)
		reg_gpio_gpio_func(GPIO_PA0) |= GPIO_PA0 & 0xff; // GPIO_PA0/SWS set gpio
#endif
		BM_CLR(reg_gpio_gpio_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 as gpio
		BM_CLR(reg_gpio_config_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 keyscan function
		BM_SET(reg_gpio_ie(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);  // enable input
	} else { // CHIP_TYPE == MCU_CORE_8269
		// UART_GPIO_CFG_PC2_PC3:  CPGIO PC2/PC3 enable uart function and enable input
		analog_write(0x0f, (analog_read(0x0f) & 0xf0) | PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
#if (SWIRE_OFF)
		reg_gpio_gpio_func(GPIO_PB0) |= GPIO_PB0 & 0xff; // GPIO_PB0/SWS set gpio
#endif
		BM_CLR(reg_gpio_gpio_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		BM_SET(reg_gpio_config_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		BM_SET(reg_gpio_ie(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);  //enable input
	}
#if USE_EXT_FLASH
#define reg_gpio_config_func6   REG_ADDR8(0x5b6)
#define reg_gpio_config_func(i)		REG_ADDR8(0x5b0 +(i>>8))  	  //5b0 5b1 5b2 5b3 5b4 5b5
	BM_CLR(reg_gpio_config_func4, (GPIO_PE7)&0xff);   //disable E6/E7 keyscan function   //(GPIO_PE6|GPIO_PE7)
	BM_CLR(reg_gpio_config_func6, BIT(5));            //disable E6/F0 as uart function
	BM_CLR(reg_gpio_gpio_func(GPIO_PE7), (GPIO_PE7)&0xff); //disable E7 as gpio
	BM_CLR(reg_gpio_gpio_func(GPIO_PF0), (GPIO_PF0)&0xff); //disable F0 as gpio
	BM_CLR(reg_gpio_gpio_func(GPIO_PF1), (GPIO_PF1)&0xff); //disable F1 as gpio
	BM_SET(reg_gpio_ie(GPIO_PE7), (GPIO_PE7)&0xff); //enable input
	BM_SET(reg_gpio_ie(GPIO_PF0), (GPIO_PF0)&0xff); //enable input
	BM_SET(reg_gpio_ie(GPIO_PF1), (GPIO_PF1)&0xff); //enable input

	BM_CLR(reg_gpio_oen(GPIO_PF0), GPIO_PF0 & 0xff); //enable output
	BM_CLR(reg_gpio_oen(GPIO_PF1), GPIO_PF1 & 0xff); //enable output

	BM_SET(reg_gpio_gpio_func(CS_EXT_FLASH), (CS_EXT_FLASH)&0xff); //enable F1 as gpio
	BM_CLR(reg_gpio_ie(CS_EXT_FLASH), CS_EXT_FLASH & 0xff); //disable input
	BM_CLR(reg_gpio_oen(CS_EXT_FLASH), CS_EXT_FLASH & 0xff); //enable output

	BM_SET(reg_spi_sp, FLD_SPI_ENABLE);  //enable spi function. because i2c and spi share part of the hardware in the chip.

	/***set the spi clock. spi_clk = system_clock/((div_clk+1)*2)***/
	BM_CLR(reg_spi_sp, FLD_MASTER_SPI_CLK);  //clear the spi clock division bits
//	reg_spi_sp |= MASK_VAL(FLD_MASTER_SPI_CLK, 0 & 0x7f); //set the clock div bits
	BM_SET(reg_spi_ctrl, FLD_SPI_MASTER_MODE_EN);  //enable spi master mode

	/***config the spi woking mode.For spi mode spec, pls refer to datasheet***/
	BM_CLR(reg_spi_inv_clk, FLD_INVERT_SPI_CLK|FLD_DAT_DLY_HALF_CLK); //clear the mode bits
//	BM_SET(reg_spi_inv_clk,  0 &0x03);  //set the mode 0
#endif
	/////////////////////////// app floader /////////////////////////////
	u16 crc16;
	while(1) {
#if MODULE_WATCHDOG_ENABLE
		WATCHDOG_CLEAR;  //in case of watchdog timeout
#endif
		if((reg_dma_tx_rdy0 & FLD_DMA_UART_TX) == 0) {
			if(reg_dma_irq_src & FLD_DMA_UART_RX) { // new command?
				utxb.len = 0;
				if(urxb.len < sizeof(urxb.pkt.head) + 2) {
//					utxb.ud[0] = 0x00000080; // BAD CMD
//					utxb.len = sizeof(urxb.pkt.head);
				} else {
					urxb.len -= 2;
					utxb.len = sizeof(blk_head_t);
					utxb.ud[0] = urxb.ud[0];
					crc16 = crcFast(urxb.uc, urxb.len);
					if((u8)crc16 != urxb.uc[urxb.len]
					 || (u8)(crc16>>8) != urxb.uc[urxb.len+1]) {
						utxb.pkt.head.cmd |= 0xC0; // BAD CRC
					} else {
						u32 faddr = urxb.ud[0] >> 8;
						switch(urxb.pkt.head.cmd) {
							case CMD_FLASH_READ: // rd 1024 bytes: 01 00 00 00 01 d8,  01 00 00 04 04 00 42 CB -> 01 00 00 04 55 AA 55 AA FE C4
								if(urxb.len == sizeof(urxb.pkt.head)
									|| urxb.pkt.fr.len > DATA_BUFF_SIZE)
									urxb.pkt.fr.len = DATA_BUFF_SIZE;
								utxb.len = urxb.pkt.fr.len + sizeof(urxb.pkt.head);
								flash_read_page(faddr, urxb.pkt.fr.len, utxb.pkt.fw.data);
								break;
							case CMD_FLASH_WRITE: // 02 00 00 04 55 AA 55 AA BE D1 -> 02 00 00 04 00 5F
								if(urxb.len == sizeof(urxb.pkt.head)
									|| urxb.len > DATA_BUFF_SIZE + sizeof(urxb.pkt.head))
									utxb.pkt.head.cmd |= 0x80;
								else
									flash_write_sector(faddr, urxb.len - sizeof(urxb.pkt.head), urxb.pkt.fw.data);
								break;
							case CMD_FLASH_SECT_ERASE: // 03 00 00 04 01 A3 -> 03 00 00 04 01 A3
								flash_erase_sector(faddr);
								break;
							case CMD_FLASH_GET_JEDEC_ID: // 05 00 00 00 00 E8 -> 05 51 40 13 21 34
								flash_get_jedec_id(&utxb.pkt.head.addrl);
								break;
							case CMD_FLASH_ALL_ERASE: // 04 00 00 00 01 14 -> 04 00 00 00 01 14
								if(!faddr) flash_erase_all();
								else utxb.pkt.head.cmd |= 0x80;
								break;
							case CMD_GET_VERSION: // 00 00 00 00 00 24 ->
								utxb.pkt.head.addrl = VERSION_BCD;
								utxb.pkt.head.addrh = reg_prod_id; //chip id
								break;
							default:	// 1F 34 56 78 79 BC -> 9F 34 56 78 50 7C
								utxb.pkt.head.cmd |= 0x80;
								break;
						}
					}
				}
				reg_dma_irq_src = FLD_DMA_UART_RX | FLD_DMA_UART_TX;
				if(utxb.len) {
					crc16 = crcFast(utxb.uc, utxb.len);
					utxb.uc[utxb.len++] = crc16;
					utxb.uc[utxb.len++] = crc16 >> 8;
					reg_dma_tx_rdy0 |= FLD_DMA_UART_TX; // start tx
				}
			}
		}
	}
	REG_ADDR8(0x6f) = 0x20;   //mcu reboot
	while (1);
}
