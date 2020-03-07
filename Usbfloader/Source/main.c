/*
 * Poject TlsrTool FLOADER
 * pvvx 09/2019
 *
 * Poject TlsrComProg
 * pvvx 02/2020
 */
#include "common.h"
#include "spi_i.h"
#include "flash.h"
#include "crc.h" // Calculator CRC-16/MODBUS: https://crccalc.com/
#include "usb.h"
//===============================
#define VERSION_BCD 0x02 // 0x12 -> Ver 1.2
//------------------------------- Init UART ---
#if UART_BAUD == 115200
#define bwpc 		8  // 32000000/(8+1)/(30+1)=114695.340502
#define uartCLKdiv	30
#elif UART_BAUD == 230400
#define bwpc 		9  // 32000000/(9+1)/(13+1)=228571.428571
#define uartCLKdiv	13
#elif UART_BAUD == 500000
#define bwpc 		3  // 32000000/(3+1)/(7+1)=1000000
#define uartCLKdiv	7
#elif UART_BAUD == 1000000
#define bwpc 		3  // 32000000/(3+1)/(3+1)=2000000
#define uartCLKdiv	3
#endif
//-------------------------------

#define SWIRE_OFF USE_INT_UART

//-------------------------------
dma_uart_buf_t urxb = {.len = 0};
dma_uart_buf_t utxb = {.len = 0};
//-------------------------------


#if (USE_USB_CDC)

volatile char usb_flg = 0; // Flg bit0: DTR - Open/Close USB-COM port
/*
_attribute_ram_code_
void irq_handler(void){
	USB_IrqHandle();
}
*/
/* rxFunc rx callback function
 * Called from Irq (!) */
_attribute_ram_code_
void USBCDC_RxCb(unsigned char *data, unsigned int len){
	if (len) { // есть данные?
		if(urxb.len == 0
			&& data
			&& len >= sizeof(blk_head_t) + 2 // + sizeof(CRC)
			&& len <= USB_CDC_MAX_RX_BLK_SIZE) {
			urxb.len = len;
		}
		USBCDC_RxBufSet((unsigned char *)&urxb.uc); // назначить новый буфер (в данном приложении единственный)
	}
}
#ifndef USB_TX_CALLBACK
_attribute_ram_code_
void USBCDC_TxCb(void) {
	utxb.len = 0;
}
#endif
#else
_attribute_ram_code_
void irq_handler(void){
}
#endif // USE_USB_CDC


_attribute_ram_code_
void flash_write_sector(u32 addr, u32 len, u8 *buf) {
	u32 sz = 256;
	while(len) {
		if (len < sz) sz = len;
		flash_write_page(addr, sz, buf);
		addr += sz;
		buf += sz;
		len -= sz;
	}
}

_attribute_ram_code_
void cmd_decode(void) {
	u16 crc16;
	if(urxb.len < sizeof(urxb.pkt.head) + 2) { // + sizeof(CRC)
//		utxb.ud[0] = 0x00000080; // BAD CMD
//		utxb.len = sizeof(urxb.pkt.head);
		utxb.len = 0;
	} else {
		urxb.len -= 2; // - sizeof(CRC)
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
					switch(urxb.pkt.head.addrl) {
					default: // case 0:
						utxb.pkt.head.addrl = VERSION_BCD;
						utxb.pkt.head.addrh = reg_prod_id; //chip id
						break;
					case 1:
						REG_ADDR8(0x6f) = 0x20;   // mcu reboot
						break;
					}
					break;
				default:	// 1F 34 56 78 79 BC -> 9F 34 56 78 50 7C
					utxb.pkt.head.cmd |= 0x80;
					break;
			}
		}
	}
}

_attribute_ram_code_
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

_attribute_ram_code_
int main (void) {
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
#if (USE_USB_CDC)
	| FLD_CLK_USB_EN
	| FLD_CLK_USB_PHY_EN
#endif
	| FLD_CLK_MCU_EN
	| FLD_CLK_MAC_EN
//	| FLD_CLK_ADC_EN	// ADC interface
	| FLD_CLK_ZB_EN
		;
//	analog_write(0x88, 0x0f);
	// rega_pwdn_setting1:
//	analog_write(0x05, 0x62);

	REG_ADDR32(0x70) = 0 // = 0x04000400
	/* reg_fhs_sel [0x70], After reset = 0x00 */
		| (((CLOCK_FHS_TYPE == FHS_SEL_RC) || (CLOCK_FHS_TYPE == FHS_SEL_PAD))?1:0)
	/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
		| ((1<<2)<<8)
	/* reg_?? [0x72], After reset [0x72] = 0x00 */
		| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
	/* reg_clk_mux_cel [0x73], After reset  = 0x14
	* [0] clk32k select; 0: sel 32k osc 1: 32k pad
	* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
	* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
	* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
		| ((1<<2)<<24);

	REG_ADDR32(0x64) = 0
    // reg_clk_en:
	| FLD_CLK_GPIO_EN
	| FLD_CLK_ALGM_EN
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
	// reg_clk_sel (0x66):
	| (((((CLOCK_FHS_TYPE == FHS_SEL_48M) || (CLOCK_FHS_TYPE == FHS_SEL_PAD))?1:0) << 7) << 16)
    | (MASK_VAL(FLD_CLK_SEL_DIV, CLOCK_PLL_CLOCK/CLOCK_SYS_CLOCK_HZ, FLD_CLK_SEL_SRC, CLOCK_SYS_TYPE) << 16)
    // reg_i2s_step (0x67):
    | (MASK_VAL(FLD_I2S_STEP, 8, FLD_I2S_CLK_EN, 0) << 24)  // i2s clk disable
	;
	// enable system tick ( clock_time() )
	reg_system_tick_ctrl = FLD_SYSTEM_TICK_START; //	REG_ADDR8(0x74f) = 0x01;

#if (USE_USB_CDC)
#if SET_PLL == QUARTZ_12MHZ
	reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN | 0x80;
	analog_write(0x099, 0xb1);
	analog_write(0x082, 0x20);
	analog_write(0x09e, 0xad);
#else // SET_PLL == CLK_QUARTZ
	reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN;
	analog_write(0x099, 0x31);
	analog_write(0x082, 0x34);
	analog_write(0x09e, 0x82);
#endif // SET_PLL
#endif // USE_USB_CDC

	crcInit();
#if	(USE_INT_UART)
	uart_init();
#endif
	// sws off and enable uart function and enable input
	if(reg_mcu_id == MCU_PROD_ID__8266) { // CHIP_TYPE == MCU_CORE_8266
#if (USE_USB_CDC)
#if USE_USB
#define PB5_FUNC	AS_USB
#define PB6_FUNC	AS_USB
#define PB5_INPUT_ENABLE	1
#define PB6_INPUT_ENABLE	1
analog_write (0x0e,  PULL_WAKEUP_SRC_PB6 |
					(PULL_WAKEUP_SRC_PB7<<2) |
#define PULL_WAKEUP_SRC_PB5           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DM
#define PULL_WAKEUP_SRC_PB6           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DP
#endif
		//-------------------------- USB pins 8266
		// USB-DM: PULL_WAKEUP_SRC_PB5 PM_PIN_PULLDOWN_100K
		analog_write(0x0d, PM_PIN_PULLDOWN_100K<<6);
		// PB5_FUNC AS_USB, PB6_FUNC AS_USB
		//BM_CLR(reg_gpio_gpio_func(GPIO_PB5), (GPIO_PB5 | GPIO_PB6) & 0xff); // disable PB5/PB6 as gpio
		// PB5_INPUT_ENABLE 1, PB6_INPUT_ENABLE 1
		//BM_SET(reg_gpio_ie(GPIO_PB5), (GPIO_PB5 | GPIO_PB6) & 0xFF);  // enable input
#endif // USE_USB_CDC
#if	(USE_INT_UART)
		//-------------------------- UART pins 8266
		//  ART_GPIO_CFG_PC6_PC7: CPGIO PC6/PC7 enable uart function and enable input
		analog_write(0x10, /*(analog_read(0x10) & 0xf0) |*/ PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
		BM_CLR(reg_gpio_gpio_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 as gpio
		//BM_CLR(reg_gpio_config_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 keyscan function
		//BM_SET(reg_gpio_ie(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);  // enable input
#endif // USE_INT_UART
#if (SWIRE_OFF)
		reg_gpio_gpio_func(GPIO_PA0) |= GPIO_PA0 & 0xff; // GPIO_PA0/SWS set gpio
#endif // SWIRE_OFF
	} else { // CHIP_TYPE == MCU_CORE_8269
#if (USE_USB_CDC)
		//-------------------------- USB pins 8269
#if USE_USB
#define PE2_FUNC	AS_USB
#define PE3_FUNC	AS_USB
#define PE2_INPUT_ENABLE	1
#define PE3_INPUT_ENABLE	1
#define PULL_WAKEUP_SRC_PE2           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DM
#define PULL_WAKEUP_SRC_PE3           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DP
#endif
#endif // USE_USB_CDC
		// USB-DM: PULL_WAKEUP_SRC_PE2 PM_PIN_PULLDOWN_100K
		analog_write(0x08, (analog_read (0x08) & 0x0f) | PM_PIN_PULLDOWN_100K<<4);
		// PE2_FUNC	AS_USB, PE3_FUNC	AS_USB
		BM_CLR(reg_gpio_gpio_func(GPIO_PE2), (GPIO_PE2 | GPIO_PE3) & 0xff); // disable PE2/PE3 as gpio
		// PE2_INPUT_ENABLE	1, PE3_INPUT_ENABLE	1
		//BM_SET(reg_gpio_ie(GPIO_PE2), (GPIO_PE2 | GPIO_PE3) & 0xFF);  // enable input
#if	(USE_INT_UART)
		//-------------------------- UART pins 8269
		// UART_GPIO_CFG_PC2_PC3:  CPGIO PC2/PC3 enable uart function and enable input
		analog_write(0x0f, /* (analog_read(0x0f) & 0xf0) | */ PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
		BM_CLR(reg_gpio_gpio_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		BM_SET(reg_gpio_config_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		//BM_SET(reg_gpio_ie(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);  //enable input
#endif // USE_INT_UART
#if (SWIRE_OFF)
		reg_gpio_gpio_func(GPIO_PB0) |= GPIO_PB0 & 0xff; // GPIO_PB0/SWS set gpio
#endif
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
#if (USE_USB_CDC)
	/* Initialize usb cdc */
	USB_Init();
	USBCDC_RxBufSet((unsigned char *)&urxb.uc);
#if !(defined(USB_TX_CALLBACK) || defined(USB_RX_CALLBACK))
	USBCDC_CBSet(USBCDC_RxCb, USBCDC_TxCb);
#endif
	usb_dp_pullup_enable();
	reg_irq_en = 1;
#endif // USE_USB_CDC
	/////////////////////////// app floader /////////////////////////////
	u16 crc16;
	while(1) {
#if MODULE_WATCHDOG_ENABLE
		WATCHDOG_CLEAR;  //in case of watchdog timeout
#endif
#if (USE_USB_CDC)
//		if(usb_flg & 1) { // USB-COM Open: DTR = 1
			if(utxb.len == 0 && urxb.len) { // new command?
				cmd_decode();
				urxb.len = 0;
				if(utxb.len) {
					crc16 = crcFast(utxb.uc, utxb.len);
					utxb.uc[utxb.len++] = crc16;
					utxb.uc[utxb.len++] = crc16 >> 8;
					USBCDC_DataSend((unsigned char *)&utxb.uc, utxb.len);
				}
			}
//		}
#endif // USE_USB_CDC
#if	(USE_INT_UART)
		if((reg_dma_tx_rdy0 & FLD_DMA_UART_TX) == 0
		&& (reg_dma_irq_src & FLD_DMA_UART_RX) != 0) { // new command?
			utxb.len = 0;
			cmd_decode();
			reg_dma_irq_src = FLD_DMA_UART_RX | FLD_DMA_UART_TX;
			if(utxb.len) {
				crc16 = crcFast(utxb.uc, utxb.len);
				utxb.uc[utxb.len++] = crc16;
				utxb.uc[utxb.len++] = crc16 >> 8;
				reg_dma_tx_rdy0 |= FLD_DMA_UART_TX; // start tx
			}
		}
#endif // USE_INT_UART
	}
	REG_ADDR8(0x6f) = 0x20;   // mcu reboot
	while (1);
}
