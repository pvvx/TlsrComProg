/********************************************************************************************************
 * @file     main.c
 *
 * @brief    OTA-Boot for TLSR8366 chips (Flash addr 0x72000)
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "common.h"
#include "spi_i.h"
#include "flash.h"

#define  DBG_LED_IND 		0   //for led DEBUG ota boot bin running statas
#ifndef			NEW_FW_SIZE
#define			NEW_FW_SIZE			128    //128k
#endif

#ifndef			NEW_FW_ADR
#define			NEW_FW_ADR			0x20000
#endif

#ifndef			OTA_FLG_ADR
#define			OTA_FLG_ADR			0x73000
#endif

//now my remote control  PC2 high LED on, PC2 low LED off
//see <<ble_gpio_lookuptable>> for GPIO register set
//notice that: 8266 output setting: OEN = 0 means output enable
//PC2 default is GPIO

#define LED_INIT_OUTPUT		do{REG_ADDR8(0x591) &= ~BIT(2); REG_ADDR8(0x592) &= ~BIT(2); }while(0)

#define LED_HIGH			( REG_ADDR8(0x593) |= BIT(2) )
#define LED_LOW				( REG_ADDR8(0x593) &= ~BIT(2) )
#define LED_TOGGLE			( REG_ADDR8(0x593) ^= BIT(2) )

u32 buff[256>>3];//  __attribute__ ((aligned (4)));
u32 check_buff[256>>3];//  __attribute__ ((aligned (4)));

_attribute_ram_code_ int ota_boot(void) {

	reg_irq_en = 0;

#if(DBG_LED_IND) //for debug : indicate that RF transforming OK, ota boot begin
	LED_INIT_OUTPUT;

	LED_HIGH;
	sleep_us(3000000);
	LED_LOW;
#endif

	flash_read_page(NEW_FW_ADR, 32, (u8 *)buff);
	int n_firmware = (int)buff[0x18>>3];
	if (n_firmware < 256
			|| n_firmware > (NEW_FW_SIZE << 10)
			|| buff[0x08>>3] != 0x544C4E4B) {
		//firmware too big, err
#if(DBG_LED_IND)  //for debug : indicate that firmware size ERR
		LED_HIGH;
#endif
		goto Error;
		//		write_reg16(0x8000,0x55aa);  //for debug
		//		while(1);
	}

	for (int i = 4096; i < n_firmware; i += 256) {
		if ((i & 0xfff) == 0) //new sector begin Addr, need erase
		{
			flash_erase_sector(i);
		}

		flash_read_page(NEW_FW_ADR + i, 256, (u8 *)buff); //read data  from 0x20000 ~ 0x30000
		flash_write_page(i, 256, (u8 *)buff); //write data  to  0x00000 ~ 0x10000
		flash_read_page(i, 256, (u8 *)check_buff); //read data to check if write OK
		for (int j = 0; j < (256>>3); j++) {
			if (buff[j] != check_buff[j]) { //write data not OK

#if(DBG_LED_IND)  //for debug : indicate that flash write ERR happens
				LED_HIGH;
#endif

				i &= 0xfff000; //back to sector begin adr, to rewrite
				i -= 256;
				break;
			}
		}
	}

	for (int i = 0; i < 4096; i += 256) //first 4K
	{
		if ((i & 0xfff) == 0) //new sector begin Addr, need erase
		{
			flash_erase_sector(i);
		}

		flash_read_page(NEW_FW_ADR + i, 256, (u8 *)buff); //read data  from 0x20000 ~ 0x30000
		flash_write_page(i, 256, (u8 *)buff); //write data  to  0x00000 ~ 0x10000
		flash_read_page(i, 256, (u8 *)check_buff); //read data to check if write OK
		for (int j = 0; j < (256>>3); j++) {
			if (buff[j] != check_buff[j]) { //write data not OK

#if(DBG_LED_IND)  //for debug : indicate that flash write ERR happens
				LED_HIGH;
#endif

				i &= 0xfff000; //back to sector begin adr, to rewrite
				i -= 256;
				break;
			}
		}
	}
Error:
	buff[0] = 0;
	flash_write_page(OTA_FLG_ADR, 1, (u8 *)buff); //clear OTA flag
#if 1
	if (n_firmware != -1)
		for(int i = (n_firmware - 1) & 0x1f000; i >= 0; i -= 4096) { //erase data on flash for next OTA
			flash_erase_sector(NEW_FW_ADR + i);
		}
#endif

#if(DBG_LED_IND)  //for debug : indicate that ota boot running OK
	for(int i=0; i< 4; i++) { //1Hz shine for 4 S
		LED_HIGH;
		sleep_us(500000);
		LED_LOW;
		sleep_us(500000);
	}
#endif

	REG_ADDR8(0x6f) = 0x20; // mcu reboot
	while(1);
}
