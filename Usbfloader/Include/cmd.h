/*
 * cmd.h
 *
 *  Created on: 07.03.2020
 *      Author: pvvx
 */

#ifndef _CMD_H_
#define _CMD_H_

//-------------------------------
#define UART_DMA_BUFF_SIZE (DATA_BUFF_SIZE+16)
#define UART_RX_BUFF_SIZE	UART_DMA_BUFF_SIZE
#define UART_TX_BUFF_SIZE	UART_DMA_BUFF_SIZE

//=============================== CMD
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

//-------------------------------

extern dma_uart_buf_t urxb;
extern dma_uart_buf_t utxb;


#endif /* _CMD_H_ */
