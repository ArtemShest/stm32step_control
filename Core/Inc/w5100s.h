/*
 * w5100s.h
 *
 *  Created on: Dec 22, 2022
 *      Author: artem
 */

#ifndef INC_W5100S_H_
#define INC_W5100S_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef enum
{
	Listen = 0x14,
	Connect = 0x17,
	Close_Wait = 0x1c,
	Close = 0x00,
	Continue = 0xff,
} w5100s_State;


typedef struct {
	w5100s_State state;
	uint8_t recieve_msg[100];
	uint16_t recieve_msg_size;
	uint8_t is_new_command;
} W5100s;

w5100s_State w5100s_check();
void w5100s_readMsg();
void w5100s_sendAns(int coordA, int coordB, int64_t coordAmm, int64_t coordBmm);
uint16_t w5100s_getSizeMsg();
uint8_t w5100s_socketState(); //состояние сокета
void w5100s_socketReOpen();
uint8_t w5100s_readReg(uint16_t addres);
void w5100s_writeReg(uint16_t addres, uint8_t data);
void w5100s_init();
void w5100s_socketClose();
void w5100s_socketOpen();
void w5100s_socketListen();

//--------------------------------------------------
#define W5100_READ		0x0F
#define W5100_WRITE		0xF0


#define CS_GPIO_PORT GPIOA
#define CS_PIN GPIO_PIN_15

#define SS_SELECT() HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET)
#define SS_DESELECT() HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET)

//--------------------------------------------------
#define MAC_ADDR {0x1,0x2,0x3,0x4,0x5,0x6}
//--------------------------------------------------
#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))
//--------------------------------------------------
#define BSB_COMMON 0x00
#define BSB_S0 0x01
#define BSB_S0_TX 0x02
#define BSB_S0_RX 0x03
//--------------------------------------------------
#define RWB_WRITE 1
#define RWB_READ 0
//--------------------------------------------------
#define OM_FDM0 0x00//режим передачи данных переменной длины
#define OM_FDM1 0x01//режим передачи данных по одному байту
#define OM_FDM2 0x02//режим передачи данных по два байта
#define OM_FDM3 0x03//режим передачи данных по четыре байта
//--------------------------------------------------
#define MR 0x0000//Mode Register
//--------------------------------------------------
//--------------------------------------------------

#define REG_SHAR0	0x0009 //mac
#define REG_SHAR1	0x000A
#define REG_SHAR2	0x000B
#define REG_SHAR3	0x000C
#define REG_SHAR4	0x000D
#define REG_SHAR5	0x000E

#define REG_MR 0x0000


#define REG_GWR0 0x0001	 //Gateway IP Address Register MSB
#define REG_GWR1 0x0002
#define REG_GWR2 0x0003
#define REG_GWR3 0x0004// LSB

#define REG_SUBR0	0x0005 // subnet musk
#define REG_SUBR1	0x0006
#define REG_SUBR2	0x0007
#define REG_SUBR3	0x0008

#define REG_SIPR0 0x000F//Source IP Address Register MSB
#define REG_SIPR1 0x0010
#define REG_SIPR2 0x0011
#define REG_SIPR3 0x0012// LSB
//--------------------------------------------------
#define REG_IMR		0x0016
#define REG_RTR0	0x0017
#define REG_RTR1	0x0018
#define REG_RCR		0x0019
#define REG_RMSR	0x001A
#define REG_TMSR	0x001B
#define REG_VERR	0x0080	// must be 0x51
// Socket_0 Registers
#define REG_S0_MR		0x0400
#define REG_S0_CR		0x0401
#define REG_S0_IR		0x0402
#define REG_S0_SR		0x0403
#define REG_S0_PORT0	0x0404
#define REG_S0_PORT1	0x0405
#define REG_S0_IMR		0x042C



#define REG_S0_RX_RD0	0x0428
#define REG_S0_RX_RD1	0x0429

#define REG_S0_RX_RSR0	0x0426
#define REG_S0_RX_RSR1	0x0427
#define REG_S0_RX_RD0	0x0428
#define REG_S0_RX_RD1	0x0429

// Rx/Tx Memory
#define TX_BASE_ADDR	0x4000
#define RX_BASE_ADDR	0x6000

#define REG_S0_TX_RD0	0x0422
#define REG_S0_TX_RD1	0x0423
#define REG_S0_TX_WR0	0x0424
#define REG_S0_TX_WR1	0x0425

#define S0_RXBUF_SIZE	8		// [kB]
#define S0_RX_MASK		((1024 * S0_RXBUF_SIZE) - 1)
#define S0_TXBUF_SIZE	8
#define S0_TX_MASK 		((1024 * S0_TXBUF_SIZE) - 1)
// Interrupt Mask Register
#define S0_INT			0x01
// Socket Registers
// Socket_N Mode Register
#define TCP_MODE		0x01
// Socket_N Command Register - Sn_CR
#define SOCKET_OPEN		0x01
#define TCP_LISTEN		0x02
#define SOCKET_CLOSE	0x10
#define SOCKET_SEND		0x20
#define SOCKET_RECV		0x40

// Socket_N Interrupt Mask & Socket_N Interrupt Registers
#define RECV			0x04


#endif /* INC_W5100S_H_ */
