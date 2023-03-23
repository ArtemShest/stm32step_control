/*
 * w5100s.c
 *
 *  Created on: Jan 20, 2023
 *      Author: artem
 */

#include "w5100s.h"
#include "main.h"

//-----------------------------------------------

extern SPI_HandleTypeDef hspi3;

//-----------------------------------------------

extern W5100s w5100s;

uint8_t macaddr[6]=MAC_ADDR;

extern uint8_t ipaddr[4];
extern uint8_t ipgate[4];
extern uint8_t ipmask[4];
extern uint16_t local_port;

w5100s_State w5100s_check()
{
	  uint8_t state = w5100s_socketState();
	  if(w5100s.state != state) //server state change
	  {
		  if (state == Listen) { }
		  else if (state == Connect)
		  {
			  w5100s_readMsg();
		  }
		  else if (state == Close_Wait)
		  {
			  w5100s_socketReOpen();
		  }
		  else if (state == Close)
		  {
			  w5100s_socketOpen();
			  w5100s_socketListen();
		  }

		  w5100s.state = state;
		  return w5100s.state;
	  }
	  else return Continue;

}

void w5100s_readMsg()
{
	w5100s.recieve_msg_size = w5100s_getSizeMsg();
	if(w5100s.recieve_msg_size > 0)
	{
		uint16_t offset;
		uint16_t offset_masked;
		uint16_t start_address;

		offset =  w5100s_readReg(REG_S0_RX_RD0) << 8;
		offset += w5100s_readReg(REG_S0_RX_RD1);

		offset_masked &= S0_RX_MASK;
		start_address = RX_BASE_ADDR; // + offset_masked;

		for (uint16_t i = 0; i< w5100s.recieve_msg_size; i++)
		{
			w5100s.recieve_msg[i] = w5100s_readReg(start_address + i);
		}
		w5100s.is_new_command = 1;

		offset += w5100s.recieve_msg_size;
		w5100s_writeReg(REG_S0_RX_RD0, offset >> 8);
		w5100s_writeReg(REG_S0_RX_RD1, offset);

		w5100s_writeReg(REG_S0_CR, SOCKET_RECV);
	}
}


void w5100s_sendAns(int coordA, int coordB, int64_t coordAmm, int64_t coordBmm)
{
	uint16_t tx_offset  = (w5100s_readReg(REG_S0_TX_RD0) << 8) +  w5100s_readReg(REG_S0_TX_RD1);
	tx_offset = (tx_offset & S0_TX_MASK);
	uint16_t tx_start_addr = tx_offset + TX_BASE_ADDR;
	uint8_t data_length = 24;

	uint8_t data[] = {coordA>>24, coordA>>16, coordA>>8, coordA, coordB>>24, coordB>>16, coordB>>8, coordB,
		coordAmm>>56, coordAmm>>48, coordAmm>>40, coordAmm >> 32, coordAmm>>24, coordAmm>>16, coordAmm>>8, coordAmm,
		coordBmm>>56, coordBmm>>48, coordBmm>>40, coordBmm >> 32, coordBmm>>24, coordBmm>>16, coordBmm>>8, coordBmm, };

	uint16_t tx_end_addr = (w5100s_readReg(REG_S0_TX_WR0) << 8) +  w5100s_readReg(REG_S0_TX_WR1);
	tx_end_addr += data_length;
	w5100s_writeReg(REG_S0_TX_WR0, tx_end_addr >> 8);
	w5100s_writeReg(REG_S0_TX_WR1, tx_end_addr);

	for (int i = 0; i < data_length; i++)
	{
		 if (tx_start_addr  > (TX_BASE_ADDR + S0_TX_MASK))
			 tx_start_addr = TX_BASE_ADDR;
		 w5100s_writeReg(tx_start_addr, data[i]) ;
		 tx_start_addr++;
	}
	w5100s_writeReg(REG_S0_CR, SOCKET_SEND);

}

uint16_t w5100s_getSizeMsg()
{
	uint16_t dataSize;
	dataSize = w5100s_readReg(REG_S0_RX_RSR0) << 8;
	dataSize += w5100s_readReg(REG_S0_RX_RSR1);
	return dataSize;
}

uint8_t w5100s_socketState() //состояние сокета
{
	return w5100s_readReg(REG_S0_SR);
}

void w5100s_socketReOpen()
{
	w5100s_socketClose();
	w5100s_socketOpen();
	w5100s_socketListen();
}

uint8_t w5100s_readReg(uint16_t address)
{
	uint8_t buf[] = {W5100_READ, address >> 8, address};
	uint8_t *rbyte;
    SS_SELECT();
    HAL_SPI_Transmit(&hspi3, buf, 3, 1000);
	HAL_SPI_Receive(&hspi3, &rbyte, 1, 1000);
	SS_DESELECT();
	return rbyte;
}

void w5100s_writeReg(uint16_t address, uint8_t data)
{
	uint8_t buf[] = {W5100_WRITE, address >> 8, address, data};
	SS_SELECT();
	HAL_SPI_Transmit(&hspi3, buf, 4, 0xFFFFFFFF);
	SS_DESELECT();
}


void w5100s_init()
{
	printf ("w5100s start init\r\n");



	//----------------- hard reset ------------------------

	HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, RESET);
	  HAL_Delay(10);
	HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, SET);

//	ETH_RST_GPIO_Port->BSRR = (uint32_t)ETH_RST_Pin << 16U;
//	ETH_RST_GPIO_Port->BSRR = ETH_RST_Pin;
	//-----------------------------------------------------

	  HAL_Delay(100);

	//-------- 1. Mode Register : soft reset --------------
	w5100s_writeReg(REG_MR, 0x80); HAL_Delay(1);

	  HAL_Delay(1);
	  printf ("1. Mode Register\r\n");
	  printf ("\t write %d to REG_MR 0x%.4x; read %d\r\n", 0x80, REG_MR,  w5100s_readReg(REG_MR));
	  HAL_Delay(1);
	//-----------------------------------------------------

	//---- 2. Interrupt Mask Register : interrupt enable --
	w5100s_writeReg(REG_IMR, S0_INT); HAL_Delay(1);
    printf ("\t write %d to REG_IMR 0x%.4x; read %d\r\n", S0_INT, REG_IMR, w5100s_readReg(REG_IMR));
	//-----------------------------------------------------

	//---------- 3. Retry Time-value Register -------------
	w5100s_writeReg(REG_RTR0, 0x0f); HAL_Delay(1);
    printf ("\t write %d to REG_RTR0 0x%.4x; read %d\r\n", 0x0f, REG_RTR0, w5100s_readReg(REG_RTR0));
	w5100s_writeReg(REG_RTR1, 0xA0); HAL_Delay(1);
    printf ("\t write %d to REG_RTR1 0x%.4x; read %d\r\n", 0xA0, REG_RTR1, w5100s_readReg(REG_RTR1));
	//-----------------------------------------------------

	//---------- 4. Retry Count Register ------------------
	w5100s_writeReg(REG_RCR, 0x01); HAL_Delay(1);
    printf ("\t write %d to REG_RCR 0x%.4x; read %d\r\n", 0x01, REG_RCR, w5100s_readReg(REG_RCR));
	//-----------------------------------------------------

	//---------------- 5. IP-gate address -----------------
	w5100s_writeReg(REG_GWR0,ipgate[0]); HAL_Delay(1);
    	printf ("\t write %d to REG_GWR0 0x%.4x; read %d\r\n", ipgate[0], REG_GWR0, w5100s_readReg(REG_GWR0));
	w5100s_writeReg(REG_GWR1,ipgate[1]); HAL_Delay(1);
    	printf ("\t write %d to REG_GWR1 0x%.4x; read %d\r\n", ipgate[1], REG_GWR0, w5100s_readReg(REG_GWR1));
	w5100s_writeReg(REG_GWR2,ipgate[2]); HAL_Delay(1);
    	printf ("\t write %d to REG_GWR2 0x%.4x; read %d\r\n", ipgate[2], REG_GWR0, w5100s_readReg(REG_GWR2));
	w5100s_writeReg(REG_GWR3,ipgate[3]); HAL_Delay(1);
    	printf ("\t write %d to REG_GWR3 0x%.4x; read %d\r\n", ipgate[3], REG_GWR0, w5100s_readReg(REG_GWR3));
	//-----------------------------------------------------

	//---------------- 6. MAC address ---------------------
	w5100s_writeReg(REG_SHAR0,macaddr[0]); HAL_Delay(1);
    	printf ("\t write %d to REG_SHAR0 0x%.4x; read %d\r\n", macaddr[0], REG_SHAR0, w5100s_readReg(REG_SHAR0));
	w5100s_writeReg(REG_SHAR1,macaddr[1]); HAL_Delay(1);
    	printf ("\t write %d to REG_SHAR1 0x%.4x; read %d\r\n", macaddr[1], REG_SHAR1, w5100s_readReg(REG_SHAR1));
	w5100s_writeReg(REG_SHAR2,macaddr[2]); HAL_Delay(1);
    	printf ("\t write %d to REG_SHAR2 0x%.4x; read %d\r\n", macaddr[2], REG_SHAR2, w5100s_readReg(REG_SHAR2));
	w5100s_writeReg(REG_SHAR3,macaddr[3]); HAL_Delay(1);
		printf ("\t write %d to REG_SHAR3 0x%.4x; read %d\r\n", macaddr[3], REG_SHAR3, w5100s_readReg(REG_SHAR3));
	w5100s_writeReg(REG_SHAR4,macaddr[4]); HAL_Delay(1);
    	printf ("\t write %d to REG_SHAR4 0x%.4x; read %d\r\n", macaddr[4], REG_SHAR4, w5100s_readReg(REG_SHAR4));
	w5100s_writeReg(REG_SHAR5,macaddr[5]); HAL_Delay(1);
    	printf ("\t write %d to REG_SHAR5 0x%.4x; read %d\r\n", macaddr[5], REG_SHAR5, w5100s_readReg(REG_SHAR5));
	//-----------------------------------------------------

	//---------------- 7. Mask address --------------------
	w5100s_writeReg(REG_SUBR0,ipmask[0]); HAL_Delay(1);
	w5100s_writeReg(REG_SUBR1,ipmask[1]); HAL_Delay(1);
	w5100s_writeReg(REG_SUBR2,ipmask[2]); HAL_Delay(1);
	w5100s_writeReg(REG_SUBR3,ipmask[3]); HAL_Delay(1);
	//-----------------------------------------------------

	//----------------- 8. IP address ---------------------
	w5100s_writeReg(REG_SIPR0,ipaddr[0]); HAL_Delay(1);
	w5100s_writeReg(REG_SIPR1,ipaddr[1]); HAL_Delay(1);
	w5100s_writeReg(REG_SIPR2,ipaddr[2]); HAL_Delay(1);
	w5100s_writeReg(REG_SIPR3,ipaddr[3]); HAL_Delay(1);


    printf ("\t write %d to REG_SIPR3 0x%.4x; read %d\r\n", ipaddr[3], REG_SIPR3, w5100s_readReg(REG_SIPR3));
	//-----------------------------------------------------

	//----------------- 9. Setting port -------------------
	w5100s_writeReg(REG_S0_PORT0, 0x00); HAL_Delay(1);
	w5100s_writeReg(REG_S0_PORT1, 10);	 HAL_Delay(1);
	//-----------------------------------------------------

	//--- 10. RX memory of Socket_0 = 8kB, others = 0kB ---
	w5100s_writeReg(REG_RMSR, 0x03); HAL_Delay(1); // RX
	w5100s_writeReg(REG_TMSR, 0x03); HAL_Delay(1); // TX
	//-----------------------------------------------------

	//-------------- 11. Interrupt enable -----------------
	//w5100s_writeReg(REG_S0_IMR, 0x01); HAL_Delay(1);
	//-----------------------------------------------------

	//----------- 12. Setting TCP protocol ----------------
	w5100s_writeReg(REG_S0_MR, TCP_MODE); HAL_Delay(1);
	//-----------------------------------------------------

	//---------------- 13. Socket OPEN  -------------------
	w5100s_writeReg(REG_S0_CR, SOCKET_OPEN); HAL_Delay(1);
	printf ("\t Socket status %d\r\n", w5100s_readReg(REG_S0_SR));
	//-----------------------------------------------------

	//---------------- 13. Socket LISTEN  -----------------
	w5100s_writeReg(REG_S0_CR, TCP_LISTEN); HAL_Delay(1);
		printf ("\t Socket status %d\r\n", w5100s_readReg(REG_S0_SR));
	//-----------------------------------------------------

	printf ("\tServer is listening! \r\n");

}

void w5100s_socketClose()
{
	w5100s_writeReg(REG_S0_CR, SOCKET_CLOSE);
}

void w5100s_socketOpen()
{
	w5100s_writeReg(REG_S0_CR, SOCKET_OPEN);
}

void w5100s_socketListen()
{
	w5100s_writeReg(REG_S0_CR, TCP_LISTEN);
}
