/*
 * buffer.c
 *
 *  Created on: Mar 11, 2025
 *      Author: chifuchang
 */

#include "buffer.h"
#include <string.h>
#include "usart.h"


void send_Data(float time, float pos, float vel, float tau)
{
	char tx_buff[sizeof(float)];
	char msg[21];
	msg[0] = 'S';
	memcpy(tx_buff, (unsigned char *)&time, sizeof(float));
	msg[1] = tx_buff[0];
	msg[2] = tx_buff[1];
	msg[3] = tx_buff[2];
	msg[4] = tx_buff[3];
	memcpy(tx_buff, (unsigned char *)&pos, sizeof(float));
	msg[5] = tx_buff[0];
	msg[6] = tx_buff[1];
	msg[7] = tx_buff[2];
	msg[8] = tx_buff[3];
	memcpy(tx_buff,  (unsigned char *)&vel, sizeof(float));
	msg[9] = tx_buff[0];
	msg[10] = tx_buff[1];
	msg[11] = tx_buff[2];
	msg[12] = tx_buff[3];
	memcpy(tx_buff,  (unsigned char *)&tau, sizeof(float));
	msg[13] = tx_buff[0];
	msg[14] = tx_buff[1];
	msg[15] = tx_buff[2];
	msg[16] = tx_buff[3];
	msg[17] = 'E';
	/* checksum */
	msg[18] = 0;
	char checksum = 0;
	for (int i = 0; i < 18; i++)
	{
		checksum ^= msg[i];
	}
	msg[19] = checksum;
	msg[20] = '\r';
	msg[21] = '\n';
//	sprintf(message, "time: %.3f, pos: %.3f, vel: %.3f, tau: %.3f\r\n", time, pos, vel, tau);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, 22);
}
