/*
 * ansi_utils.c
 *
 *  Created on: Oct 25, 2025
 *      Author: vunguyen
 */
#include "ANSI_utils.h"

void clear_screen(UART_HandleTypeDef* huart)
{
	const char* CLEAR = "\033[2J\0";
	HAL_UART_Transmit(&huart, (uint8_t*)CLEAR, strlen(CLEAR), 100);
}



