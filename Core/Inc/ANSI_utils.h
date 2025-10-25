/*
 * ansi_utils.h
 *
 *  Created on: Oct 25, 2025
 *      Author: vunguyen
 */

#ifndef INC_ANSI_UTILS_H_
#define INC_ANSI_UTILS_H_

#define ANSI_GREEN 	"\033[1;1H\x1b[32m"
#define ANSI_END 	"\x1b[0m"
#define ANSI_YELLOW "\x1b[33m"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void clear_screen(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* INC_ANSI_UTILS_H_ */
