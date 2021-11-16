/*
 * zenith.h
 *
 *  Created on: Oct 10, 2021
 *      Author: leocelente
 */

#ifndef INC_ZENITH_H_
#define INC_ZENITH_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "cmsis_os2.h"
extern osSemaphoreId_t uartSemaphoreHandle;

#ifdef NDEBUG
    #define debug(fmt, ...)
#else
#define debug(fmt, ...) \
		if (osMutexAcquire(uartSemaphoreHandle, osWaitForever) == osOK) {\
			/*printf("%s:%s:%d: \t"fmt"\r\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);   */\
			printf(fmt"\r\n", ##__VA_ARGS__);   \
			osMutexRelease(uartSemaphoreHandle);\
		}
#endif

enum {
	OK, FAIL
};

#endif /* INC_ZENITH_H_ */
