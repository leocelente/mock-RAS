/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_can.h"
#include "Fake.h"
#include "queue.h"
#include "zenith.h"
#include "CANZenTool.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan;
uint32_t mailbox;

uint8_t system_config_register[8];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for grupo1Task */
osThreadId_t grupo1TaskHandle;
uint32_t grupo1TaskBuffer[128];
osStaticThreadDef_t grupo1TaskControlBlock;
const osThreadAttr_t grupo1Task_attributes = { .name = "grupo1Task", .cb_mem =
		&grupo1TaskControlBlock, .cb_size = sizeof(grupo1TaskControlBlock),
		.stack_mem = &grupo1TaskBuffer[0], .stack_size =
				sizeof(grupo1TaskBuffer), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for grupo2Task */
osThreadId_t grupo2TaskHandle;
uint32_t grupo2TaskBuffer[128];
osStaticThreadDef_t grupo2TaskControlBlock;
const osThreadAttr_t grupo2Task_attributes = { .name = "grupo2Task", .cb_mem =
		&grupo2TaskControlBlock, .cb_size = sizeof(grupo2TaskControlBlock),
		.stack_mem = &grupo2TaskBuffer[0], .stack_size =
				sizeof(grupo2TaskBuffer), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for canGKTask */
osThreadId_t canGKTaskHandle;
uint32_t canGKTaskBuffer[128];
osStaticThreadDef_t canGKTaskControlBlock;
const osThreadAttr_t canGKTask_attributes = { .name = "canGKTask", .cb_mem =
		&canGKTaskControlBlock, .cb_size = sizeof(canGKTaskControlBlock),
		.stack_mem = &canGKTaskBuffer[0], .stack_size = sizeof(canGKTaskBuffer),
		.priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for storageTask */
osThreadId_t storageTaskHandle;
uint32_t storageTaskBuffer[128];
osStaticThreadDef_t storageTaskControlBlock;
const osThreadAttr_t storageTask_attributes = { .name = "storageTask", .cb_mem =
		&storageTaskControlBlock, .cb_size = sizeof(storageTaskControlBlock),
		.stack_mem = &storageTaskBuffer[0], .stack_size =
				sizeof(storageTaskBuffer), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for thermalCtrlTask */
osThreadId_t thermalCtrlTaskHandle;
uint32_t thermalCtrlTaskBuffer[128];
osStaticThreadDef_t thermalCtrlTaskControlBlock;
const osThreadAttr_t thermalCtrlTask_attributes = { .name = "thermalCtrlTask",
		.cb_mem = &thermalCtrlTaskControlBlock, .cb_size =
				sizeof(thermalCtrlTaskControlBlock), .stack_mem =
				&thermalCtrlTaskBuffer[0], .stack_size =
				sizeof(thermalCtrlTaskBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for stimulusTask */
osThreadId_t stimulusTaskHandle;
uint32_t stimulusTaskBuffer[128];
osStaticThreadDef_t stimulusTaskControlBlock;
const osThreadAttr_t stimulusTask_attributes = { .name = "stimulusTask",
		.cb_mem = &stimulusTaskControlBlock, .cb_size =
				sizeof(stimulusTaskControlBlock), .stack_mem =
				&stimulusTaskBuffer[0],
		.stack_size = sizeof(stimulusTaskBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for canEventQueue */
osMessageQueueId_t canEventQueueHandle;
uint8_t canEventQueueBuffer[10 * sizeof(CAN_Event_t)];
osStaticMessageQDef_t canEventQueueControlBlock;
const osMessageQueueAttr_t canEventQueue_attributes = { .name = "canEventQueue",
		.cb_mem = &canEventQueueControlBlock, .cb_size =
				sizeof(canEventQueueControlBlock), .mq_mem =
				&canEventQueueBuffer, .mq_size = sizeof(canEventQueueBuffer) };
/* Definitions for grupo1Queue */
osMessageQueueId_t grupo1QueueHandle;
uint8_t grupo1QueueBuffer[2 * 64];
osStaticMessageQDef_t grupo1QueueControlBlock;
const osMessageQueueAttr_t grupo1Queue_attributes = { .name = "grupo1Queue",
		.cb_mem = &grupo1QueueControlBlock, .cb_size =
				sizeof(grupo1QueueControlBlock), .mq_mem = &grupo1QueueBuffer,
		.mq_size = sizeof(grupo1QueueBuffer) };
/* Definitions for grupo2Queue */
osMessageQueueId_t grupo2QueueHandle;
uint8_t grupo2QueueBuffer[12 * 12];
osStaticMessageQDef_t grupo2QueueControlBlock;
const osMessageQueueAttr_t grupo2Queue_attributes = { .name = "grupo2Queue",
		.cb_mem = &grupo2QueueControlBlock, .cb_size =
				sizeof(grupo2QueueControlBlock), .mq_mem = &grupo2QueueBuffer,
		.mq_size = sizeof(grupo2QueueBuffer) };
/* Definitions for grupo1Timer */
osTimerId_t grupo1TimerHandle;
osStaticTimerDef_t grupo1TimerControlBlock;
const osTimerAttr_t grupo1Timer_attributes = { .name = "grupo1Timer", .cb_mem =
		&grupo1TimerControlBlock, .cb_size = sizeof(grupo1TimerControlBlock), };
/* Definitions for grupo2Timer */
osTimerId_t grupo2TimerHandle;
osStaticTimerDef_t grupo2TimerControlBlock;
const osTimerAttr_t grupo2Timer_attributes = { .name = "grupo2Timer", .cb_mem =
		&grupo2TimerControlBlock, .cb_size = sizeof(grupo2TimerControlBlock), };
/* Definitions for uartSemaphore */
osSemaphoreId_t uartSemaphoreHandle;
osStaticSemaphoreDef_t uartSemaphoreControlBlock;
const osSemaphoreAttr_t uartSemaphore_attributes = { .name = "uartSemaphore",
		.cb_mem = &uartSemaphoreControlBlock, .cb_size =
				sizeof(uartSemaphoreControlBlock), };
/* Definitions for cfgRegSemaphore */
osSemaphoreId_t cfgRegSemaphoreHandle;
osStaticSemaphoreDef_t cfgRegSemaphoreControlBlock;
const osSemaphoreAttr_t cfgRegSemaphore_attributes = {
		.name = "cfgRegSemaphore", .cb_mem = &cfgRegSemaphoreControlBlock,
		.cb_size = sizeof(cfgRegSemaphoreControlBlock), };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartGrupo1(void *argument);
void StartGrupo2(void *argument);
void StartCanGK(void *argument);
void StartStorage(void *argument);
void StartThermalCtrl(void *argument);
void StartStimulus(void *argument);
void Grupo1TimerCallback(void *argument);
void Grupo2TimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
	debug("[*] STK_OF:%s [*]", pcTaskName);
}
/* USER CODE END 4 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of uartSemaphore */
	uartSemaphoreHandle = osSemaphoreNew(1, 1, &uartSemaphore_attributes);

	/* creation of cfgRegSemaphore */
	cfgRegSemaphoreHandle = osSemaphoreNew(1, 1, &cfgRegSemaphore_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of grupo1Timer */
	grupo1TimerHandle = osTimerNew(Grupo1TimerCallback, osTimerPeriodic, NULL,
			&grupo1Timer_attributes);

	/* creation of grupo2Timer */
	grupo2TimerHandle = osTimerNew(Grupo2TimerCallback, osTimerPeriodic, NULL,
			&grupo2Timer_attributes);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of canEventQueue */
	canEventQueueHandle = osMessageQueueNew(10, sizeof(CAN_Event_t),
			&canEventQueue_attributes);

	/* creation of grupo1Queue */
	grupo1QueueHandle = osMessageQueueNew(2, 64, &grupo1Queue_attributes);

	/* creation of grupo2Queue */
	grupo2QueueHandle = osMessageQueueNew(12, 12, &grupo2Queue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of grupo1Task */
	grupo1TaskHandle = osThreadNew(StartGrupo1, NULL, &grupo1Task_attributes);

	/* creation of grupo2Task */
	grupo2TaskHandle = osThreadNew(StartGrupo2, NULL, &grupo2Task_attributes);

	/* creation of canGKTask */
	canGKTaskHandle = osThreadNew(StartCanGK, NULL, &canGKTask_attributes);

	/* creation of storageTask */
	storageTaskHandle = osThreadNew(StartStorage, NULL,
			&storageTask_attributes);

	/* creation of thermalCtrlTask */
	thermalCtrlTaskHandle = osThreadNew(StartThermalCtrl, NULL,
			&thermalCtrlTask_attributes);

	/* creation of stimulusTask */
	stimulusTaskHandle = osThreadNew(StartStimulus, NULL,
			&stimulusTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	osThreadExit();
	/* Infinite loop */
	for (;;) {
		osDelay(100);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartGrupo1 */
/**
 * @brief Function implementing the grupo1Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGrupo1 */
void StartGrupo1(void *argument) {
	/* USER CODE BEGIN StartGrupo1 */
	sensor1_init();
	sensor1_init();
	osTimerStart(grupo1TimerHandle, 1000);
	/* Infinite loop */
	uint8_t data[] = { 0xDE, 0xAD, 0xBE, 0xEF };
	for (;;) {
		osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
		uint8_t d = 0x00;
		sensor1_get(&d);
		osMessageQueuePut(grupo1QueueHandle, &data, 0, 100);
	}
	/* USER CODE END StartGrupo1 */
}

/* USER CODE BEGIN Header_StartGrupo2 */
/**
 * @brief Function implementing the grupo2Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGrupo2 */
void StartGrupo2(void *argument) {
	/* USER CODE BEGIN StartGrupo2 */
	sensor2_init();
	sensor2_init();
	osTimerStart(grupo2TimerHandle, 100);
	uint32_t last_count = 0;
	uint8_t data[] = { 0xDE, 0xAD, 0xBE, 0xEF };
	uint8_t counter = 0;
	/* Infinite loop */
	for (;;) {
		osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
		data[sizeof(data) - 1] = counter++;
		uint32_t new_count = osMessageQueueGetCount(grupo2QueueHandle);
		if (new_count < last_count && new_count != 0) {
			osDelay(1);
		} else {
			uint8_t d = 0x00;
			sensor2_get(&d);
			osMessageQueuePut(grupo2QueueHandle, &data, 0, 10);
		}
		last_count = new_count;
	}
	/* USER CODE END StartGrupo2 */
}

/* USER CODE BEGIN Header_StartCanGK */
/**
 * @brief Function implementing the canGKTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanGK */
void StartCanGK(void *argument) {
	/* USER CODE BEGIN StartCanGK */
	CAN_Event_t can_event = { 0 };
	CAN_TxHeaderTypeDef header = CANZenTool_writeStdCanFrame(8, 0xAD, true);

	/* Infinite loop */
	for (;;) {
		osMessageQueueGet(canEventQueueHandle, &can_event, 0, osWaitForever);
		debug("CAN Event: [%.3X] %s", can_event.ID,
				can_event.type ? "TX" : "RX");
		switch (can_event.type) {
		case TX:
			CANZenTool_sendCanFrameMsg(&hcan, &header, can_event.data,
					&mailbox);
			break;
		case RX:
			debug("MSG: [%.2X%.2X%.2X%.2X %.2X%.2X%.2X%.2X]", can_event.data[0],
					can_event.data[1], can_event.data[2], can_event.data[3],
					can_event.data[4], can_event.data[5], can_event.data[6],
					can_event.data[7])
			;
			break;
		default:
			break;
		}
	}
	/* USER CODE END StartCanGK */
}

/* USER CODE BEGIN Header_StartStorage */
/**
 * @brief Function implementing the storageTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStorage */
void StartStorage(void *argument) {
	/* USER CODE BEGIN StartStorage */
	storage1_init();
	storage2_init();
	/* Infinite loop */
	uint8_t data[4] = { 0 };
	uint8_t buffer[40] = { 0 };
	for (;;) {
		osMessageQueueGet(grupo1QueueHandle, &data, NULL, osWaitForever);
		storage1_save(sizeof(data), data);
		storage2_save(sizeof(data), data);
		uint32_t g2count = osMessageQueueGetCount(grupo2QueueHandle);
		while (g2count--) {
			int offset = sizeof(data) * (sizeof(buffer) - g2count);
			osMessageQueueGet(grupo2QueueHandle, buffer + offset, 0, 10);
		}
		debug("IMU Queue flushed");
		storage1_save(sizeof(buffer), buffer);
		storage2_save(sizeof(buffer), buffer);
	}
	/* USER CODE END StartStorage */
}

/* USER CODE BEGIN Header_StartThermalCtrl */
/**
 * @brief Function implementing the thermalCtrlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartThermalCtrl */
void StartThermalCtrl(void *argument) {
	/* USER CODE BEGIN StartThermalCtrl */
	/* Infinite loop */
	for (;;) {
		actuator_get();
		osDelay(500);
		actuator_set(1);
		osDelay(500);
	}
	/* USER CODE END StartThermalCtrl */
}

/* USER CODE BEGIN Header_StartStimulus */
/**
 * @brief Function implementing the stimulusTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStimulus */
void StartStimulus(void *argument) {
	/* USER CODE BEGIN StartStimulus */
	/* Infinite loop */
	uint8_t buffer[8] = { 0, [1]=0x55 };
	CAN_Event_t can_ev_tx = { //
			.ID = 0x123, //
					.RTR = CAN_RTR_DATA, //
					.size = 8, //
					.type = TX, //
			};
	uint8_t counter = 0;
	for (;;) {
		osDelay(2000);
		buffer[sizeof(buffer) - 1] = counter++;
		memcpy(can_ev_tx.data, buffer, can_ev_tx.size);
		osMessageQueuePut(canEventQueueHandle, &can_ev_tx, 0, 100);
	}
	/* USER CODE END StartStimulus */
}

/* Grupo1TimerCallback function */
void Grupo1TimerCallback(void *argument) {
	/* USER CODE BEGIN Grupo1TimerCallback */
	osThreadFlagsSet(grupo1TaskHandle, 0x00000001U);
	/* USER CODE END Grupo1TimerCallback */
}

/* Grupo2TimerCallback function */
void Grupo2TimerCallback(void *argument) {
	/* USER CODE BEGIN Grupo2TimerCallback */
	osThreadFlagsSet(grupo2TaskHandle, 0x00000001U);
	/* USER CODE END Grupo2TimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t buffer[8];
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, buffer);
	CAN_Event_t event;
	event.RTR = header.RTR;
	event.ID = header.StdId;
	event.size = (uint8_t) header.DLC;
	event.type = RX;
	memcpy(event.data, buffer, event.size);
	osMessageQueuePut(canEventQueueHandle, &event, 0, 0);
}

int run_commmand(ras_cmd_t cmd, uint8_t data[8]) {
	switch (cmd) {
	case TEST:
		return OK;
	case R_CFG:
		CAN_Event_t response = { //
				.ID = 0x123, //
						.RTR = CAN_RTR_DATA, //
						.size = 8, //
						.type = TX, //
				};
		if (osMutexAcquire(cfgRegSemaphoreHandle, osWaitForever) == osOK) {
			memcpy(response.data, system_config_register, response.size);
			osMutexRelease(cfgRegSemaphoreHandle);
		}
		return osMessageQueuePut(canEventQueueHandle, &response, 1, 100);
	case W_CFG:
		if (osMutexAcquire(cfgRegSemaphoreHandle, 250) == osOK) {
			memcpy(system_config_register, data,
					sizeof(system_config_register));
			osMutexRelease(cfgRegSemaphoreHandle);
			return OK;
		} else {
			return FAIL;
		}
//	case
		return OK;
	}
	return OK;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
