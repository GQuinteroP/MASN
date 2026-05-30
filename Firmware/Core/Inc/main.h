/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_NRST_Pin GPIO_PIN_0
#define GPS_NRST_GPIO_Port GPIOC
#define LPUART_GPS_Tx_Pin GPIO_PIN_2
#define LPUART_GPS_Tx_GPIO_Port GPIOA
#define LPUART_GPS_Rx_Pin GPIO_PIN_3
#define LPUART_GPS_Rx_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PB_REC_Pin GPIO_PIN_13
#define PB_REC_GPIO_Port GPIOB
#define PB_REC_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_P_Pin GPIO_PIN_6
#define STATUS_P_GPIO_Port GPIOC
#define STATUS_N_Pin GPIO_PIN_7
#define STATUS_N_GPIO_Port GPIOC
#define LPWA_NRST_Pin GPIO_PIN_9
#define LPWA_NRST_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART_DEBUG_Tx_Pin GPIO_PIN_6
#define UART_DEBUG_Tx_GPIO_Port GPIOB
#define UART_DEBUG_Rx_Pin GPIO_PIN_7
#define UART_DEBUG_Rx_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define 	ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Base @ of Page 5, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Base @ of Page 6, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Base @ of Page 7, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_128   ((uint32_t)0x08040000) /* Base @ of Page 128, 2 Kbytes */

#define 	ADDR_FLASH_PAGE_251   ((uint32_t)0x0807d800) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_252   ((uint32_t)0x0807e000) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_253   ((uint32_t)0x0807e800) /* Base @ of Page 254, 2 Kbytes */

#define 	ADDR_FLASH_PAGE_254   ((uint32_t)0x0807f000) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_255   ((uint32_t)0x0807f800) /* Base @ of Page 255, 2 Kbytes */
#define 	FLASH_ROW_SIZE          32

#define		Fs			32000
#define		fast_125ms 	4000
#define		padding		96
#define 	NLeqSlow	8
#define		false		0
#define		true		1
#define 	CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define 	GET_BITS(x, pos) ((x & ( 1 << pos)) >> pos)
#define 	CLR_BIT(p,n) ((p) &= ~((1) << (n)))

#define 	n_bytes_header			15		//
#define 	n_bytes_payload			33		// Base payload size (1st sample)
#define 	n_bytes_payload_delta	27		// Compressed delta payload size (subsequent samples)
#define		block_payloads			8		// Np -> 1 header + 1 base payload + 7 delta payloads (237 bytes total). Limited by SIM7022 since only 255 bytes can be sent in one round
#define		n_blocks_LPWA			4		// Nb -> Transmit data when XX blocks are acquired //TODO: Evaluate its modification on the configuring tool
#define		n_blocks_LPWA_limit		4		// If n_blocks_LPWA is higher than this value LPWA will power off

#define		max_blocks_LPWA			50		// Maximum number of blocks that can be transmitted in a single connection
#define		block_len				(n_bytes_header + n_bytes_payload + ((block_payloads - 1) * n_bytes_payload_delta))

#define 	main_buffer_size		20		//Main buffer number of blocks

#define 	buffer_lwpa_config_size 200
#define		eeprom_size				8191	//Size in pages number
#define 	n_blocks_full_buffer 	((uint8_t)((main_buffer_size * 5) / 10))	//Specify when the main buffer (packets) is full, then write data down to EEPROM
#define		n_blocks_EEPROM			4		//Number of blocks required to perform a EEPROM write (offline task)
#define 	temp_tests				72.8

#define		cal_def					21.8 //Kind of pre-calibrated value
#define		cal_def_LF				23 //Kind of pre-calibrated value

#define		gnss_buff_len			255
#define		usb_buff_len  			50		//Length of USB buffer
#define		usb_max_n_msg			5		//Max number of USB messages
#define 	LPWA_rx_def_tout		10		//Interval of waiting time to reach "timeout"
#define		LPWA_rx_len				100		//LPWA reception buffer length

#define		wait_ack				0		//Wait for server response (created/invalid...)


//DEBUG AND TESTING DEFINES
//#if			n_blocks_LPWA>8	// Considering ta = 23 s, tg = 1s, n_blocks_LPWA>=4, better 8 (approx 1 min)
//#define 	LOW_POWER		//Enable low power mode
//#endif

#define		LOW_FREQ		//Enable processor's clock frequency reduction when no USB is detected
//#define		NO_MIC				//Use sine signal instead of MIC

#define		debug_GNSS		0//Print GNSS information (0 - Only NGGA string, 1 - Time, Lat, Lon, Lock, and Sat)
//#define		debug_coding
#define		debug_USB
#define		debug_LPWA
#define		debug_EEPROM
#define		debug_Leq		0//Print Leq timing, TOB_LF (0), TOB (1) and LPWA (2)
#define		debug_offline


//#define		IIR_DEBUG	//Test old pars

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
