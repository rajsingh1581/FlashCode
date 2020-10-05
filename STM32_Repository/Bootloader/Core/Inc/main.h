/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"
#include <stdint.h>
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
/* Boot loader Functions Prototypes*/
void bootloader_app_init();
void user_app_init();

/* Boot loader prototypes supporting Function
 *
 */
void bootloader_RX_Buffer();
void bootloader_handle_getver_cmd(uint8_t *buff);
void bootloader_handle_gethelp_cmd(uint8_t *buff);
void bootloader_handle_getcid_cmd(uint8_t *buff);
void bootloader_handle_getrdp_cmd(uint8_t *buff);
void bootloader_handle_go_cmd(uint8_t *buff);
void bootloader_handle_flash_erase_cmd(uint8_t *buff);
void bootloader_handle_mem_write_cmd(uint8_t *buff);
void bootloader_handle_endis_rw_protect(uint8_t *buff);
void bootloader_handle_read_sector_status(uint8_t *buff);
void bootloader_handle_read_otp(uint8_t *buff);
void bootloader_handle_dis_rw_protect(uint8_t *buff);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TX2_Pin GPIO_PIN_2
#define TX2_GPIO_Port GPIOA
#define RX2_Pin GPIO_PIN_3
#define RX2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define RX1_Pin GPIO_PIN_10
#define RX1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* Bootloader Command implementation is declared here
 *
 */
//our bootloader command

//#define <command name> <command code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER 				0x51

//This command is used to read to know what are the commands supported by the bootloader
#define BL_GET_HELP 			0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID 				0x53

//This command is used to read the FLASH read protection level
#define BL_GET_RDP_STATUS 		0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR 			0x55

//This command is used to mass erase or sector erase of the user flash
#define BL_FLASH_ERASE 			0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE 			0x57

//This command is used to enable or disable read/write protect on different sectors
#define BL_ENDIS_RW_PROTECT 	0x58

//This command is used to read data from different memories of the MCU
#define BL_MEM_READ		 		0x59

//This command is used to read data from different memories of the MCU
#define BL_READ_SECTOR_STATUS 	0x5A

//This command is used to read the OTP contents
#define BL_OTP_READ 			0x5B

//This command is used to disable read write protection off mcu
#define BL_DIS_RW_PROTECT 		0x5c
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
