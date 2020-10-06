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
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);
uint8_t get_bootloader_version(void);
uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
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
//version 1.0
#define BL_VERSION 0x10
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

/*Check for Address Valid*/
//SRAM Address Starting to End ADDRESS
#define SRAM_SIZE 32*1024
#define SRAM_BASE_ADDR 0x20000000
#define SRAM_END_ADDR  (SRAM1_BASE + SRAM_SIZE)

//FLASH Address Starting to End ADDRESS
//FLASH Starting address to End Address
#define FLASH_SIZE 128*1024
#define FLASH_BASE_ADDR 0x08000000
#define FLASH_END_ADDR  (FLASH_BASE_ADDR + FLASH_SIZE)

//Check Valid AND Invalid Address locations
#define ADDR_VALID   0x00
#define ADDR_INVALID 0x01

/* ACK and NACK Bytes*/
#define BL_ACK 0XA5
#define BL_NACK 0x7E
/* Check for CRC Verification Value */
#define VERIFY_CRC_SUCCESS 	0
#define VERIFY_CRC_FAIL 	1

/* USER CODE END Private defines */



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
