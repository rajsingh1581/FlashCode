/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include<stdarg.h>
#include<string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000
#define D_UART &huart2
#define B_UART &huart1
#define BL_DEBUG_MSG_EN
#define BL_RX_LEN 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t bl_rx_buff[BL_RX_LEN];
uint8_t supported_commands[11] = {BL_GET_VER,
									BL_GET_HELP,
									BL_GET_CID,
									BL_GET_RDP_STATUS,
									BL_GO_TO_ADDR,
									BL_FLASH_ERASE,
									BL_MEM_WRITE,
									BL_ENDIS_RW_PROTECT,
									BL_MEM_READ,
									BL_READ_SECTOR_STATUS,
									BL_OTP_READ};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
void debug(char *str);
void bootloader_uart_write_data(uint8_t *str, uint32_t len);
static void debugprint(char *format, ...);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug(char *str)
{
	HAL_UART_Transmit(D_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}


void bootloader_uart_write_data(uint8_t *str, uint32_t len)
{
	HAL_UART_Transmit(B_UART, str, len, HAL_MAX_DELAY);
}

void debugprint(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[1000];
	/*Extract the arguments list using Va apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
#endif
}

void bootloader_app_init()
{
	debugprint("\r\ndebug==>Bootloader Jump to Bootloader Code:");
	bootloader_RX_Buffer();
}


void user_app_init()
{
	//just a function to hold the address of the reset handler of the user app.
	void (*app_reset_handler)(void);

	debugprint("\r\ndebug==>Bootloader Jump to User Code:");

	//1. Configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	debugprint("\r\ndebug==>MSP Value : %#x",msp_value);

	//This function come from CMSIS
	__set_MSP(msp_value);

	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

	/*2. Now Fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR_BASE_ADDRESS+4
	 */
	uint32_t resethandler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
	app_reset_handler = (void*) resethandler_address;

	debugprint("\r\ndebug==> App reset handler addr : %#x",app_reset_handler);

	//3. jump to reset handler of the application
	app_reset_handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();// We are using GPIO peripherals
  MX_USART2_UART_Init();//UART2 peripheral for Debugging
  MX_USART1_UART_Init();//UART1 peripheral for boot loader Command
  MX_CRC_Init();//CRC we are using to check data correction
  /* USER CODE BEGIN 2 */

  debugprint("\r\nBootloader Debug: HELLO! From Bootloader!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  //bootloader("Hi, This is bootloader UART Testing\r\n");
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	  {
		  debugprint("\r\nDebug==> Bootloader code is Running");
		  bootloader_app_init();
	  }

	  else
	  {
		  debugprint("\r\nDebug==> USER code is Running");
		  user_app_init();
	  }
	  //HAL_Delay(5000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



void bootloader_RX_Buffer()
{
	uint8_t rcv_len = 0;

	while(1)
	{
		memset(bl_rx_buff, 0, BL_RX_LEN);
		//here we will read and decode the coomands coming from host
		//first read only one byte from the host, which is the "length" field of the command packet
		HAL_UART_Receive(B_UART, bl_rx_buff, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buff[0];
		HAL_UART_Receive(B_UART, &bl_rx_buff[1], rcv_len, HAL_MAX_DELAY);

		switch(bl_rx_buff[1])
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buff);
				break;

			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buff);
				break;

			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buff);
				break;

			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(bl_rx_buff);
				break;

			case BL_GO_TO_ADDR:
				bootloader_handle_go_cmd(bl_rx_buff);
				break;

			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase_cmd(bl_rx_buff);
				break;

			case BL_MEM_WRITE:
				bootloader_handle_mem_write_cmd(bl_rx_buff);
				break;

			case BL_ENDIS_RW_PROTECT:
				bootloader_handle_endis_rw_protect(bl_rx_buff);
				break;

			case BL_READ_SECTOR_STATUS:
				bootloader_handle_read_sector_status(bl_rx_buff);
				break;

			case BL_OTP_READ:
				bootloader_handle_read_otp(bl_rx_buff);
				break;

			/*case BL_DIS_RW_PROTECT:
				bootloader_handle_dis_rw_protect(bl_rx_buff);
				break;*/

			default:
				debugprint("\r\nDebug==> Invalid command Received from serial host.....");
				break;
		}
	}

}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************************** IMPLMENTATION OF BOOTLOADER COMMAND HANDLE FUNCTIONS*************************************/
void bootloader_handle_getver_cmd(uint8_t *buff)
{
	uint8_t bl_version;

	//1) verify the checksum
	debugprint("\r\ndebug==> Bootloader handle getver cmd");

	//Total length of the command packet
	uint32_t command_packet_len = buff[0]+1;

	//extract the CRC32 sent by the HOST
	uint32_t host_crc = *((uint32_t *)(buff + command_packet_len - 4) );

	if(! bootloader_verify_crc(&buff[0], command_packet_len - 4, host_crc))
	{
		debugprint("\r\ndebug==> checksum success !!");
		//checksum is correct
		bootloader_send_ack(buff[0],1);
		bl_version = get_bootloader_version();
		debugprint("\r\ndebug==> BL_VER :: %d %#x",bl_version,bl_version);
		bootloader_uart_write_data(&bl_version,1);
	}

	else
	{
		debugprint("\r\ndebug==> CheckSum Fail !!");
		//checksum is wrong send nack
		bootloader_send_nack();
	}
}

/******************************************************************************************************
 * Helper Function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command Codes
 * *****************************************************************************************************/
void bootloader_handle_gethelp_cmd(uint8_t *buff)
{
	debugprint("\r\ndebug==> Bootloader Handle Get Help");

	//Total Length of the command packet
	uint32_t command_packet_len = buff[0]+1;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*)(buff+command_packet_len - 4));

	if(! bootloader_verify_crc(&buff[0], command_packet_len-4, host_crc))
	{
		debugprint("\r\ndebug==> CheckSum Success !!\n");
		bootloader_send_ack(buff[0], sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
	}

	else
	{
		debugprint("\r\ndebug==> CheckSum Fail !!");
		bootloader_send_nack();
	}
}

/******************************************************************************************************
 * Bootloader Get Chip ID Command is Implemented
 * Chip ID is available on get_mcu_chip_id() function
 * *****************************************************************************************************/

void bootloader_handle_getcid_cmd(uint8_t *buff)
{
	uint16_t bl_cid_num = 0;
	debugprint("\r\ndebug==> Bootloader Handle Get CID");

	//Total Length of the command packet
	uint32_t command_packet_len = buff[0]+1;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*)(buff+command_packet_len - 4));

	if(! bootloader_verify_crc(&buff[0], command_packet_len-4, host_crc))
	{
		debugprint("\r\ndebug==> CheckSum Success !!\n");
		bootloader_send_ack(buff[0], sizeof(supported_commands));
		bl_cid_num = get_mcu_chip_id();
		debugprint("\r\ndebug==> MCU ID :: %d %#x !!",bl_cid_num, bl_cid_num);
		bootloader_uart_write_data((uint8_t*)&bl_cid_num,2);

	}

	else
	{
		debugprint("\r\ndebug==> CheckSum Fail !!");
		bootloader_send_nack();
	}
}

/******************************************************************************************************
 * Bootloader Get Read Write Protection ON the chip available on a Option Bytes Memory Section
 * *****************************************************************************************************/

void bootloader_handle_getrdp_cmd(uint8_t *buff)
{
	uint8_t rdp_level = 0x00;
	debugprint("\r\ndebug==> Bootloader Handle Get Read Write Protection Level");

	//Total Length of the command packet
	uint32_t command_packet_len = buff[0]+1;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t*)(buff+command_packet_len - 4));

	if(! bootloader_verify_crc(&buff[0], command_packet_len-4, host_crc))
	{
		debugprint("\r\ndebug==> CheckSum Success !!\n");
		bootloader_send_ack(buff[0], 1);
		rdp_level = get_flash_rdp_level();
		debugprint("\r\ndebug==>RDP Level: %d %#x",rdp_level, rdp_level);
		bootloader_uart_write_data(&rdp_level, 1);
	}

	else
	{
		debugprint("\r\ndebug==> CheckSum Fail !!");
		bootloader_send_nack();
	}
}

void bootloader_handle_go_cmd(uint8_t *buff)
{
	uint32_t go_address =0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;

	debugprint("\r\ndebug==> Bootloader Handle Go Command");

		//Total Length of the command packet
		uint32_t command_packet_len = buff[0]+1;

		//extract the CRC32 sent by the Host
		uint32_t host_crc = *((uint32_t*)(buff+command_packet_len - 4));

		if(! bootloader_verify_crc(&buff[0], command_packet_len-4, host_crc))
		{
			debugprint("\r\ndebug==> CheckSum Success !!\n");
			bootloader_send_ack(buff[0], 1);

			//extract the go address
			go_address = *((uint32_t*)&buff[2]);
			debugprint("\r\ndebug==> Go Addr :: %#x", go_address);

			if( verify_address(go_address) == ADDR_VALID)
			{
				//tell host that address is fine
				bootloader_uart_write_data(&addr_valid, 1);

				/*jump to "go" address.
				 * we don't care what is being done there.
				 * host must ensure that valid code is present over thaere
				 * Its not the duty of bootloader. So just trust and jump
				 */
				 /*Not doing the below line will result in hardfault eception From ARM Cortex M */
				//watch : https://www.youtube.com/watch?v=VX_12SjnNhY

				go_address += 1; //make T bit = 1

				void (*lets_jump)(void) = (void *)go_address;
				debugprint("\r\ndebug==> Jumping to go address !");
				lets_jump();

			}

			else
			{
				debugprint("\r\ndebug==> Go Address INVALID!!");
				//Tell Host address is invalid
				bootloader_uart_write_data(&addr_invalid, 1);
			}
		}

		else
		{
			debugprint("\r\ndebug==> CheckSum Fail !!");
			bootloader_send_nack();
		}

}

void bootloader_handle_flash_erase_cmd(uint8_t *buff)
{

}

void bootloader_handle_mem_write_cmd(uint8_t *buff)
{

}

void bootloader_handle_endis_rw_protect(uint8_t *buff)
{

}

void bootloader_handle_read_sector_status(uint8_t *buff)
{

}

void bootloader_handle_read_otp(uint8_t *buff)
{

}

void bootloader_handle_dis_rw_protect(uint8_t *buff)
{

}


/*******************Functions to implement Send ACK and NCK*******************************/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	//Here we send 2 bytes.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(B_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(B_UART, &nack, 1, HAL_MAX_DELAY);
}


/***********************This Function Verifies the CRC of the given buffer in pData.******/
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;

	for(uint32_t i=0; i<len; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}

//Just returns the macro value
uint8_t get_bootloader_version(void)
{
	return (uint8_t)BL_VERSION;
}

//Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id(void)
{
/**************************************************************************************
 * The STM32F410XX MCUs integrate MCU ID code. This ID identifies the ST MCU Part no
 * and the die revision. It is part of the DBG_MCU component and is mapped on
 * external PPB bus (see Section 33.16 on page 1304). This code is accessible
 * JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the
 * It is even accessible while the MCU is under system reset.
 ***************************************************************************************/
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}

uint8_t get_flash_rdp_level(void)
{
	uint8_t rdp_status = 0;
#if 0
	FLASH_OBProgramInitTypeDef ob_handle;//OB Means Option Bytes
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else
	volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	rdp_status = (uint8_t)(*pOB_addr >> 8);
#endif
	return rdp_status;
}

	//So, what are the valid addresses to which we can jump?
	//Can we jump to system memory? yes
	//Can we jump to sram1 memory? yes
	//Can we jump to sram2 memory? yes
	//Can we jump to backup sram memory? yes
	//Can we jump to peripheral memory? its possible, but don't allow, so no
	//Can we jump to external memory? yes

uint8_t verify_address(uint32_t go_address)
{
//incomplete -poorly written .. optimize it
	if(go_address >= SRAM_BASE_ADDR && go_address <= SRAM_END_ADDR)
	{
		return ADDR_VALID;
	}

	/*else if( go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}*/

	else if( go_address >= FLASH_BASE_ADDR && go_address <= FLASH_END_ADDR)
	{
		return ADDR_VALID;
	}

	/*else if( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}*/

	else
		return ADDR_INVALID;

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
