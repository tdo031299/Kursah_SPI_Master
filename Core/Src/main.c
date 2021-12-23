/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROTOCOL_CMD_CS0	"CS0\n"
#define PROTOCOL_CMD_CS1	"CS1\n"
#define PROTOCOL_CMD_M0	"M0\n"
#define PROTOCOL_CMD_M1	"M1\n"
#define PROTOCOL_CMD_M2	"M2\n"
#define PROTOCOL_CMD_M3	"M3\n"
#define PROTOCOL_CMD_CD	"CD "

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int rx(unsigned char *r) {
	int result = 0;
	if (HAL_UART_Receive(&huart1, r, 1, 500) == HAL_OK) {
		result = 1;
	}
	return result;
}

enum state_list {
	ST_RESET = 0,
	ST_D1,
	ST_D2,
	STN,
	ST_C,
	ST_M,
	ST_CS,
	ST_CSn0,
	ST_CSn1,
	ST_M0,
	ST_M1,
	ST_M2,
	ST_M3,
	ST_Dn,
};
enum state_list main_state;

static char halfhex2char(char c) {
	char s;
	if (c >= '0' && c <= '9')
		s = c - '0';
	else {
		if (c >= 'A' && c <= 'F')
			s = c - 'A' + 10;
	}
	return s;
}

static char hex2char(char c1, char c2) {
	return (halfhex2char(c1) << 4) + halfhex2char(c2);
}

static char bytetohex(char c) {
	char s;
	if (c >= 0x00 && c <= 0x09)
		s = c + 0x30;
	else {
		if (c >= 0x0A && c <= 0x0F)
			s = c + 0x37;
	}
	return s;
}

static void chtohex(char c, unsigned char *str) {
	str[0] = bytetohex(c >> 4);
	str[1] = bytetohex(c * 0x0F);
}

static void SPI1_Init_Clk(uint32_t CLKPolarity, uint32_t CLKPhase) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = CLKPolarity;
	hspi1.Init.CLKPhase = CLKPhase;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	main_state = ST_RESET;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	unsigned char ress;
	char sy1;
	char sy2;
	unsigned char c;
	unsigned char symbols[2];
	unsigned char buf[64];
	unsigned char rxbuf[64];
	int k = 0;

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (rx(&c)) {
			switch (main_state) {
			case ST_RESET:
				switch (c) {
				case 'D':
					main_state = ST_D1;
					break;
				case 'C':
					main_state = ST_C;
					break;
				case 'M':
					main_state = ST_M;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_C:
				switch (c) {
				case 'S':
					main_state = ST_CS;
					break;
				default:
					main_state = ST_RESET;
				}

			case ST_CS:
				switch (c) {
				case '0':
					main_state = ST_CSn0;
					break;
				case '1':
					main_state = ST_CSn1;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_CSn0:
				switch (c) {
				case '\n':
					HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,
							GPIO_PIN_RESET);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_CSn1:
				switch (c) {
				case '\n':
					HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,
							GPIO_PIN_SET);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_M:
				switch (c) {
				case '0':
					main_state = ST_M0;
					break;
				case '1':
					main_state = ST_M1;
					break;
				case '2':
					main_state = ST_M2;
					break;
				case '3':
					main_state = ST_M3;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_M0:
				switch (c) {
				case '\n':
					SPI1_Init_Clk(SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_M1:
				switch (c) {
				case '\n':
					SPI1_Init_Clk(SPI_POLARITY_LOW, SPI_PHASE_2EDGE);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_M2:
				switch (c) {
				case '\n':
					SPI1_Init_Clk(SPI_POLARITY_HIGH, SPI_PHASE_1EDGE);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_M3:
				switch (c) {
				case '\n':
					SPI1_Init_Clk(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
					main_state = ST_RESET;
					break;
				default:
					main_state = ST_RESET;
				}
				break;

			case ST_D1:
				switch (c) {
				case '\n':
					HAL_SPI_TransmitReceive(&hspi1, &buf[0], &rxbuf[0], k, 500);
					for (int i = 0; i < k; i++) {
						chtohex(rxbuf[i], symbols);
						HAL_UART_Transmit(&huart1, &symbols[0], 2, 500);
					}
					main_state = ST_RESET;
					break;
				default:
					sy1 = c;
					main_state = ST_D2;
					break;
				}
				break;

			case ST_D2:
				sy2 = c;
				ress = hex2char(sy1, sy2);
				buf[k] = ress;
				k++;
				main_state = ST_D1;
				break;
			default:
				main_state = ST_RESET;

			}
		}
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SPI1_NSS_Pin */
	GPIO_InitStruct.Pin = SPI1_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
