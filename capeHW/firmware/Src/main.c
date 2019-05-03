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
  * COPYRIGHT(c) 2019 STMicroelectronics
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USB_PD_core.h"
#include "bmp3.h"
#include "lsm303agr_reg.h"
#include "ccs811.h"
#include "ens210.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUF_DIM          1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned int Address;
unsigned int AddressSize = I2C_MEMADD_SIZE_8BIT;
USB_PD_I2C_PORT STUSB45DeviceConf[USBPORT_MAX];
uint8_t USB_PD_Interupt_Flag[USBPORT_MAX];

//static axis3bit16_t data_raw_acceleration;
//static axis3bit16_t data_raw_magnetic;
//static axis1bit16_t data_raw_temperature;
//static float acceleration_mg[3];
//static float magnetic_mG[3];
//static float temperature_degC;
//static uint8_t whoamI, rst;
//static uint8_t tx_buffer[TX_BUF_DIM];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,void *DataR ,uint16_t Length);
HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,uint8_t *DataW ,uint16_t Length);

//int8_t i2c_bmp388_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
//int8_t i2c_bmp388_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
//void bmp388_delay_ms(uint32_t ms);
//int8_t bmp388_set_normal_mode(struct bmp3_dev *dev);
//int8_t bmp388_get_sensor_data(struct bmp3_dev *dev);
static int32_t lsm303_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
static int32_t lsm303_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
void tx_com( uint8_t *tx_buffer, uint16_t len);

/* USER CODE END Prototypes */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	int PD_port = 0;

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

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2S3_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init(); // USB-PD I2C
	MX_I2C2_Init(); // Sensors I2C
	MX_SPI2_Init();


	ccs811_dev_t ccs811_dev_0 = {
		.address = (0x5A << 1),
		.read = &read_ccs811_i2c,
		.write = &write_ccs811_i2c
	};

	ens210_dev_t ens210_dev = {
		.address = (ENS210_ADDRESS << 1),
		.part_id = 3,
		.read = &read_ens210_i2c,
		.write = &write_ens210_i2c
	};


	/* Initialize ENS210 sensor */
	uint8_t err;
	err = ens210_init(&ens210_dev);
	char buf3[70];

	sprintf(buf3,  "ENS210 init return code: %i\r\nPart ID: %i\r\nAddress: %x\r\n", err, ens210_dev.part_id, ens210_dev.address);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf3, sizeof(buf3), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf3, sizeof(buf3), HAL_MAX_DELAY);

	/* Initialize CCS811 sensor */
	err = ccs811_init(&ccs811_dev_0);
	sprintf(buf3,  "CCS811 init return code: %i\r\n", err);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf3, sizeof(buf3), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf3, sizeof(buf3), HAL_MAX_DELAY);


	  
	/* USER CODE BEGIN 2 */

	STUSB45DeviceConf[PD_port].I2cBus = PD_port;
	STUSB45DeviceConf[PD_port].I2cDeviceID_7bit = 0x28;
	AddressSize = I2C_MEMADD_SIZE_8BIT;

	uint8_t Cut[USBPORT_MAX];

	int Status = I2C_Read_USB_PD(STUSB45DeviceConf[PD_port].I2cBus,STUSB45DeviceConf[PD_port].I2cDeviceID_7bit,DEVICE_ID ,&Cut[PD_port], 1 );
	if (Cut[PD_port] == 0x21 )
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);

	HW_Reset_state(PD_port);
	Print_PDO_FROM_SRC(PD_port);
	Read_SNK_PDO(PD_port);
	Print_SNK_PDO(PD_port);
	//    nvm_flash(STUSB45DeviceConf[PD_port].I2cBus);




	/* USER CODE END 2 */


	/* CCS811 Firmware app read test */
	ccs811_alg_results_t ccs811_results;
	char buf2[70];
	sprintf(buf2, "Firmware app version before get: %i\r\n", ccs811_dev_0.fw_app_version);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);
	HAL_Delay(1000);
	ccs811_dev_0.fw_app_version = get_firmware_app_version(&ccs811_dev_0);
	sprintf(buf2, "Firmware app version after get: %i\r\n", ccs811_dev_0.fw_app_version);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);

	/* Report ENS210 operating status */
	char buf4[110];
	ens210_status_t ens210_status;
	err = ens210_get_status(&ens210_dev, &ens210_status); 
	sprintf(buf4,"ENS210 get status error = %i\r\nlow power = %i\r\npower state = %i\r\nsensor run modes = %i\r\nstart = %i\r\n",
			err, ens210_status.low_power, ens210_status.power_state, ens210_status.sensor_run_modes, ens210_status.start);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf4, sizeof(buf4), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)buf4, sizeof(buf4), HAL_MAX_DELAY);


	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	char buf5[64];
	uint16_t co2_ppm;
	uint16_t tvoc_ppb;
	ens210_data_t ens210_data = {
		.T_VAL = 5,
		.H_VAL = 6
	};

	while (1)
	{
		/* Get CCS811 eCO2 reading */
		ccs811_results = ccs811_get_data(&ccs811_dev_0);
		co2_ppm = ccs811_results.eCO2;
		tvoc_ppb = ccs811_results.TVOC;

		/* Get ENS210 temp and humidity */
		err = ens210_get_data(&ens210_dev, &ens210_data);
		
		memset(buf5, 0, sizeof(buf5));
		sprintf(buf5, "sensors,%i,%i,%0.2f,%0.2f\r\n", co2_ppm, tvoc_ppb, ens210_data.T_VAL, ens210_data.H_VAL);
		HAL_UART_Transmit(&huart1, (uint8_t *)buf5, sizeof(buf5), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)buf5, sizeof(buf5), HAL_MAX_DELAY);


		/* USER CODE END WHILE */
		//	  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		//	  HAL_Delay(200);
		//HAL_UART_Transmit(&huart1, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);
		//bmp388_get_sensor_data(&bmp388);
		/*
		* Read output only if new value is available
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
		*/

		/* USER CODE BEGIN 3 */


	}
/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);
  /**Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_LED_Pin|CO2_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CO2_WAKE_Pin */
  GPIO_InitStruct.Pin = CO2_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CO2_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI1_ALS_INT_Pin */
  GPIO_InitStruct.Pin = EXTI1_ALS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI1_ALS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI4_PRESS_INT_Pin EXTI6_MAG_DRDY_Pin */
  GPIO_InitStruct.Pin = EXTI4_PRESS_INT_Pin|EXTI6_MAG_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI0_CO2_INT_Pin EXTI5_PD_I2C_ALERT_Pin */
  GPIO_InitStruct.Pin = EXTI0_CO2_INT_Pin|EXTI5_PD_I2C_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI14_XL_1_Pin EXTI15_XL_2_Pin */
  GPIO_InitStruct.Pin = EXTI14_XL_1_Pin|EXTI15_XL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8_GPIO_Pin */
  GPIO_InitStruct.Pin = PC8_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PC8_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RK_GPIO_Pin */
  GPIO_InitStruct.Pin = RK_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RK_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD_ATTACH_Pin PD_GPIO_Pin */
  GPIO_InitStruct.Pin = PD_ATTACH_Pin|PD_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,void *DataR ,uint16_t Length)
{
	return  HAL_I2C_Mem_Read(&hi2c1,(I2cDeviceID_7bit<<1), Address ,AddressSize, DataR, Length ,2000);
}
HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,uint8_t *DataW ,uint16_t Length)
{
	return  HAL_I2C_Mem_Write(&hi2c1,(I2cDeviceID_7bit<<1), Address ,AddressSize, DataW, Length,2000 ); // unmask all alarm status
}

int8_t i2c_bmp388_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	return HAL_I2C_Mem_Read(&hi2c2, dev_id, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, 0xFFFF);
}

int8_t i2c_bmp388_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	return HAL_I2C_Mem_Write(&hi2c2, dev_id, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, 0xFFFF);
}

void bmp388_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

int8_t bmp388_set_normal_mode(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev->settings.press_en = BMP3_ENABLE;
    dev->settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    dev->settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    dev->settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    dev->settings.odr_filter.odr = BMP3_ODR_200_HZ;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, dev);

    /* Set the power mode to normal mode */
    dev->settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(dev);

    return rslt;
}

int8_t bmp388_get_sensor_data(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, dev);

    /* Print the temperature and pressure data */
    char buf1[40] = "Temperature\t Pressure\t\n";
    //printf("Temperature\t Pressure\t\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf1, sizeof(buf1), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf1, sizeof(buf1), HAL_MAX_DELAY);
    char buf2[40];
    sprintf(buf2, "%0.2f\t\t %0.2f\t\t\r\n", data.temperature, data.pressure);
    //printf("%0.2f\t\t %0.2f\t\t\n",data.temperature, data.pressure);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf2, sizeof(buf2), HAL_MAX_DELAY);

    return rslt;
}

static int32_t lsm303_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
  }
  HAL_I2C_Mem_Write(&hi2c2, i2c_add, Reg,
                    I2C_MEMADD_SIZE_8BIT, Bufp, len, 0xFFFF);
  return 0;
}

static int32_t lsm303_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
  }
  HAL_I2C_Mem_Read(&hi2c2, (uint8_t) i2c_add, Reg,
                   I2C_MEMADD_SIZE_8BIT, Bufp, len, 0xFFFF);
  return 0;
}

/*
 *  Function to print messages
 */
void tx_com( uint8_t *tx_buffer, uint16_t len )
{
  HAL_UART_Transmit(&huart1, tx_buffer, len, 1000 );
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000 );
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
