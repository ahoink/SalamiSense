/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
UART_HandleTypeDef huart1;
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
#define RED_LED_Pin GPIO_PIN_13
#define RED_LED_GPIO_Port GPIOC
#define RCC_OSC32K_IN_Pin GPIO_PIN_14
#define RCC_OSC32K_IN_GPIO_Port GPIOC
#define RCC_OSC32K_OUT_Pin GPIO_PIN_15
#define RCC_OSC32K_OUT_GPIO_Port GPIOC
#define RCC_OSC26M_IN_Pin GPIO_PIN_0
#define RCC_OSC26M_IN_GPIO_Port GPIOH
#define RCC_OSC26M_OUT_Pin GPIO_PIN_1
#define RCC_OSC26M_OUT_GPIO_Port GPIOH
#define CO2_WAKE_Pin GPIO_PIN_0
#define CO2_WAKE_GPIO_Port GPIOC
#define EXTI1_ALS_INT_Pin GPIO_PIN_1
#define EXTI1_ALS_INT_GPIO_Port GPIOC
#define UART2_RK_CTS_Pin GPIO_PIN_0
#define UART2_RK_CTS_GPIO_Port GPIOA
#define UART2_RK_RTS_Pin GPIO_PIN_1
#define UART2_RK_RTS_GPIO_Port GPIOA
#define UART2_RK_TX_Pin GPIO_PIN_2
#define UART2_RK_TX_GPIO_Port GPIOA
#define UART2_RK_RX_Pin GPIO_PIN_3
#define UART2_RK_RX_GPIO_Port GPIOA
#define ISENSE_12V_Pin GPIO_PIN_5
#define ISENSE_12V_GPIO_Port GPIOA
#define SPI1_RK_MISO_Pin GPIO_PIN_6
#define SPI1_RK_MISO_GPIO_Port GPIOA
#define SPI1_RK_MOSI_Pin GPIO_PIN_7
#define SPI1_RK_MOSI_GPIO_Port GPIOA
#define EXTI4_PRESS_INT_Pin GPIO_PIN_4
#define EXTI4_PRESS_INT_GPIO_Port GPIOC
#define EXTI0_CO2_INT_Pin GPIO_PIN_0
#define EXTI0_CO2_INT_GPIO_Port GPIOB
#define ISENSE_3V3_Pin GPIO_PIN_1
#define ISENSE_3V3_GPIO_Port GPIOB
#define I2C2_SENSOR_SCL_Pin GPIO_PIN_10
#define I2C2_SENSOR_SCL_GPIO_Port GPIOB
#define EXTI14_XL_1_Pin GPIO_PIN_14
#define EXTI14_XL_1_GPIO_Port GPIOB
#define EXTI15_XL_2_Pin GPIO_PIN_15
#define EXTI15_XL_2_GPIO_Port GPIOB
#define EXTI6_MAG_DRDY_Pin GPIO_PIN_6
#define EXTI6_MAG_DRDY_GPIO_Port GPIOC
#define PC8_GPIO_Pin GPIO_PIN_8
#define PC8_GPIO_GPIO_Port GPIOC
#define RK_GPIO_Pin GPIO_PIN_8
#define RK_GPIO_GPIO_Port GPIOA
#define UART1_FTDI_TX_Pin GPIO_PIN_9
#define UART1_FTDI_TX_GPIO_Port GPIOA
#define UART1_FTDI_RX_Pin GPIO_PIN_10
#define UART1_FTDI_RX_GPIO_Port GPIOA
#define UART1_FTDI_CTS_Pin GPIO_PIN_11
#define UART1_FTDI_CTS_GPIO_Port GPIOA
#define UART1_FTDI_RTS_Pin GPIO_PIN_12
#define UART1_FTDI_RTS_GPIO_Port GPIOA
#define SPI1_RK_NSS_Pin GPIO_PIN_15
#define SPI1_RK_NSS_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_2
#define GREEN_LED_GPIO_Port GPIOD
#define SPI1_RK_SCK_Pin GPIO_PIN_3
#define SPI1_RK_SCK_GPIO_Port GPIOB
#define PD_ATTACH_Pin GPIO_PIN_4
#define PD_ATTACH_GPIO_Port GPIOB
#define EXTI5_PD_I2C_ALERT_Pin GPIO_PIN_5
#define EXTI5_PD_I2C_ALERT_GPIO_Port GPIOB
#define I2C1_PD_SCL_Pin GPIO_PIN_6
#define I2C1_PD_SCL_GPIO_Port GPIOB
#define I2C1_PD_SDA_Pin GPIO_PIN_7
#define I2C1_PD_SDA_GPIO_Port GPIOB
#define PD_GPIO_Pin GPIO_PIN_8
#define PD_GPIO_GPIO_Port GPIOB
#define I2C2_SENSOR_SDA_Pin GPIO_PIN_9
#define I2C2_SENSOR_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
