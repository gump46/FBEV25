/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "spi.h"
#include "gpio.h"
#include "CANSPI.h"
#include "ltc2497.h"  // Adjusted include to match the filename
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LTC2497_DEVICE_ADDRESS    LTC2497_I2C_ADDRESS  // Use the address defined in ltc2497.h
#define NUM_POINTS 33
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IPCC_HandleTypeDef hipcc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
int8_t thermistor_values[NUM_THERMISTORS];  // Store temperature values in °C

// Define the differential channels you want to read
uint8_t differential_channels[NUM_THERMISTORS] = {
    LTC2497_CH_P0_N1,  // First differential pair (Adjusted constants)
    LTC2497_CH_P2_N3,   // Second differential pair
	LTC2497_CH_P4_N5,
	LTC2497_CH_P6_N7,
	LTC2497_CH_P8_N9,
	LTC2497_CH_P10_N11,
	LTC2497_CH_P12_N13,
	LTC2497_CH_P14_N15
};

uint32_t last_address_claim_time = 0;
uint32_t last_module_broadcast_time = 0;
uint32_t last_general_broadcast_time = 0;

uCAN_MSG txMessage;
uCAN_MSG rxMessage;

uint8_t adc_command;
int32_t adc_code;
float adc_voltage;
float temperature_celsius;
float vref = 5.0f; // Reference voltage for the LTC2497, adjust if different

float voltage_table[NUM_POINTS] = {
    2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99,
    1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43,
    1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30
};

float temperature_table[NUM_POINTS] = {
    -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
    20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75,
    80, 85, 90, 95, 100, 105, 110, 115, 120
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_IPCC_Init(void);
/* USER CODE BEGIN PFP */
void Send_Address_Claim_Broadcast(void);
void Send_Thermistor_Module_Broadcast(void);
void Send_Thermistor_General_Broadcast(void);

uint8_t I2C_DeviceReady(I2C_HandleTypeDef *hi2c, uint16_t device_address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Function to convert differential voltage to temperature */
float VoltageToTemperature(float voltage)
{
    // Handle out-of-range voltages
    if (voltage >= voltage_table[0])
    {
        return temperature_table[0]; // Voltage is higher than highest in table
    }
    if (voltage <= voltage_table[NUM_POINTS - 1])
    {
        return temperature_table[NUM_POINTS - 1]; // Voltage is lower than lowest in table
    }

    // Search for the interval containing the voltage
    for (int i = 0; i < NUM_POINTS - 1; i++)
    {
        if (voltage <= voltage_table[i] && voltage >= voltage_table[i+1])
        {
            // Perform linear interpolation
            float v1 = voltage_table[i];
            float v2 = voltage_table[i+1];
            float t1 = temperature_table[i];
            float t2 = temperature_table[i+1];

            // Interpolate
            float temperature = t1 + (voltage - v1) * (t2 - t1) / (v2 - v1);

            return temperature;
        }
    }

    // If voltage not found in table, return an error value
    return -999; // Indicate error
}


uint8_t I2C_DeviceReady(I2C_HandleTypeDef *hi2c, uint16_t device_address)
{
    if (HAL_I2C_IsDeviceReady(hi2c, device_address << 1, 3, 1000) == HAL_OK)
    {
        // Device is ready
        return 1;
    }
    else
    {
        // Device is not ready
        return 0;
    }
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CANSPI_Initialize();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        for (uint8_t i = 0; i < NUM_THERMISTORS; i++)
        {
            // Build command for the current differential channel
            adc_command = differential_channels[i];  // Adjusted command building

            // Read temperature data from LTC2497
            if (LTC2497_Read(LTC2497_DEVICE_ADDRESS, adc_command, &adc_code, 1000) == 0)  // Removed &hi2c1
            {
                // Convert ADC code to voltage
                adc_voltage = LTC2497_CodeToVoltage(adc_code, vref);

                // Convert voltage to temperature in degrees Celsius
                temperature_celsius = VoltageToTemperature(adc_voltage);

                // Store temperature as int8_t in degrees Celsius
                thermistor_values[i] = (int8_t)(temperature_celsius);
            }
            else
            {
                // Handle LTC2497 read error
            }

        }

              // Send Address Claim Broadcast every 200 ms
              if (HAL_GetTick() - last_address_claim_time >= 200)
              {
                  Send_Address_Claim_Broadcast();
                  last_address_claim_time = HAL_GetTick();
              }

              // Send Thermistor Module Broadcast every 100 ms
              if (HAL_GetTick() - last_module_broadcast_time >= 100)
              {
                  Send_Thermistor_Module_Broadcast();
                  last_module_broadcast_time = HAL_GetTick();
              }

              // Send Thermistor General Broadcast every 100 ms
              if (HAL_GetTick() - last_general_broadcast_time >= 100)
              {
                  Send_Thermistor_General_Broadcast();
                  last_general_broadcast_time = HAL_GetTick();
              }

              // Optional: Handle incoming CAN messages
              if (CANSPI_Receive(&rxMessage))
              {
                  // Process received messages if necessary
              }

  /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pull_up_Test_GPIO_Port, Pull_up_Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pull_up_Test_Pin */
  GPIO_InitStruct.Pin = Pull_up_Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Pull_up_Test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Send_Address_Claim_Broadcast(void)
{
    uCAN_MSG txMessage;
    txMessage.frame.idType = CAN_FRAME_EXT;
    txMessage.frame.id = 0x18EEFF80 | MODULE_NUMBER;  // The last byte is the source address (MODULE_NUMBER)
    txMessage.frame.dlc = 8;

    txMessage.frame.data0 = 0xF3;                                  // Unique identifier
    txMessage.frame.data1 = 0x00;
    txMessage.frame.data2 = 0x80;
    txMessage.frame.data3 = BMS_TARGET_ADDRESS;                    // BMS Target Address (default 0x80)
    txMessage.frame.data4 = MODULE_NUMBER << 3;                    // Thermistor Module Number shifted left by 3
    txMessage.frame.data5 = 0x40;                                  // Constant
    txMessage.frame.data6 = 0x1E;                                  // Constant
    txMessage.frame.data7 = 0x90;                                  // Constant
                                // Unused or default value

    // Transmit the CAN message
    if (CANSPI_Transmit(&txMessage) != 0)
    {
        // Handle transmission error if needed
    }
}

void Send_Thermistor_Module_Broadcast(void)
{
    uCAN_MSG txMessage;
    txMessage.frame.idType = CAN_FRAME_EXT;
    txMessage.frame.id = 0x1839F380 | MODULE_NUMBER;  // Source address is module number
    txMessage.frame.dlc = 8;

    int8_t lowest_temp = thermistor_values[0];
    int8_t highest_temp = thermistor_values[0];
    int32_t sum_temp = thermistor_values[0];
    uint8_t lowest_temp_id = 0;
    uint8_t highest_temp_id = 0;

    for (uint8_t i = 1; i < NUM_THERMISTORS; i++)
    {
        if (thermistor_values[i] < lowest_temp)
        {
            lowest_temp = thermistor_values[i];
            lowest_temp_id = i;
        }
        if (thermistor_values[i] > highest_temp)
        {
            highest_temp = thermistor_values[i];
            highest_temp_id = i;
        }
        sum_temp += thermistor_values[i];
    }

    txMessage.frame.data0 = MODULE_NUMBER;
    txMessage.frame.data1 = lowest_temp;                              // Lowest temperature
    txMessage.frame.data2 = highest_temp;                             // Highest temperature
    txMessage.frame.data3 = (int8_t)(sum_temp / NUM_THERMISTORS);     // Average temperature
    txMessage.frame.data4 = NUM_THERMISTORS;                          // Number of thermistors enabled
    txMessage.frame.data5 = highest_temp_id;                          // Highest thermistor ID
    txMessage.frame.data6 = lowest_temp_id;                           // Lowest thermistor ID

    // Calculate checksum
    uint16_t sum = 0x39 + txMessage.frame.dlc;  // Start with 0x39 and length
    sum += txMessage.frame.data0;
    sum += txMessage.frame.data1;
    sum += txMessage.frame.data2;
    sum += txMessage.frame.data3;
    sum += txMessage.frame.data4;
    sum += txMessage.frame.data5;
    sum += txMessage.frame.data6;
    txMessage.frame.data7 = (uint8_t)(sum & 0xFF);  // Checksum is lower 8 bits

    // Transmit the CAN message
    if (CANSPI_Transmit(&txMessage) != 0)
    {
        // Handle transmission error if needed
    }
}


/* Implement Send_Thermistor_General_Broadcast as before */
void Send_Thermistor_General_Broadcast(void)
{
    static uint8_t thermistor_index = 0;  // Keep track of which thermistor to send

    uCAN_MSG txMessage;
    txMessage.frame.idType = CAN_FRAME_EXT;
    txMessage.frame.id = 0x1838F380 | MODULE_NUMBER;  // Source address is module number
    txMessage.frame.dlc = 8;

    // Calculate Thermistor ID relative to all configured modules
    uint8_t thermistor_id_all = MODULE_NUMBER * NUM_THERMISTORS + thermistor_index;
    int8_t lowest_temp = thermistor_values[0];
    int8_t highest_temp = thermistor_values[0];
    int32_t sum_temp = thermistor_values[0];
    uint8_t lowest_temp_id = 0;
    uint8_t highest_temp_id = 0;

    for (uint8_t i = 1; i < NUM_THERMISTORS; i++)
    {
        if (thermistor_values[i] < lowest_temp)
        {
            lowest_temp = thermistor_values[i];
            lowest_temp_id = i;
        }
        if (thermistor_values[i] > highest_temp)
        {
            highest_temp = thermistor_values[i];
            highest_temp_id = i;
        }
        sum_temp += thermistor_values[i];
    }

    txMessage.frame.data0 = thermistor_id_all;
    txMessage.frame.data1 = thermistor_id_all;
    txMessage.frame.data2 = thermistor_values[thermistor_index];  // Temperature in °C
    txMessage.frame.data3 = thermistor_index;                     // Thermistor ID relative to this module
    txMessage.frame.data4 = lowest_temp;  // Current temperature
    txMessage.frame.data5 = highest_temp;  // Can be used for other data
    txMessage.frame.data6 = highest_temp_id;                // Highest thermistor ID
    txMessage.frame.data7 = lowest_temp_id;                 // Lowest thermistor ID

    // Update thermistor_index for next call
    thermistor_index = (thermistor_index + 1) % NUM_THERMISTORS;

    // Transmit the CAN message
    if (CANSPI_Transmit(&txMessage) != 0)
    {
        // Handle transmission error if needed
    }
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
  __disable_irq();
  while (1)
  {
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
