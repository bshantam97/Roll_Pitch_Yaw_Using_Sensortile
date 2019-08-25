/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32l4xx_hal.h"
#include <lsm6dsm_reg.h>
#include <stdio.h>
#include <math.h>

#define BSP_LSM6DSM_CS_PORT					GPIOB
#define BSP_LSM6DSM_CS_PIN					GPIO_PIN_12
#define BSP_LSM6DSM_CS_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define TIMEOUT_DURATION 					1000
#define BSP_ERROR_UNKNOWN_FAILURE        	-6
#define BSP_ERROR_NONE						0
#define pi									3.141592653589793238462643383279502884f
#define GyroMeasError						pi * (5.0f / 180.0f)   // gyroscope measurement error in rads/s (start at 5 deg/s)
#define GyroMeasDrift                       pi * (0.0f  / 180.0f) // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define beta 								sqrtf(3.0f / 4.0f) * GyroMeasError
#define zeta								sqrtf(3.0f / 4.0f) * GyroMeasDrift
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int32_t BSP_LSM6DSM_WriteReg(void* handle, uint8_t Reg, uint8_t *pdata, uint16_t len);
int32_t BSP_LSM6DSM_ReadReg(void* handle, uint8_t Reg, uint8_t *pdata, uint16_t len);
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t len);
void LSM6DSM_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
void LSM6DSM_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void MadWickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
//static axis1bit16_t data_raw_temperature;
volatile float acc_x, acc_y, acc_z;			//linear acceleration in x, y, z direction
volatile float dps_gx, dps_gy, dps_gz;		//degrees per second
//static float temperature_degC;
static uint8_t whoamI;
//static uint8_t rst;
uint32_t sumCount = 0;
static uint8_t tx_buffer[200];
uint8_t offset_x = 0x09;
uint8_t offset_y = 0x09;
uint8_t offset_z = 0x05;
float deltat = 0.0f, sum = 0.0f; 		   //integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; //used to calculate integration interval
uint32_t Now = 0;					      //used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //vector to hold quaternion
float pitch, roll, yaw;				   //absolute orientation
float a12, a22, a31, a32, a33; 		   //rotation matrix coefficients for euler angles and gravity components

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int32_t xret = 0;
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	lsm6dsm_ctx_t dev_ctx;
	dev_ctx.write_reg = BSP_LSM6DSM_WriteReg;
	dev_ctx.read_reg = BSP_LSM6DSM_ReadReg;
	dev_ctx.handle = &hspi2;
	uint8_t config_data= 0x0C;
	BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_CTRL3_C, &config_data, 1); //write register to configure the SPI in 3 wire mode
	lsm6dsm_device_id_get(&dev_ctx, (uint8_t*)&whoamI);			  //
	if (whoamI != LSM6DSM_ID)
		while(1)
		{

		}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	/**RESTORE DEFAULT VALUES OF THE CONFIGURATIONS**/

	/*lsm6dsm_reset_set(&dev_ctx , PROPERTY_ENABLE);
	do
	{
		lsm6dsm_reset_get(&dev_ctx , &rst);
	}while(rst);*/

	//HAL_Delay(20);
	/**STARTUP SEQUENCE FOR ACCELEROMETER**/
	uint8_t startup_acc_gyro = 0x60;
	uint8_t int_acc = 0x01;
	BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_CTRL1_XL, &startup_acc_gyro , 1);
	BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_INT1_CTRL, &int_acc, 1);

	/**STARTUP SEQUENCE FOR GYROSCOPE**/
	uint8_t int_gyro = 0x02;
	BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_CTRL2_G, &startup_acc_gyro, 1);
	BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_INT1_CTRL, &int_gyro, 1);

	/**ENABLE BLOCK DATA UPDATE**/
	lsm6dsm_block_data_update_set(&dev_ctx , PROPERTY_ENABLE);

	/**SET ACCELEROMETER AND GYROSCOPE DATA RATE**/
	lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_12Hz5);
	lsm6dsm_gy_data_rate_set(&dev_ctx,  LSM6DSM_GY_ODR_12Hz5);

	/**SET FULL SCALE**/

	lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);
	lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_500dps);

	/*
	 * CONFIGURE FILTER CHAIN(NO AUX)
	 * ACCELEROMETER - ANALOG INTERFACE
	 */

	lsm6dsm_xl_filter_analog_set(&dev_ctx, LSM6DSM_XL_ANA_BW_400Hz);

	 /*
	  * Accelerometer - LPF1 + LPF2 path (low pass filter1 + low pass filter2)
	  */
    lsm6dsm_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100);

    /*
     * Accelerometer - High Pass / Slope path
     */
  //lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);

    /*
     * Gyroscope - filtering chain
     */

    lsm6dsm_gy_band_pass_set(&dev_ctx, LSM6DSM_HP_260mHz_LP1_STRONG);
    lsm6dsm_ctrl6_c_t CTRL6;
    SET_BIT(CTRL6.usr_off_w, 1);
    BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_X_OFS_USR, &offset_x, 3);
    BSP_LSM6DSM_WriteReg(NULL,LSM6DSM_Y_OFS_USR, &offset_y, 3);
    BSP_LSM6DSM_WriteReg(NULL, LSM6DSM_Z_OFS_USR, &offset_z, 3);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	lsm6dsm_reg_t reg;

	/*lsm6dsm_int1_ctrl_t CTRL1;
	lsm6dsm_int2_ctrl_t CTRL2;*/
	/**ENABLE DATA READY SIGNAL FOR ACCELEROMETER AND GYROSCOPE**/
	/**SET_BIT(CTRL1.int1_drdy_xl,0x03);
	SET_BIT(CTRL2.int2_drdy_g, 0x03);
	SET_BIT(CTRL1.int1_drdy_g,0x03);
	SET_BIT(CTRL2.int2_drdy_xl, 0x03);*/
	/*
	 * Read output only if new value is available
	 */

	lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda)
	{

		/*
		 * Read acceleration field data
		 */
		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);   //OUTX_L_XL Register (Linear acceleration)
		acc_x =
			(LSM6DSM_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[0])) / 1000.0f; //conversion to g's
		acc_y =
			(LSM6DSM_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[1])) / 1000.0f; //conversion to g's
 		acc_z =
			(LSM6DSM_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[2])) / 1000.0f; //conversion to g's


		sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
		              acc_x, acc_y, acc_z);
	   // tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}
	if (reg.status_reg.gda)
	{
	   /*
	    * Read angular rate field data
	    */
	    memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
	    lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);  //OUTX_L_G Register

	         dps_gx =
	    		(LSM6DSM_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[0])) / 57.1428f; //degrees per second
	         dps_gy =
	            (LSM6DSM_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[1])) / 57.1428f; //
	         dps_gz =
	            (LSM6DSM_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[2])) / 57.1428f;

	    sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",dps_gx
	                 , dps_gy, dps_gz);
	 }

	for(uint8_t i = 0 ; i<10 ; i++)
	{
		Now = HAL_GetTick();
		deltat = ((Now - lastUpdate) / 1000.0f);
		lastUpdate = Now;

		sum += deltat;

		sumCount++;

		MadWickQuaternionUpdate(-acc_x, acc_y, acc_z, dps_gx * pi/180.0f, -dps_gy * pi/180.0f , -dps_gz * pi/180.0f);
	}
	a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);

    pitch *= (180.0f / pi); //radian to degrees
    roll  *= (180.0f / pi); //radians to degrees
    yaw   *= (180.0f / pi); //radians to degrees

  /* USER CODE END 3 */

}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  __HAL_RCC_PWR_CLK_ENABLE();
	  HAL_PWR_EnableBkUpAccess();

	  /* Enable the LSE Oscilator */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
	  HAL_RCCEx_DisableLSECSS();

	  /* Enable MSI Oscillator and activate PLL with MSI as source */
	  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
	  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM            = 6;
	  RCC_OscInitStruct.PLL.PLLN            = 40;
	  RCC_OscInitStruct.PLL.PLLP            = 7;
	  RCC_OscInitStruct.PLL.PLLQ            = 4;
	  RCC_OscInitStruct.PLL.PLLR            = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable MSI Auto-calibration through LSE */
	  HAL_RCCEx_EnableMSIPLLMode();

	  /* Select MSI output as USB clock source */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	  clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; //2
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  HAL_Delay(5);
  SPI_1LINE_TX(&hspi2);
  HAL_Delay(5);
  __HAL_SPI_ENABLE(&hspi2);
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
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t len)
{
	int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

	  if(HAL_SPI_Transmit(&hspi2, pData, len, TIMEOUT_DURATION) == HAL_OK)
	  {
	      ret = len;
	  }
	  return ret;
}
int32_t BSP_LSM6DSM_WriteReg(void* handle, uint8_t Reg, uint8_t *pdata, uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	  uint8_t dataReg = (uint8_t)Reg;

	  /* CS Enable */
	  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);

	  if (BSP_SPI2_Send(&dataReg, 1) != 1)
	  {
	    ret = BSP_ERROR_UNKNOWN_FAILURE;
	  }

	  if (BSP_SPI2_Send(pdata, len) != len)
	  {
	    ret = BSP_ERROR_UNKNOWN_FAILURE;
	  }

	  /* CS Disable */
	  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

	  return ret;
}

int32_t BSP_LSM6DSM_ReadReg(void* handle, uint8_t Reg, uint8_t *pdata, uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;

	  uint8_t dataReg = (uint8_t)Reg;
	  int32_t i = 0;

	  for (i = 0; i < len; i++)
	  {
	     //CS Enable
	    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);

	    LSM6DSM_SPI_Write(&hspi2, (dataReg + i) | 0x80); //changes were made here
	    __HAL_SPI_DISABLE(&hspi2);						 //changes were made here
	    SPI_1LINE_RX(&hspi2);							 //changes were made here
	    LSM6DSM_SPI_Read(&hspi2, (pdata + i));			 //changes were made here

	     //CS Disable
	    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

	    SPI_1LINE_TX(&hspi2);
	    __HAL_SPI_ENABLE(&hspi2);
	  }
	  return ret;
}
void LSM6DSM_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
	/* check TXE flag */
	  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

	  /* Write the data */
	  *((__IO uint8_t*)&xSpiHandle->Instance->DR) = val;

	  /* Wait BSY flag */
	  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
	  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void LSM6DSM_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	  So to guarantee the clock generation for only one data, the clock must be
	  disabled after the first bit and before the latest bit */
	  /* Interrupts should be disabled during this operation */

	  __disable_irq();
	  __HAL_SPI_ENABLE(xSpiHandle);
	  __asm("dsb\n");
	  __asm("dsb\n");
	  __HAL_SPI_DISABLE(xSpiHandle);
	  __enable_irq();

	  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
	  /* read the received data */
	  __NOP();
	  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
	  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void MadWickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	//float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	//gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	//gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	//gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	//gbiasx += gerrx * deltat * zeta;
	//gbiasy += gerry * deltat * zeta;
	//gbiasz += gerrz * deltat * zeta;
	//gx -= gbiasx;
	//gy -= gbiasy;
	//gz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(beta * hatDot1)) * deltat;
	q2 += (qDot2 -(beta * hatDot2)) * deltat;
	q3 += (qDot3 -(beta * hatDot3)) * deltat;
	q4 += (qDot4 -(beta * hatDot4)) * deltat;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
