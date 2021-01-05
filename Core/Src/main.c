/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "itoc.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t DataToSend[40];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;

char ReceivedData[40];
uint8_t ReceivedDataFlag = 0;

_Bool TesterMode = 0; //tester mode - 0 = PC Mode
_Bool NormalTestFlag = 0; // =1, if normal test in progress
volatile _Bool timeOnMeasureMode; //1->measuring switch on time
volatile _Bool timeOffMeasureMode;//1->measuring switch off time

float CoilVoltage,PWMVoltage,DifVoltage,CurrentMeas;  //Measured values float
float CoilVoltageSetFl,PWMVoltageSetFl; 			  //Settings Value Float
float PrevCoilVoltageSetFl=0;
uint16_t PrevCoilVoltageSet=0;
uint16_t CoilVoltageSet,PWMVoltageSet; 				  //Settings Value
uint16_t CoilVoltageToSet,PWMVoltageToSet; 			  //Values to Set

uint32_t adc[5][101],adcbuffer[5];                    //adc - measured adc 0-4w data
float adcConverted[4],adcRaw[4];
uint32_t adcTemperature;
uint16_t min=5000,max=0;
volatile uint32_t countTime = 0;

uint8_t ssf = 10;
uint8_t sempIndex=0;

uint8_t selfCalibration=0;
uint8_t selfCalibCheck = 0;

uint32_t currTemp = 0; //delete it
uint32_t adcRawCurrent = 0;

int temperature = 0;
char Temp[18];
char LCDMessage[2][16],LCDMessageCurr[2][16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t StrCmp (char* Buf,char* cmpBuf);
void AllOff(void);
void ReadADC(void);
float StrToFloat(char* Buf);
uint16_t StrToInt(char* Buf);
void DioOn(uint8_t DioNum);
void RelOn(uint8_t RelNum);
void DioOff(uint8_t DioNum);
void RelOff(uint8_t RelNum);
void ConvertData(void);
void sampleCurrent (void);

uint32_t OnLimit =2950; //if current from ADC < OnLimit -> relay is ON
uint32_t OffLimit=2950; //if current from ADC > OnLimit -> relay is OFF 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int i=0;i<=3;i++)
		{
		adc[i][sempIndex]=adcbuffer[i];
		}
	sempIndex++;
  if (sempIndex==100)
	 {
		adcTemperature=adcbuffer[4];
		sempIndex=0;
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,32);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);  //350 - 13,5V if Power sourse 15V
 	
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
	
	LCDInit(0x40);
	HAL_Delay(100);
	
	LCDPrint("  Relay Tester  ");
	SecString();
	LCDPrint("    Rev: 2.0    ");
	HAL_Delay(1500);
	FirstString();
	LCDPrint("  Designed by:  ");
	SecString();
	LCDPrint("Ivan  Perehiniak");
	HAL_Delay(1000);
	//LCDClear();
	
	HAL_ADC_Start_DMA(&hadc1,adcbuffer,5);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  		if(ReceivedDataFlag == 1 && selfCalibration!=1)//&& NormalTestFlag == 0		
		{
				__HAL_TIM_SET_COUNTER(&htim6,0);
			  if((StrCmp(ReceivedData,"$")>0 || StrCmp(ReceivedData,"@")>0) && StrCmp(ReceivedData,"$AllOFF")==0)
					{
					sprintf(LCDMessage[0],"Test in progress");
					}
				if(StrCmp(ReceivedData,"Who are you")>0)
					{
					sprintf(LCDMessage[0],"  Wrong PC App  ");
					TesterMode = 1;
					}
				if(StrCmp(ReceivedData,"WhoAreYou")>0)
					{
					CDC_Transmit_String("I am Tester,Relay Tester");
					TesterMode = 1;
					}
				else if(StrCmp(ReceivedData,"Are you alive")>0)
					{
					TesterMode = 1;
					CDC_Transmit_String("Yes,I feel good");
					}
				else if(StrCmp(ReceivedData,"$SetCoilVoltage")>0)
					{ 
					CoilVoltageSetFl = StrToFloat(ReceivedData);
					if (CoilVoltageSetFl == PrevCoilVoltageSetFl)
						{
						CoilVoltageSet = PrevCoilVoltageSet;
						TesterMode = 1;
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
						CDC_Transmit_String("$SetCoilVoltageOK");
						}
						else
						{
						CoilVoltageSet = CoilVoltageSetFl*15;            // 25 - index, need to be changed
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
						selfCalibration = 1;
						HAL_Delay(250);
						}
					}	
				else if(StrCmp(ReceivedData,"$SetRelCurrent")>0)
					{
					PWMVoltageSetFl = StrToFloat(ReceivedData);      // current, not voltage
					PWMVoltageSet = PWMVoltageSetFl*26*PWMVoltageSetFl;            // 145 - index, need to be changed
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PWMVoltageSet);
					CDC_Transmit_String("$SetRelCurrentOK");
					}
				else if(StrCmp(ReceivedData,"$DIOON")>0)
					{ 
					DioOn(StrToInt(ReceivedData));	
					sprintf(Temp,"$DIOON%u", StrToInt(ReceivedData));	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"$RelON")>0)
					{ 
					RelOn(StrToInt(ReceivedData));	
					sprintf(Temp,"$RelOn%u", StrToInt(ReceivedData));	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"$DIOOFF")>0)
					{ 
					DioOff(StrToInt(ReceivedData));	
					sprintf(Temp,"$DIOOFF%u", StrToInt(ReceivedData));	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"$RelOFF")>0)
					{ 
					RelOff(StrToInt(ReceivedData));	
					sprintf(Temp,"$RelOff%u", StrToInt(ReceivedData));	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"@GetCurrentOFF")>0)
					{ 
					ConvertData();
					sprintf(Temp,"@CurrentOFF%.*f",4, adcConverted[3]);	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"@GetVoltageOFF")>0)
					{
					ConvertData();
					sprintf(Temp,"@VoltageOFF%.*f",4, adcConverted[2]);	
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"@GetCurrentON")>0)
					{ 
					ConvertData();
					sprintf(Temp,"@CurrentON%.*f",4, adcConverted[3]);
					CDC_Transmit_String(Temp);
					}
				else if(StrCmp(ReceivedData,"@GetVoltageON")>0)
					{
					ConvertData();
					sprintf(Temp,"@VoltageON%.*f",4, adcConverted[2]);	
					CDC_Transmit_String(Temp);
					}	
				else if(StrCmp(ReceivedData,"@RelTimeON")>0) //turn rel ON and measure time
					{ 
					sampleCurrent();
					countTime = 0;
					RelOn(StrToInt(ReceivedData));
					for(uint32_t i =0; i < 10000 ; i++)
						{
							if(adcRawCurrent<OnLimit)
							{
								countTime=i;
								i = 10000;
							}
							else sampleCurrent();
						}
					if (adcRawCurrent<OnLimit)sprintf(Temp,"@RelTimeON%lu",countTime);
					else sprintf(Temp,"@RelTimeON%lu",adcRawCurrent+10001);
					CDC_Transmit_String(Temp);
					countTime = 0;
					}
			 else if(StrCmp(ReceivedData,"@RelTimeOFF")>0) //turn rel ON and measure time
					{ 
					sampleCurrent();
					countTime = 0;
					RelOff(StrToInt(ReceivedData));
					for(uint32_t i =0; i < 10000 ; i++)
						{
							if(adcRawCurrent>OffLimit)
							{
								countTime=i;
								i = 10000;
							}
							else sampleCurrent();
						}
					if (adcRawCurrent>OffLimit)sprintf(Temp,"@RelTimeOFF%lu",countTime);
					else sprintf(Temp,"@RelTimeOFF%lu",adcRawCurrent+10001);
					CDC_Transmit_String(Temp);
					countTime = 0;
					}
				else if(StrCmp(ReceivedData,"$AllOFF")>0)   // Turn All DIO OFF, Relays after last step
					{
					AllOff();
					sprintf(LCDMessage[0]," Test completed ");
					}
					
			ReceivedDataFlag = 0;
				}	


		
	if (selfCalibration == 1)
	{
		sprintf(LCDMessage[0],"Self Calibration");
		sprintf(LCDMessage[1],"Coil Volt.=%.*f",2, adcConverted[0]);
		if ((adcConverted[0]-1) >= CoilVoltageSetFl && CoilVoltageSet > 16)
		{
		selfCalibCheck = 0;
		CoilVoltageSet -= 9;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
		}
		else if ((adcConverted[0]+1) <= CoilVoltageSetFl && CoilVoltageSet < 450)
		{
		selfCalibCheck = 0;
		CoilVoltageSet += 10;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
		}
		else if ((adcConverted[0]-0.15) >= CoilVoltageSetFl && CoilVoltageSet > 8)
		{
		selfCalibCheck = 0;
		CoilVoltageSet -= 1;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
		}
		else if ((adcConverted[0]+0.05 ) <= CoilVoltageSetFl && CoilVoltageSet < 450)
		{
		selfCalibCheck = 0;
		CoilVoltageSet += 2;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
		}else if (adcConverted[0] >= (CoilVoltageSetFl - 0.06) && adcConverted[0] <= (CoilVoltageSetFl+0.16))
		{
		selfCalibCheck++;
		if(selfCalibCheck == 15)
			{
			selfCalibCheck = 0;
			TesterMode = 1;
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CoilVoltageSet);
			CDC_Transmit_String("$SetCoilVoltageOK");
			PrevCoilVoltageSet = CoilVoltageSet;
			PrevCoilVoltageSetFl = CoilVoltageSetFl;
			selfCalibration = 0;
			}
		}
	}		
				
	if(TesterMode == 0 || StrCmp(ReceivedData,"$SetRelCurrent"))
	{
		ConvertData();
		FirstString();
		LCDPrint(LCDMessage[0]);
		SecString();
		LCDPrint(LCDMessage[1]);

		HAL_Delay(50);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 720;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 60000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2400;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 72;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Out10_Pin|Out12_Pin|Out11_Pin|Out13_Pin
                          |Out15_Pin|Out14_Pin|Out16_Pin|RelOn7_Pin
                          |RelOn8_Pin|Out1_Pin|Out3_Pin|Out2_Pin
                          |Out4_Pin|Out6_Pin|Out5_Pin|Out7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RelOn5_Pin|RelOn6_Pin|RelOn3_Pin|RelOn4_Pin
                          |RelOn1_Pin|RelOn2_Pin|Out9_Pin|Out8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RelOn9_Pin|RelOn10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Out10_Pin Out12_Pin Out11_Pin Out13_Pin
                           Out15_Pin Out14_Pin Out16_Pin RelOn7_Pin
                           RelOn8_Pin Out1_Pin Out3_Pin Out2_Pin
                           Out4_Pin Out6_Pin Out5_Pin Out7_Pin */
  GPIO_InitStruct.Pin = Out10_Pin|Out12_Pin|Out11_Pin|Out13_Pin
                          |Out15_Pin|Out14_Pin|Out16_Pin|RelOn7_Pin
                          |RelOn8_Pin|Out1_Pin|Out3_Pin|Out2_Pin
                          |Out4_Pin|Out6_Pin|Out5_Pin|Out7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : But1_Pin But2_Pin But3_Pin But4_Pin */
  GPIO_InitStruct.Pin = But1_Pin|But2_Pin|But3_Pin|But4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RelOn5_Pin RelOn6_Pin RelOn3_Pin RelOn4_Pin
                           RelOn1_Pin RelOn2_Pin Out9_Pin Out8_Pin */
  GPIO_InitStruct.Pin = RelOn5_Pin|RelOn6_Pin|RelOn3_Pin|RelOn4_Pin
                          |RelOn1_Pin|RelOn2_Pin|Out9_Pin|Out8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RelOn9_Pin RelOn10_Pin */
  GPIO_InitStruct.Pin = RelOn9_Pin|RelOn10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

uint16_t StrToInt(char* Buf)
{
char IntMas[10] = {"0123456789"};
uint16_t Len = strlen(Buf);
uint32_t TempInt = 0;
for(uint16_t i = 0; i<Len;i++)
{
	for(uint8_t j = 0; j<10;j++)
	{
		if (Buf[i] == IntMas[j])
		{
		TempInt = (TempInt*10)+j;
		break;
		}
	}
}
return TempInt;
}

float StrToFloat(char* Buf)
{
char IntMas[10] = {"0123456789"};
uint16_t Len = strlen(Buf);
float TempInt = 0;
uint8_t dotIndex = 0;

for(uint16_t i = 0; i<Len;i++)
{
	if (Buf[i] == '.' || Buf[i] == ',')
	{
	dotIndex = 1;
	i++;
	}
	for(uint8_t j = 0; j<10;j++)
	{
		if (Buf[i] == IntMas[j]&& dotIndex == 0)
		{
		TempInt = (TempInt*10)+j;
		break;
		}else if (Buf[i] == IntMas[j]&& dotIndex > 0)
		{
		TempInt = (pow(0.1, dotIndex)*j)+TempInt;
		dotIndex++;
		break;
		}
	}
}
return TempInt;
}

uint8_t StrCmp (char* Buf,char* cmpBuf)
{
	uint8_t Len = strlen(cmpBuf); 
	for(int i=0;i<Len;i++)
		{
			if (Buf[i]!=cmpBuf[i])
				{
					return 0;
				}
		}
	return 1;
}

void AllOff(void)
{
DioOff(200);
RelOff(200);
__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
}



void DioOn(uint8_t DioNum)
{
    switch (DioNum)
		{
			case 1:   HAL_GPIO_WritePin(GPIOC, Out2_Pin, GPIO_PIN_SET); break; 
			case 2:   HAL_GPIO_WritePin(GPIOC, Out4_Pin, GPIO_PIN_SET); break; 
			case 3:   HAL_GPIO_WritePin(GPIOC, Out1_Pin, GPIO_PIN_SET); break; 
			case 4:   HAL_GPIO_WritePin(GPIOC, Out3_Pin, GPIO_PIN_SET); break; 
			case 5:   HAL_GPIO_WritePin(GPIOC, Out6_Pin, GPIO_PIN_SET); break; 
			case 6:   HAL_GPIO_WritePin(GPIOC, Out5_Pin, GPIO_PIN_SET); break; 
			case 7:   HAL_GPIO_WritePin(GPIOC, Out7_Pin, GPIO_PIN_SET); break; 
			case 8:   HAL_GPIO_WritePin(GPIOB, Out9_Pin, GPIO_PIN_SET); break; 
			case 9:   HAL_GPIO_WritePin(GPIOB, Out8_Pin, GPIO_PIN_SET); break; 
			case 10:  HAL_GPIO_WritePin(GPIOC, Out10_Pin, GPIO_PIN_SET); break; 
			case 11:  HAL_GPIO_WritePin(GPIOC, Out12_Pin, GPIO_PIN_SET); break;
			case 12:  HAL_GPIO_WritePin(GPIOC, Out11_Pin, GPIO_PIN_SET); break; 
			case 13:  HAL_GPIO_WritePin(GPIOC, Out13_Pin, GPIO_PIN_SET); break;
			case 14:  HAL_GPIO_WritePin(GPIOC, Out15_Pin, GPIO_PIN_SET); break; 
			case 15:  HAL_GPIO_WritePin(GPIOC, Out14_Pin, GPIO_PIN_SET); break;
			case 16:  HAL_GPIO_WritePin(GPIOC, Out16_Pin, GPIO_PIN_SET); break;
		}		
}

void DioOff(uint8_t DioNum)
{
    switch (DioNum)
		{
			case 1:   HAL_GPIO_WritePin(GPIOC, Out2_Pin, GPIO_PIN_RESET); break; 
			case 2:   HAL_GPIO_WritePin(GPIOC, Out4_Pin, GPIO_PIN_RESET); break; 
			case 3:   HAL_GPIO_WritePin(GPIOC, Out1_Pin, GPIO_PIN_RESET); break; 
			case 4:   HAL_GPIO_WritePin(GPIOC, Out3_Pin, GPIO_PIN_RESET); break; 
			case 5:   HAL_GPIO_WritePin(GPIOC, Out6_Pin, GPIO_PIN_RESET); break; 
			case 6:   HAL_GPIO_WritePin(GPIOC, Out5_Pin, GPIO_PIN_RESET); break; 
			case 7:   HAL_GPIO_WritePin(GPIOC, Out7_Pin, GPIO_PIN_RESET); break; 
			case 8:   HAL_GPIO_WritePin(GPIOB, Out9_Pin, GPIO_PIN_RESET); break; 
			case 9:   HAL_GPIO_WritePin(GPIOB, Out8_Pin, GPIO_PIN_RESET); break; 
			case 10:  HAL_GPIO_WritePin(GPIOC, Out10_Pin, GPIO_PIN_RESET); break; 
			case 11:  HAL_GPIO_WritePin(GPIOC, Out12_Pin, GPIO_PIN_RESET); break;
			case 12:  HAL_GPIO_WritePin(GPIOC, Out11_Pin, GPIO_PIN_RESET); break; 
			case 13:  HAL_GPIO_WritePin(GPIOC, Out13_Pin, GPIO_PIN_RESET); break;
			case 14:  HAL_GPIO_WritePin(GPIOC, Out15_Pin, GPIO_PIN_RESET); break; 
			case 15:  HAL_GPIO_WritePin(GPIOC, Out14_Pin, GPIO_PIN_RESET); break;
			case 16:  HAL_GPIO_WritePin(GPIOC, Out16_Pin, GPIO_PIN_RESET); break;
			
			case 200: HAL_GPIO_WritePin(GPIOC, Out10_Pin|Out12_Pin|Out11_Pin|Out13_Pin // RESET ALL
                          |Out15_Pin|Out14_Pin|Out16_Pin|Out1_Pin|Out3_Pin|Out2_Pin 
                          |Out4_Pin|Out6_Pin|Out5_Pin|Out7_Pin, GPIO_PIN_RESET); 
								HAL_GPIO_WritePin(GPIOB, Out8_Pin|Out9_Pin, GPIO_PIN_RESET); break;
		}		
}

void RelOn(uint8_t RelNum)
{
		switch (RelNum)
		{
			case 1:HAL_GPIO_WritePin(GPIOB, RelOn1_Pin, GPIO_PIN_SET); break;
			case 2:HAL_GPIO_WritePin(GPIOB, RelOn2_Pin, GPIO_PIN_SET); break;
			case 3:HAL_GPIO_WritePin(GPIOB, RelOn3_Pin, GPIO_PIN_SET); break;
			case 4:HAL_GPIO_WritePin(GPIOB, RelOn4_Pin, GPIO_PIN_SET); break;
			case 5:HAL_GPIO_WritePin(GPIOB, RelOn5_Pin, GPIO_PIN_SET); break;
			case 6:HAL_GPIO_WritePin(GPIOB, RelOn6_Pin, GPIO_PIN_SET); break;
			case 7:HAL_GPIO_WritePin(GPIOC, RelOn7_Pin, GPIO_PIN_SET); break;
			case 8:HAL_GPIO_WritePin(GPIOC, RelOn8_Pin, GPIO_PIN_SET); break;
			case 9:HAL_GPIO_WritePin(GPIOA, RelOn9_Pin, GPIO_PIN_SET); break;
			case 10:HAL_GPIO_WritePin(GPIOA, RelOn10_Pin, GPIO_PIN_SET); break;
		}
}

void RelOff(uint8_t RelNum)
{
		switch (RelNum)
		{
			case 1:HAL_GPIO_WritePin(GPIOB, RelOn1_Pin, GPIO_PIN_RESET); break;
			case 2:HAL_GPIO_WritePin(GPIOB, RelOn2_Pin, GPIO_PIN_RESET); break;
			case 3:HAL_GPIO_WritePin(GPIOB, RelOn3_Pin, GPIO_PIN_RESET); break;
			case 4:HAL_GPIO_WritePin(GPIOB, RelOn4_Pin, GPIO_PIN_RESET); break;
			case 5:HAL_GPIO_WritePin(GPIOB, RelOn5_Pin, GPIO_PIN_RESET); break;
			case 6:HAL_GPIO_WritePin(GPIOB, RelOn6_Pin, GPIO_PIN_RESET); break;
			case 7:HAL_GPIO_WritePin(GPIOC, RelOn7_Pin, GPIO_PIN_RESET); break;
			case 8:HAL_GPIO_WritePin(GPIOC, RelOn8_Pin, GPIO_PIN_RESET); break;
			case 9:HAL_GPIO_WritePin(GPIOA, RelOn9_Pin, GPIO_PIN_RESET); break;
			case 10:HAL_GPIO_WritePin(GPIOA, RelOn10_Pin, GPIO_PIN_RESET); break;
			
			case 200:HAL_GPIO_WritePin(GPIOB, RelOn5_Pin|RelOn6_Pin|RelOn3_Pin|RelOn4_Pin|RelOn1_Pin|RelOn2_Pin, GPIO_PIN_RESET); 
							 HAL_GPIO_WritePin(GPIOC, RelOn7_Pin|RelOn8_Pin, GPIO_PIN_RESET);
							 HAL_GPIO_WritePin(GPIOA, RelOn9_Pin|RelOn10_Pin, GPIO_PIN_RESET);break;
		}
}

void ConvertData (void)
{
		adc[0][100]= adc[1][100]= adc[2][100]= adc[3][100]=0;

		for(uint8_t ind = 0; ind<100;ind++)
		 {
			adc[0][100]= adc[0][100]+adc[0][ind];
			adc[1][100]= adc[1][100]+adc[1][ind];
			adc[2][100]= adc[2][100]+adc[2][ind];
			adc[3][100]= adc[3][100]+adc[3][ind];
		 }	
		adcRaw[0] = (adc[0][100]/100)+40;
		adcRaw[1] = (adc[1][100]/100)+40;
		adcRaw[2] = adc[2][100]/100; 
		adcRaw[3] = adc[3][100]/100;  	

	
	if(adcRaw[2]>=2500)
	{
	adcConverted[2] = 9.0;
	}else
	{
	adcConverted[2] = adcRaw[2]/1240;
	}
	adcConverted[0] = adcRaw[0]/202;
	adcConverted[1] = adcRaw[1]/202;
	adcConverted[3] = ((3017-adcRaw[3])/230)-0.015;
}

void sampleCurrent (void)
{		
	  uint32_t adcSum = 0;

		for(uint8_t ind = 0; ind<20;ind++)
		 {
			adcSum = adcSum+adc[3][ind*5];
		 }	
		adcRawCurrent = adcSum/20;
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
