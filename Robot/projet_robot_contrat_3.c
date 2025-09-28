/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//VARIABLES POUR GERER LES ETATS
volatile uint8_t GO = 0;				//bouton start

typedef enum {
    ETAT_ATTENTE,         				// robot en pause, attente commande GO
    ETAT_AVANCE,          				// robot avance
    ETAT_ROTATION_DROITE_45, 			// robot tourne à droite de 45° après un appui court
	//ETAT_ATTENTE_45,					// transition fluide de la rotation 45° à continue
	ETAT_ROTATION_DROITE_LONGUE,		// robot tourne à droite tant qu'on lui demande après un appui long
} EtatRobot;

volatile EtatRobot etat_courant = ETAT_ATTENTE; // état courant


//VARIABLES POUR GERER L'ADC
volatile uint32_t T_batt = 0;			//incrémenté par TIM16 toutes les 0,5s
volatile uint32_t V_batt = 0;			//contiendra la veleur de la tension de batterie
//quand la batterie est chargée, V_batt est à 256-1 soit 255 (255 pour 5,2V de tension de batterie)!
const uint32_t seuil = 147; 			// valeur de seuil à partir duquel on allume la LED2, quand V_batt < 3V


//VARIABLES POUR GERER LA ROTATION
volatile uint8_t compteur = 0; 			//incrémenté par TIM7 toutes les 0,5s pour mesurer la durée de l'appui
volatile uint32_t debut_rotation = 0;
volatile uint8_t en_rotation = 0;
uint32_t received = 0; 					//flag pour savoir si on a bien reçu qqch
volatile int right = 0;
volatile int temps_right_court = 0; 	//temps pour réaliser 45° de rotation, à étalonner
volatile uint8_t flag_rotation_courte = 0;


//VARIABLES POUR GERER L'UART
volatile char Buffer_commande[1];			//stocke la lettre en entrée
volatile char Buffer_precedent[1];			//stocke la lettre précédente

//VARIABLES POUR GERER L'ANTIREBOND
/*
volatile uint32_t dernier_appui_bouton_GO = 0;
volatile uint32_t dernier_appui_bouton_F = 0;
volatile uint32_t dernier_appui_bouton_R = 0;
volatile uint32_t dernier_appui_bouton_S = 0;
*/
volatile int anti_rebond = 0;


//VARIABLES POUR GERER L'ASSERVISSEMENT EN VITESSE
volatile int32_t pos_gauche_prec = 0; //position précédente à gauche
volatile int32_t pos_droite_prec = 0; //position précédente à droite

uint16_t vitesse_right = 25000;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//DECLARATION DE NOS PROTOTYPES

void avancer(int vitesse_percent);
void tourner_droite(int vitesse_percent);
void stop();
void transitions();
void Machine_Etat();

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
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  	HAL_UART_Receive_IT(&huart3, (uint8_t*)Buffer_commande, 1); // Initialisation réception UART3, déclenché dès réception d'un caractère
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, 0);

	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL); // démarre le compteur encodeur gauche
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL); // démarre le compteur encodeur droit

	HAL_TIM_Base_Start_IT(&htim6);  // lance l'interruption périodique toutes les 5ms pour l'asservissement
	HAL_TIM_Base_Start_IT(&htim16);  // lance l'interruption périodique toutes les 5ms pour T_batt et la LED2
	HAL_TIM_Base_Start_IT(&htim17);  //pour incrémenter le temps_right_court

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Vérification batterie toutes les 5 s

	  if (T_batt > 10) { //T_batt incrémenté toutes les 5 ms par TIM6
		  T_batt = 0;
		  HAL_ADC_Start(&hadc1);
		  V_batt = HAL_ADC_GetValue(&hadc1);
		  if (V_batt < seuil) {
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); // allume LED
		  } else {
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			  // si on est en batterie faible mais que la tension est revenue
			  }
	  }

	  transitions();
	  Machine_Etat();
	  HAL_Delay(10);
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 57143-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 57143;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 7-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 57143;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim7.Init.Prescaler = 611-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65467-1;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 611-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65476-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 916-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65503-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_moteur_gauche_GPIO_Port, DIR_moteur_gauche_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_moteur_droit_GPIO_Port, DIR_moteur_droit_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_moteur_gauche_Pin */
  GPIO_InitStruct.Pin = DIR_moteur_gauche_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_moteur_gauche_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_moteur_droit_Pin */
  GPIO_InitStruct.Pin = DIR_moteur_droit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_moteur_droit_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
	  if (Buffer_commande[0] == 'R') {
		  flag_rotation_courte = 1;
	  }
    HAL_UART_Receive_IT(&huart3, (unsigned char*)Buffer_commande, 1);
  }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		if (anti_rebond >= 100){ //incrémenté par TIM6 toutes les 5ms
			GO = !GO;
			anti_rebond = 0;
	    }
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
    	anti_rebond ++;
        // Lecture des positions courantes
        int32_t pos_gauche = __HAL_TIM_GET_COUNTER(&htim3);
        int32_t pos_droite = __HAL_TIM_GET_COUNTER(&htim4);

        // Calcul des vitesses (delta position sur 5 ms)
        int32_t vit_gauche = pos_gauche - pos_gauche_prec;
        int32_t vit_droite = pos_droite - pos_droite_prec;

        pos_gauche_prec = pos_gauche;
        pos_droite_prec = pos_droite;

        //on réinitialise les compteurs pour éviter les débordements
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        // Pour tester : afficher les vitesses
        //printf("Vg: %ld\tVd: %ld\r\n", vit_gauche, vit_droite);

        //ajuster l'erreur ici
        // (objectif : vitesse consigne - vitesse mesurée)

        int consigne = 50; // à régler en fonction de la consigne réelle
        int Kp = 5; //

        int erreur_gauche = consigne - vit_gauche;
        int erreur_droite = consigne - vit_droite;

        int32_t pwm_gauche = 2000 + erreur_gauche * Kp;
        int32_t pwm_droite = 2000 + erreur_droite * Kp;

        if (pwm_gauche > 40000) pwm_gauche = 40000;
        if (pwm_gauche < 0) pwm_gauche = 0;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_gauche);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_droite);

    }

    else if (htim->Instance == TIM7) {
    	compteur++;

    }

    else if (htim->Instance == TIM16) {
    	T_batt++;
    }

    else if (htim->Instance == TIM17){
    	if(etat_courant == ETAT_ROTATION_DROITE_45)
    		temps_right_court ++;
    }
}



//FONCTIONS DE MOUVEMENT

void avancer(int pulse) {
	HAL_GPIO_WritePin(DIR_moteur_gauche_GPIO_Port, DIR_moteur_gauche_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_moteur_droit_GPIO_Port, DIR_moteur_droit_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); //actualisation du pulse, gestion du côté gauche
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse); //actualisation du pulse, gestion du côté droit
}

void tourner_droite(int pulse){
	//HAL_TIM_Base_Stop_IT(&htim6); // désactiver l’asservissement

	HAL_GPIO_WritePin(DIR_moteur_droit_GPIO_Port, DIR_moteur_droit_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIR_moteur_gauche_GPIO_Port, DIR_moteur_gauche_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); //actualisation du pulse, gestion du côté gauche
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse); //actualisation du pulse, gestion du côté droit

	//HAL_TIM_Base_Start_IT(&htim6); // réactiver ensuite l'asservissement
}

void tourner_droite_45(int pulse){
	//HAL_TIM_Base_Stop_IT(&htim6); // désactiver l’asservissement

	HAL_GPIO_WritePin(DIR_moteur_droit_GPIO_Port, DIR_moteur_droit_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIR_moteur_gauche_GPIO_Port, DIR_moteur_gauche_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); //actualisation du pulse, gestion du côté gauche
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse); //actualisation du pulse, gestion du côté droit
	//HAL_TIM_Base_Start_IT(&htim6); // réactiver ensuite l'asservissement
}

void stop() {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //Arret à gauche
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); //Arret à droite
}




//FONCTIONS POUR GERER LA MACHINE D'ETAT

void transitions() {

	if (etat_courant == ETAT_ATTENTE) {

		switch(Buffer_commande[0]) {
			case 'F' : etat_courant = ETAT_AVANCE;
			break;
			default : etat_courant = ETAT_ATTENTE;
			break;
		}
		if(GO == 0)
			etat_courant = ETAT_ATTENTE;
	}

	else if (etat_courant == ETAT_AVANCE){
		switch(Buffer_commande[0]){
			case 'F' : etat_courant = ETAT_ATTENTE;
			break;
			case 'R' : etat_courant = ETAT_ROTATION_DROITE_45;
			break;
			default : etat_courant = ETAT_AVANCE;
			break;
		}
		if(GO == 0)
			etat_courant = ETAT_ATTENTE;
	}

	/*else if(etat_courant == ETAT_ROTATION_DROITE_45){
		etat_courant = ETAT_ATTENTE_45;
		if(GO == 0)
			etat_courant = ETAT_ATTENTE;
	}*/

	else if (etat_courant == ETAT_ROTATION_DROITE_45)
	{
		if (Buffer_commande[0] == 'R') {
				if(temps_right_court >= 3) { //tourne pendant 750ms (car tim17 incrémente toutes les 250ms
					etat_courant = ETAT_ROTATION_DROITE_LONGUE;
					temps_right_court = 0;
					}
				else etat_courant = ETAT_ROTATION_DROITE_45;
		}
		else {
			if(temps_right_court >= 3) { //tourne pendant 750ms (car tim17 incrémente toutes les 250ms
				etat_courant = ETAT_AVANCE;
				temps_right_court = 0;
				}
			else etat_courant = ETAT_ROTATION_DROITE_45;
		}

		if(GO == 0)
			etat_courant = ETAT_ATTENTE;
	}

	else if (etat_courant == ETAT_ROTATION_DROITE_LONGUE){
		switch(Buffer_commande[0]){
			case 'R' : etat_courant = ETAT_ROTATION_DROITE_LONGUE;
			break;
			default : etat_courant = ETAT_AVANCE;
			break;
		}
		if(GO == 0)
			etat_courant = ETAT_ATTENTE;
	}
}



//MACHINE D'ETAT
void Machine_Etat() {
	  switch (etat_courant) {

		  case ETAT_ATTENTE:
			  stop();
			  /*if (GO && received) {
				  received = 0;
			  }*/
			  break;

		  case ETAT_AVANCE:
			  avancer(25000);
			  /*if (!GO && received) {
				  received = 0;
			  }
			  if (right && !en_rotation) {
				  debut_rotation = HAL_GetTick();
				  en_rotation = 1;
			  }
			  */
			  break;

		  case ETAT_ROTATION_DROITE_45:
			  // tourner à droite pendant `temps_right_court`
			  tourner_droite_45(25000);
			  /*if ((HAL_GetTick() - debut_rotation) > temps_right_court) {
				  en_rotation = 0;
				  right = 0;
				  etat_courant = ETAT_AVANCE;
			  }*/
			  break;

		  /*
		  case ETAT_ATTENTE_45:
			  stop();
		  */

		  case ETAT_ROTATION_DROITE_LONGUE:
			  tourner_droite(25000);
			  // reste dans cet état tant que la commande "F" ou "S" ne change rien
			  /*if (!right) {
				  etat_courant = ETAT_AVANCE;
			  }*/
			  break;
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
