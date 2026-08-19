/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "math.h"
#include "stdbool.h"
#include "analog_veri_okuma.h"
#include "control.h"
#include "stdlib.h"
#include "clampf.h"
#include "map.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f
#define SQRT3_BY_2   0.866025403f
#define PI 3.14159265359f


#define V_dc 28.0f
#define I_max 33.132f

#define TIM3_CLK_HZ       72000000UL
#define TIM3_PRESCALER       720UL
#define TIM3_CNT_HZ          (TIM3_CLK_HZ / TIM3_PRESCALER)


#define SIMULATE_MOTOR false

#define DAC_OUT false

#define PWM_OUT false

#define SVPWM_OUT true



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */


bool gpiostate = false;




motor MOTOR_1= {
	.rotor_angle = 0,
	.rotor_angle_interp = 0,
	.rotor_rpm = 0,
	.kama_rpm = 0,
	.Id_curr = 0.0f,
	.Iq_curr = 0.0f,
	.Ia_curr = 0.0f,
	.Ib_curr = 0.0f,
	.Ic_curr = 0.0f,
	.Ia_curr_map = 0.0f,
	.Ib_curr_map = 0.0f,
	.Ic_curr_map = 0.0f,
	.Va = 0,
	.Vb = 0,
	.Vc = 0,
	.NUM_OF_POLE_PAIRS = 2,
	.STOPPED = true,
	.last_hall_edge_tick = 0,
	.STOPPED_FAULT = false,
	.STOPPED_FAULT_COUNT = 0,
	.STOPPED_TIMEOUT = 300,
	.HALL_OFSET = 90,
	.ALIGNED = false,
	.HALL_ERROR_0 = 0,
	.HALL_ERROR_7 = 0,
	.HALL_SECTOR_OFFSET = -30,
	.E_d = 0,
	.E_q = 5,
	.periodlist = {0,0,0,0,0,0},
	.periodnum = 0,
	.spdcnt = 0,
	.READY = false,
	.OUT = {
		.A = TIM_CHANNEL_1,
		.B = TIM_CHANNEL_2,
		.C = TIM_CHANNEL_3,
	},
	.IN = {
		.HAL = {
			.CHANNEL = GPIOC,
			.A = GPIO_PIN_6,
			.B = GPIO_PIN_7,
			.C = GPIO_PIN_8,
		},
		.SHUNT_CH = &hadc1,
	},
	.PWM = {
		.A = 0,
		.B = 0,
		.C = 0
	},
	.SVPWM = {
		.A = 0,
		.B = 0,
		.C = 0
	},
	.REF = {
		.Id = 0,
		.Iq = 0,
		.RPM = 1000,
		.RPM_cur = 0,
		.STEP = 20,
		.RPM_lim = 7500,
	},
	.SPEED_PI_PARAMS = {
			.SPEED_LOOP_PERIOD_MS = 5U,
			.SPEED_INTEGRAL_LIM = 400.0f,
			.Speed_integral = 0,
			.IQ_REF_LIMIT = 8.0f,
			.kp = 0.001f,
			.ki = 0.001f,
			.E = 0,
		},
	.DQ_PI_PARAMS = {
		.Id_integral_lim = 28.0f,
		.Iq_integral_lim = 28.0f,
		.Iq_integral = 0.0f,
		.Id_integral = 0.0f,
		.Id_kp = 1.0f,
		.Id_ki = 200.0f,
		.Iq_kp = 1.0f,
		.Iq_ki = 200.0f,
		.Iq_E = 0.0f,
		.Id_E = 0.0f,
	}
};





uint8_t hall_state;
uint16_t sanal_rpm = 100;
volatile uint16_t tim_last = 0;
volatile uint16_t tim = 0;
float_t sin_lut[360];
uint16_t timer = 0;

float_t PWM_A_DUTY =  0;
float_t PWM_B_DUTY =  0;
float_t PWM_C_DUTY =  0;

float_t salinim = 500;
volatile float_t sim_rpm = 0.0f;
volatile float_t K_TORQUE = 300.0f;
volatile float_t FRICTION = 0.5f;
uint8_t a = 0;
float_t period;
uint16_t new_tim;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



static inline void sin_lut_hesapla(float_t *array)
{
    for (int16_t i = 0; i < 360; i++) {
        array[i] = sinf((float_t)i * (float_t)(PI / 180.0f));
    }
}

static inline void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val)
{
	if (angle_deg >= 360) angle_deg %= 360;
    uint16_t cos_index = angle_deg + 90U;

    if (cos_index >= 360U)
        cos_index -= 360U;

    *sin_val = sin_lut[angle_deg];
    *cos_val = sin_lut[cos_index];
}



uint32_t ADC_Read_Regular_Channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;


    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint32_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return val;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	sin_lut_hesapla(sin_lut);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

#if (PWM_OUT == true || SVPWM_OUT == true)
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.A);
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.B);
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.C);


  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.A);
  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.B);
  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.C);
#endif

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);



  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_TIMEx_HallSensor_Start_IT(&htim3);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
#if (DAC_OUT == true)
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
#endif
  Analog_Calibrate_Offsets(&MOTOR_1, 2000);
  Align_Motor(&MOTOR_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  acildurum(&MOTOR_1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, MOTOR_1.OUT.A) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, MOTOR_1.OUT.B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, MOTOR_1.OUT.C) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1795;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 45;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//__attribute__((section(".ccmram")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
		gpiostate = !gpiostate;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, gpiostate);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, gpiostate);


	  // --------------------------- HIZ DÖNGÜSÜ -----------------------------

if(MOTOR_1.READY){
	if (MOTOR_1.spdcnt == 10){

	  static uint32_t last_sim_tick = 0;
	  static uint32_t last_speed_tick = 0;
	  uint32_t now = HAL_GetTick();

	  if (SIMULATE_MOTOR){
	        static float_t a = 0;
	        static float_t b = 0;

	        if ((now - last_sim_tick) >= (10000/sim_rpm))
	        {
	            get_sin_cos_fast(timer, &a, &b);
//	            MOTOR_1.REF.RPM = ((a * salinim) + 2000);
	            timer++;
	            if(timer == 360) timer = 0;
	            last_sim_tick = now;

	            tim_last = tim;
	            tim = __HAL_TIM_GET_COUNTER(&htim3);

	            MOTOR_1.STOPPED = false;

	            MOTOR_1.rotor_angle +=60;
	            if (MOTOR_1.rotor_angle >= 360) {
	                MOTOR_1.rotor_angle = 0;
	            }

	            MOTOR_1.last_hall_edge_tick = now;
	        }
	  }else{
		  if ((now - last_speed_tick) >= MOTOR_1.SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS)
		  		  	  {
						last_speed_tick = now;
						calculate_speed_pi(&MOTOR_1);
		  		  	  }

	        if (SIMULATE_MOTOR) {
	            float_t dt = MOTOR_1.SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS / 1000.0f;

	            sim_rpm += (K_TORQUE * MOTOR_1.REF.Iq - FRICTION * sim_rpm) * dt;
	            MOTOR_1.rotor_rpm = (int16_t)sim_rpm;

	  }}
	MOTOR_1.spdcnt = 0;

	} else {MOTOR_1.spdcnt += 1 ;};
};

	if (!MOTOR_1.ALIGNED) return;
	if (hadc->Instance == ADC1)
	{

		float_t dt = 0.00005f;
		Analog_Read_Currents(&MOTOR_1, SIMULATE_MOTOR, I_max);


		// ------------------------ FOC --------------------------
				  uint32_t current_cnt;
				  uint16_t current_tim;

				  current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
				  current_tim = tim;

		          // -----------------------------------------

				if ((HAL_GetTick() - MOTOR_1.last_hall_edge_tick) >= MOTOR_1.STOPPED_TIMEOUT) {
				    MOTOR_1.STOPPED = true;
				    MOTOR_1.rotor_rpm = 0;
				    if(MOTOR_1.REF.RPM > 100){
				    MOTOR_1.STOPPED_FAULT_COUNT++;}
				}

				if (MOTOR_1.STOPPED) {
							MOTOR_1.rotor_angle_interp = MOTOR_1.rotor_angle;
				        } else {
				        	MOTOR_1.STOPPED_FAULT_COUNT = 0;
				        	MOTOR_1.REF.RPM_cur = clampf(MOTOR_1.REF.RPM_cur, -MOTOR_1.REF.RPM_lim, MOTOR_1.REF.RPM_lim);
				            if (current_tim == 0) current_tim = 65535;

				            float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
				            if (interp_ratio > 1.0f) interp_ratio = 1.0f;

				            if (MOTOR_1.REF.RPM_cur >= 0.0f) {

				                MOTOR_1.rotor_angle_interp = MOTOR_1.rotor_angle + (uint16_t)(60.0f * interp_ratio);
				                if (MOTOR_1.rotor_angle_interp >= 360) MOTOR_1.rotor_angle_interp -= 360;
				            } else {

				                int16_t temp_angle = (MOTOR_1.rotor_angle + 60) - (int16_t)(60.0f * interp_ratio);

				                if (temp_angle < 0) temp_angle += 360;
				                else if (temp_angle >= 360) temp_angle -= 360;

				                MOTOR_1.rotor_angle_interp = (uint16_t)temp_angle;
				            }
				        }

				  float_t sin_angle;
				  float_t cos_angle;

				  get_sin_cos_fast(MOTOR_1.rotor_angle_interp + MOTOR_1.HALL_OFSET, &sin_angle, &cos_angle);

		          float_t Ia_foc = MOTOR_1.Ia_curr_map;
				  float_t Ib_foc = MOTOR_1.Ib_curr_map;
				  float_t Id_raw, Iq_raw;
				  clarke_park(Ia_foc, Ib_foc, sin_angle, cos_angle, &Id_raw, &Iq_raw);

				  MOTOR_1.Id_curr = (MOTOR_1.Id_curr * 0.7f) + (Id_raw * 0.3f);
				  MOTOR_1.Iq_curr = (MOTOR_1.Iq_curr * 0.7f) + (Iq_raw * 0.3f);

				  // -------------------- PI döngüsü ------------------
				  // Iq
				  MOTOR_1.DQ_PI_PARAMS.Iq_E = (MOTOR_1.REF.Iq - MOTOR_1.Iq_curr);
				  MOTOR_1.DQ_PI_PARAMS.Iq_integral += MOTOR_1.DQ_PI_PARAMS.Iq_E * dt;
				  MOTOR_1.DQ_PI_PARAMS.Iq_integral = clampf(MOTOR_1.DQ_PI_PARAMS.Iq_integral, - MOTOR_1.DQ_PI_PARAMS.Iq_integral_lim, MOTOR_1.DQ_PI_PARAMS.Iq_integral_lim);

				  MOTOR_1.E_q = MOTOR_1.DQ_PI_PARAMS.Iq_kp * MOTOR_1.DQ_PI_PARAMS.Iq_E + MOTOR_1.DQ_PI_PARAMS.Iq_ki * MOTOR_1.DQ_PI_PARAMS.Iq_integral;

				  // Id
				  MOTOR_1.DQ_PI_PARAMS.Id_E = (MOTOR_1.REF.Id - MOTOR_1.Id_curr);
				  MOTOR_1.DQ_PI_PARAMS.Id_integral += MOTOR_1.DQ_PI_PARAMS.Id_E * dt;
				  MOTOR_1.DQ_PI_PARAMS.Id_integral = clampf(MOTOR_1.DQ_PI_PARAMS.Id_integral, - MOTOR_1.DQ_PI_PARAMS.Id_integral_lim, MOTOR_1.DQ_PI_PARAMS.Id_integral_lim);

				  MOTOR_1.E_d = MOTOR_1.DQ_PI_PARAMS.Id_kp * MOTOR_1.DQ_PI_PARAMS.Id_E + MOTOR_1.DQ_PI_PARAMS.Id_ki * MOTOR_1.DQ_PI_PARAMS.Id_integral;

		          MOTOR_1.E_q = clampf(MOTOR_1.E_q, -V_dc, V_dc);
		          MOTOR_1.E_d = clampf(MOTOR_1.E_d, -V_dc, V_dc);
		          // --------------------------------------------------

				  inv_clarke_park(MOTOR_1.E_d, MOTOR_1.E_q, sin_angle, cos_angle, &MOTOR_1.Va, &MOTOR_1.Vb, &MOTOR_1.Vc);
		  if(SVPWM_OUT){
		  //------------------- SVPWM -------------------------

			  float_t V_max = MOTOR_1.Va;
			  float_t V_min = MOTOR_1.Va;

			  if (MOTOR_1.Vb > V_max) {V_max = MOTOR_1.Vb;}
			  if (MOTOR_1.Vc > V_max) {V_max = MOTOR_1.Vc;}

			  if (MOTOR_1.Vb < V_min) {V_min = MOTOR_1.Vb;}
			  if (MOTOR_1.Vc < V_min) {V_min = MOTOR_1.Vc;}

			  float_t V_com = -(V_max + V_min) / 2.0f;

			  MOTOR_1.SVPWM.A = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Va + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
			  MOTOR_1.SVPWM.B = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Vb + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
			  MOTOR_1.SVPWM.C = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Vc + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.A, MOTOR_1.SVPWM.A );
			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.B, MOTOR_1.SVPWM.B );
			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.C, MOTOR_1.SVPWM.C );
//		  //------------------- SVPWM -------------------------
		  } else {

			  MOTOR_1.PWM.A = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Va, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
			  MOTOR_1.PWM.B = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Vb, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
			  MOTOR_1.PWM.C = (uint16_t)clampf(map((float_t)clampf(MOTOR_1.Vc, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.A, MOTOR_1.PWM.A );
			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.B, MOTOR_1.PWM.B );
			  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_1.OUT.C, MOTOR_1.PWM.C );

			  }





#if DAC_OUT == true
			  uint32_t dac_pwm_a;
			  uint32_t dac_pwm_b;
			  if(SVPWM_OUT){
				  dac_pwm_a = (uint32_t)map((float_t)MOTOR_1.SVPWM.A, 0.0f, 1800.0f, 0.0f, 4095.0f);
				  dac_pwm_b = (uint32_t)map((float_t)MOTOR_1.SVPWM.B, 0.0f, 1800.0f, 0.0f, 4095.0f);
			  }else{
				  dac_pwm_a = (uint32_t)map((float_t)MOTOR_1.PWM.A, 0.0f, 1800.0f, 0.0f, 4095.0f);
				  dac_pwm_b = (uint32_t)map((float_t)MOTOR_1.PWM.B, 0.0f, 1800.0f, 0.0f, 4095.0f);
//				  if(a == 0){dac_pwm_b = (uint32_t)map((float_t)MOTOR_1.rotor_angle, 0.0f, 1800.0f, 0.0f, 4095.0f);}
//				  else if (a == 1){dac_pwm_b = (uint32_t)map((float_t)MOTOR_1.rotor_angle_interp, 0.0f, 1800.0f, 0.0f, 4095.0f);}
//				  else if(a == 2){dac_pwm_b = (uint32_t)map((float_t)MOTOR_1.rotor_rpm, 0.0f, 1800.0f, 0.0f, 4095.0f);}
			  }
		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_pwm_a);
		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_pwm_b);

#endif
	}

	gpiostate = !gpiostate;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, gpiostate);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, gpiostate);

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (!MOTOR_1.ALIGNED) return;

    if (htim->Instance == TIM3)
    {
		gpiostate = !gpiostate;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, gpiostate);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, gpiostate);

        uint32_t new_tim_raw = __HAL_TIM_GET_COMPARE(&htim3, MOTOR_1.OUT.A);
        period = new_tim_raw;

        if (period <= 0) period += 65536;
        if (period < 20) return;
        MOTOR_1.last_hall_edge_tick = HAL_GetTick();
        MOTOR_1.STOPPED = false;

        uint8_t hall_A = HAL_GPIO_ReadPin(MOTOR_1.IN.HAL.CHANNEL, MOTOR_1.IN.HAL.A);
        uint8_t hall_B = HAL_GPIO_ReadPin(MOTOR_1.IN.HAL.CHANNEL, MOTOR_1.IN.HAL.B);
        uint8_t hall_C = HAL_GPIO_ReadPin(MOTOR_1.IN.HAL.CHANNEL, MOTOR_1.IN.HAL.C);
        hall_state = (hall_C << 2) | (hall_B << 1) | hall_A;

        static uint8_t prev_hall = 0;
        static int8_t hall_direction = 1;

        if (prev_hall != 0 && prev_hall != hall_state) {
            if ((prev_hall == 1 && hall_state == 3) || (prev_hall == 3 && hall_state == 2) ||
                (prev_hall == 2 && hall_state == 6) || (prev_hall == 6 && hall_state == 4) ||
                (prev_hall == 4 && hall_state == 5) || (prev_hall == 5 && hall_state == 1)) {
                hall_direction = 1;
            } else {
                hall_direction = -1;
            }
        }
        prev_hall = hall_state;
        // ---------------------------

        switch(hall_state){
            case 1 : MOTOR_1.rotor_angle = 0;   break;
            case 2 : MOTOR_1.rotor_angle = 120; break;
            case 3 : MOTOR_1.rotor_angle = 60;  break;
            case 4 : MOTOR_1.rotor_angle = 240; break;
            case 5 : MOTOR_1.rotor_angle = 300; break;
            case 6 : MOTOR_1.rotor_angle = 180; break;
            case 0 : MOTOR_1.HALL_ERROR_0 += 1; break;
            case 7 : MOTOR_1.HALL_ERROR_7 += 1; break;
            default: MOTOR_1.STOPPED = true;    break;
        }

        tim = (uint16_t)period;

        float_t inst_rpm = (float_t)hall_direction * (10.0f * (float_t)TIM3_CNT_HZ) / ((float_t)period * (float_t)MOTOR_1.NUM_OF_POLE_PAIRS);
        MOTOR_1.rotor_rpm = (MOTOR_1.rotor_rpm * 0.4f) + (inst_rpm * 0.6f);
        MOTOR_1.kama_rpm = MOTOR_1.rotor_rpm / 4.5f;
		gpiostate = !gpiostate;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, gpiostate);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, gpiostate);

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
#ifdef USE_FULL_ASSERT
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
