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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f
#define SQRT3_BY_2   0.866025403f
#define PI 3.1415f
#define V_dc 24
#define I_max 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint16_t pwm_duty_A = 0;
uint16_t pwm_duty_B = 0;
uint16_t pwm_duty_C = 0;

uint16_t adc_read_A = 0;
uint16_t adc_read_B = 0;
uint16_t adc_read_C = 0;
uint16_t adc_read_1 = 0;
uint16_t adc_read_2 = 0;
uint16_t adc_read_3 = 0;


uint16_t rotor_angle = 0;
uint16_t rotor_angle_interp = 0;
int16_t rotor_spd = 0;
uint16_t tim_last = 0;
uint16_t tim = 0;

float_t Id = 0;
float_t Iq = 0;
float_t Id_curr = 0;
float_t Iq_curr = 0;
float_t Iq_integral = 0;
float_t Id_integral = 0;


float_t Ia = 0;
float_t Ib = 0;
float_t Ic = 0;
float_t Ia_curr = 0;
float_t Ib_curr = 0;
float_t Ic_curr = 0;

float_t Iq_kp;
float_t Iq_ki;
float_t Id_kp;
float_t Id_ki;

const float sin_lut[360] = {
    0.000000f,  0.017452f,  0.034899f,  0.052336f,  0.069756f,  0.087156f,  0.104528f,  0.121869f,
    0.139173f,  0.156434f,  0.173648f,  0.190809f,  0.207912f,  0.224951f,  0.241922f,  0.258819f,
    0.275637f,  0.292372f,  0.309017f,  0.325568f,  0.342020f,  0.358368f,  0.374607f,  0.390731f,
    0.406737f,  0.422618f,  0.438371f,  0.453990f,  0.469472f,  0.484810f,  0.500000f,  0.515038f,
    0.529919f,  0.544639f,  0.559193f,  0.573576f,  0.587785f,  0.601815f,  0.615661f,  0.629320f,
    0.642788f,  0.656059f,  0.669131f,  0.681998f,  0.694658f,  0.707107f,  0.719340f,  0.731354f,
    0.743145f,  0.754710f,  0.766044f,  0.777146f,  0.788011f,  0.798636f,  0.809017f,  0.819152f,
    0.829038f,  0.838671f,  0.848048f,  0.857167f,  0.866025f,  0.874620f,  0.882948f,  0.891007f,
    0.898794f,  0.906308f,  0.913545f,  0.920505f,  0.927184f,  0.933580f,  0.939693f,  0.945519f,
    0.951057f,  0.956305f,  0.961262f,  0.965926f,  0.970296f,  0.974370f,  0.978148f,  0.981627f,
    0.984808f,  0.987688f,  0.990268f,  0.992546f,  0.994522f,  0.996195f,  0.997859f,  0.999048f,
    0.999391f,  0.999848f,  1.000000f,  0.999848f,  0.999391f,  0.999048f,  0.997859f,  0.996195f,
    0.994522f,  0.992546f,  0.990268f,  0.987688f,  0.984808f,  0.981627f,  0.978148f,  0.974370f,
    0.970296f,  0.965926f,  0.961262f,  0.956305f,  0.951057f,  0.945519f,  0.939693f,  0.933580f,
    0.927184f,  0.920505f,  0.913545f,  0.906308f,  0.898794f,  0.891007f,  0.882948f,  0.874620f,
    0.866025f,  0.857167f,  0.848048f,  0.838671f,  0.829038f,  0.819152f,  0.809017f,  0.798636f,
    0.788011f,  0.777146f,  0.766044f,  0.754710f,  0.743145f,  0.731354f,  0.719340f,  0.707107f,
    0.694658f,  0.681998f,  0.669131f,  0.656059f,  0.642788f,  0.629320f,  0.615661f,  0.601815f,
    0.587785f,  0.573576f,  0.559193f,  0.544639f,  0.529919f,  0.515038f,  0.500000f,  0.484810f,
    0.469472f,  0.453990f,  0.438371f,  0.422618f,  0.406737f,  0.390731f,  0.374607f,  0.358368f,
    0.342020f,  0.325568f,  0.309017f,  0.292372f,  0.275637f,  0.258819f,  0.241922f,  0.224951f,
    0.207912f,  0.190809f,  0.173648f,  0.156434f,  0.139173f,  0.121869f,  0.104528f,  0.087156f,
    0.069756f,  0.052336f,  0.034899f,  0.017452f,  0.000000f, -0.017452f, -0.034899f, -0.052336f,
   -0.069756f, -0.087156f, -0.104528f, -0.121869f, -0.139173f, -0.156434f, -0.173648f, -0.190809f,
   -0.207912f, -0.224951f, -0.241922f, -0.258819f, -0.275637f, -0.292372f, -0.309017f, -0.325568f,
   -0.342020f, -0.358368f, -0.374607f, -0.390731f, -0.406737f, -0.422618f, -0.438371f, -0.453990f,
   -0.469472f, -0.484810f, -0.500000f, -0.515038f, -0.529919f, -0.544639f, -0.559193f, -0.573576f,
   -0.587785f, -0.601815f, -0.615661f, -0.629320f, -0.642788f, -0.656059f, -0.669131f, -0.681998f,
   -0.694658f, -0.707107f, -0.719340f, -0.731354f, -0.743145f, -0.754710f, -0.766044f, -0.777146f,
   -0.788011f, -0.798636f, -0.809017f, -0.819152f, -0.829038f, -0.838671f, -0.848048f, -0.857167f,
   -0.866025f, -0.874620f, -0.882948f, -0.891007f, -0.898794f, -0.906308f, -0.913545f, -0.920505f,
   -0.927184f, -0.933580f, -0.939693f, -0.945519f, -0.951057f, -0.956305f, -0.961262f, -0.965926f,
   -0.970296f, -0.974370f, -0.978148f, -0.981627f, -0.984808f, -0.987688f, -0.990268f, -0.992546f,
   -0.994522f, -0.996195f, -0.997859f, -0.999048f, -0.999391f, -0.999848f, -1.000000f, -0.999848f,
   -0.999391f, -0.999048f, -0.997859f, -0.996195f, -0.994522f, -0.992546f, -0.990268f, -0.987688f,
   -0.984808f, -0.981627f, -0.978148f, -0.974370f, -0.970296f, -0.965926f, -0.961262f, -0.956305f,
   -0.951057f, -0.945519f, -0.939693f, -0.933580f, -0.927184f, -0.920505f, -0.913545f, -0.906308f,
   -0.898794f, -0.891007f, -0.882948f, -0.874620f, -0.866025f, -0.857167f, -0.848048f, -0.838671f,
   -0.829038f, -0.819152f, -0.809017f, -0.798636f, -0.788011f, -0.777146f, -0.766044f, -0.754710f,
   -0.743145f, -0.731354f, -0.719340f, -0.707107f, -0.694658f, -0.681998f, -0.669131f, -0.656059f,
   -0.642788f, -0.629320f, -0.615661f, -0.601815f, -0.587785f, -0.573576f, -0.559193f, -0.544639f,
   -0.529919f, -0.515038f, -0.500000f, -0.484810f, -0.469472f, -0.453990f, -0.438371f, -0.422618f,
   -0.406737f, -0.390731f, -0.374607f, -0.358368f, -0.342020f, -0.325568f, -0.309017f, -0.292372f,
   -0.275637f, -0.258819f, -0.241922f, -0.224951f, -0.207912f, -0.190809f, -0.173648f, -0.156434f,
   -0.139173f, -0.121869f, -0.104528f, -0.087156f, -0.069756f, -0.052336f, -0.034899f, -0.017452f
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
float_t map(float_t variable, float_t min_fm, float_t max_fm, float_t min_to, float_t max_to);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float_t map(float_t variable, float_t min_fm, float_t max_fm, float_t min_to, float_t max_to) {
  float percentage = (variable - min_fm)/(float)(max_fm - min_fm);
  float_t result = percentage*(max_to - min_to) + min_to;
  return result;}



void clarke_park(float_t Ia, float_t Ib, float_t sin_theta, float_t cos_theta, float_t *Id, float_t *Iq)
{

    float_t I_alpha = Ia;
    float_t I_beta  = (Ia * ONE_BY_SQRT3) + (Ib * TWO_BY_SQRT3);

    *Id =  (I_alpha * cos_theta) + (I_beta * sin_theta);
    *Iq = -(I_alpha * sin_theta) + (I_beta * cos_theta);
}


void inv_clarke_park(float_t Vd, float_t Vq, float_t sin_theta, float_t cos_theta, float_t *Va, float_t *Vb, float_t *Vc)
{

    float_t V_alpha = (Vd * cos_theta) - (Vq * sin_theta);
    float_t V_beta  = (Vd * sin_theta) + (Vq * cos_theta);

    *Va = V_alpha;
    *Vb = (-0.5f * V_alpha) + (SQRT3_BY_2 * V_beta);
    *Vc = (-0.5f * V_alpha) - (SQRT3_BY_2 * V_beta);
}



static inline void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val)
{
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

static inline float_t clampf(float_t x, float_t lo, float_t hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
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
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_ADCEx_InjectedStart_IT(&hadc1);

  HAL_TIMEx_HallSensor_Start_IT(&htim3);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // --------------------------- HIZ DÖNGÜSÜ -----------------------------


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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	if (hadc->Instance == ADC1)
	{

		  Ia_curr= map((float_t)(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)),(float_t)0,(float_t)4095,(float_t)-I_max,(float_t) I_max);
		  Ib_curr= map((float_t)(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)),(float_t)0,(float_t)4095,(float_t)-I_max,(float_t) I_max);
		  Ic_curr= map((float_t)(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3)),(float_t)0,(float_t)4095,(float_t)-I_max,(float_t) I_max);



		  // ------------------------ FOC --------------------------
		  uint32_t current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
		  int32_t period = (int32_t)tim - (int32_t)tim_last;
		  		  if (period <= 0) period += 65536;

		  		  int32_t time_since_cap = (int32_t)current_cnt - (int32_t)tim;
		  		  if (time_since_cap < 0) time_since_cap += 65536;

		  		  float_t interp_ratio = 0.0f;
		  		  if (period > 0) {
		  			  interp_ratio = (float_t)time_since_cap / (float_t)period;
		  			  if (interp_ratio > 1.0f) interp_ratio = 1.0f;
		  		  }

		  		  rotor_angle_interp = rotor_angle + (uint16_t)(60.0f * interp_ratio);
		            if (rotor_angle_interp >= 360) rotor_angle_interp -= 360;
//		  float_t rotor_angle_interp_rad = (float_t)rotor_angle_interp * (3.1415926535f / 180.0f);

		  float_t sin_angle;
		  float_t cos_angle;

		  get_sin_cos_fast(rotor_angle_interp, &sin_angle, &cos_angle);

		  clarke_park(Ia_curr, Ib_curr, sin_angle, cos_angle, &Id_curr, &Iq_curr);

//		  // -------------------- PI döngüsü ------------

		  		  // Iq

		  		  Iq_kp = 0.1; Iq_ki = 0.025; Id_kp = 0.1; Id_ki = 0.025;

		  		  Iq_integral += (Iq - Iq_curr);
		  		  if(Iq_integral < -100) Iq_integral = -100;
		  		  else if(Iq_integral > 100) Iq_integral = 100;

		  		  float_t E_q = Iq_kp * (Iq - Iq_curr) + Iq_ki * (Iq_integral);

		  		  // Id

		  		  Id_integral += (Id - Id_curr);
		  		  if(Id_integral < -100) Id_integral = -100;
		  		  else if(Id_integral > 100) Id_integral = 100;

		  		  float_t E_d = Id_kp * (Id - Id_curr) + Id_ki * (Id_integral);

		  // -------------------- PI döngüsü ------------------


//		  float_t E_d = 0; float_t E_q = 1;

		  inv_clarke_park(E_d, E_q, sin_angle, cos_angle, &Ia, &Ib, &Ic);

		  float_t PWM_A = clampf(map((float_t)clampf(Ia, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
		  float_t PWM_B = clampf(map((float_t)clampf(Ib, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
		  float_t PWM_C = clampf(map((float_t)clampf(Ic, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

//		  float_t PWM_A = 100
//		  float_t PWM_B = 50;
//		  float_t PWM_C = 10;




		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_A );
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_B );
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_C );
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
    	tim_last = tim;
    	tim = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);



        uint8_t hall_A = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
        uint8_t hall_B = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
        uint8_t hall_C = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

        uint8_t hall_state = (hall_C << 2) | (hall_B << 1) | hall_A;

        switch(hall_state){
        case 1 : rotor_angle = 0; break;
        case 2 : rotor_angle = 60; break;
        case 3 : rotor_angle = 120; break;
        case 4 : rotor_angle = 180; break;
        case 5 : rotor_angle = 240; break;
        case 6 : rotor_angle = 300; break;
        }

        if (period <= 0) {
                    period += 65536;
                }
        if (period > 0) {
                    rotor_spd = (int16_t)(72000000 / period);
                }
        rotor_spd = 60 / abs((float_t)tim - (float_t)tim_last);


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
