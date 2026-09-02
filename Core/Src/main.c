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
#include "acildurum.h"
#include "foc_interrupt.h"


#if SPEED_TEST
#include "test_spd.h"
#endif
#if DQ_TEST
#include "test_dq.h"
#endif
#if TEST
#include "test.h"
#endif
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/**
 * @brief Filtrelenmiş DC bara (VBUS) gerilimi [V].
 * @note  `HAL_ADCEx_InjectedConvCpltCallback()` içinde alçak geçiren
 *        filtre ile güncellenir (bkz. foc_interrupt.c).
 */
volatile float_t V_dc = 28.0f;

/** @brief VBUS ölçüm gerilim bölücüsünün (voltage divider) oranı. */
float_t VBUS_DIVIDER_RATIO = 19.25f;


/**
 * @brief  Uygulamadaki tek motorun (Motor 1) tüm durum, parametre, PI
 *         regülatörü, gözlemci ve donanım bağlantı bilgilerini tutan
 *         ana `motor` yapısı örneği.
 *
 * Burada verilen başlangıç değerleri; Hall ofseti, kutup çifti sayısı,
 * akım sensörü ofsetleri, hız/akım PI kazançları, Hall periyodu asimetri
 * kompanzasyon tablosu (`hall_comp_lut`) ve ilgili zamanlayıcı/ADC/GPIO
 * donanım eşleştirmelerini (PWM_TIMER, HALL_TIMER, ADC_TIMER, Hall
 * pinleri) içerir.
 */
motor MOTOR_1= {
	.STATUS = {
		.ALIGNED = false,
		.HALL_ERROR_0 = 0,
		.HALL_ERROR_7 = 0,
		.STOPPED_FAULT = false,
		.STOPPED_FAULT_COUNT = 0,
		.STOPPED = true,
		.STOPPED_TIMEOUT = 300,
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
		.spdcnt = 0,
		.READY = false,
		.tim = 0,
		.tim_last = 0,
		.hall_state = 0,
		.PWM_A_DUTY =  0,
		.PWM_B_DUTY =  0,
		.PWM_C_DUTY =  0,
		.period = 0,
		.MOE_ENABLE = 1,
		.rotor_accel = 0,
		.gecersiz_hall_okumasi = 0,
		.BRAKE = false,
	},
	.PARAMS = {
		.NUM_OF_POLE_PAIRS = 2,
		.HALL_OFSET = 90,
		.FW = false,
		.MAX_RPM_ACCEL = 0,
		.Ia_offset = 1990.0f,
		.Ib_offset = 1999.0f,
		.Ic_offset = 2005.0f,
		.MIN_RPM = 10,
		.MAX_RPM = 10000,
		.CIRCULAR_LIM = true,
		.HIGH_Z_BREAK = true,
		.Ls = 0.0000321f,
		.psi_m =0.007518f,
		.FF = true,
		.omega_e = 0,
		.hall_comp_lut = {
				1.000f,
				1.042f, // State 1
				1.039f, // State 2
				0.944f, // State 3
				0.994f, // State 4
				0.988f, // State 5
				1.000f  // State 6
		}
	},

	.OUT = {
		.A = TIM_CHANNEL_1,
		.B = TIM_CHANNEL_2,
		.C = TIM_CHANNEL_3,
		.E_d = 0,
		.E_q = 0,
		.Va = 0,
		.Vb = 0,
		.Vc = 0,


	},
	.TIMER = {
		.PWM_TIMER = &htim1,
		.HALL_TIMER = &htim3,
		.ADC_TIMER = &hadc1,
	},
	.IN = {
		.HALL = {
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
		.RPM = 0,
		.RPM_cur = 0,
		.STEP = 30,
	},
	.SPEED_PI_PARAMS = {
			.SPEED_LOOP_PERIOD_MS = 5U,
			.SPEED_INTEGRAL_LIM = 400.0f,
			.Speed_integral = 0,
			.IQ_REF_LIMIT = 20.0f,
			.kp = 0.0010f,
			.ki = 0.00005f,
			.E = 0,
		},
	.DQ_PI_PARAMS = {
		.Id_integral_lim = 2800.0f,
		.Iq_integral_lim = 2800.0f,
		.Iq_integral = 0.0f,
		.Id_integral = 0.0f,
		.Id_kp = 0.06f,
		.Id_ki = 0.012f,
		.Iq_kp = 0.06f,
		.Iq_ki = 0.012f,
		.Iq_E = 0.0f,
		.Id_E = 0.0f,
	},
	.OBSERVER = {
		.hall_direction = 0,
		.prev_hall = 0,
		.prev_rpm = 0,
		.prev2_rpm = 0,
		.prev3_rpm = 0,
		.rpm_filter_stage1 = 0,
		.filtered_fw_rpm = 0,
		.prev_angle_interp = 0,


	},
};

/**
 * @brief 0-359° için önceden hesaplanmış sinüs değerleri arama tablosu
 *        (LUT). `sin_lut_hesapla()` ile doldurulur, `get_sin_cos_fast()`
 *        tarafından okunur.
 */
float_t sin_lut[360];
/** @brief VBUS örneklemesiyle ilgili son işlem zaman damgası (şu an kullanılmıyor). */
uint32_t last_vbus_tick = 0;

#if SIMULATE_MOTOR
/** @brief Motor simülasyonunda kullanılan sanal RPM değeri (SIMULATE_MOTOR açıkken). */
uint16_t sanal_rpm = 100;
/** @brief Simülasyon açı sayacı (SIMULATE_MOTOR açıkken). */
uint16_t timer = 0;
/** @brief Simülasyonda kullanılan salınım parametresi (SIMULATE_MOTOR açıkken). */
float_t salinim = 500;
/** @brief Simüle edilen motor hızı [RPM] (SIMULATE_MOTOR açıkken). */
volatile float_t sim_rpm = 0.0f;
/** @brief Simülasyon motor modelinin moment/hız kazancı (SIMULATE_MOTOR açıkken). */
volatile float_t K_TORQUE = 300.0f;
/** @brief Simülasyon motor modelinin sürtünme katsayısı (SIMULATE_MOTOR açıkken). */
volatile float_t FRICTION = 0.5f;
/** @brief Simülasyon yardımcı değişkeni (SIMULATE_MOTOR açıkken). */
uint8_t a = 0;
#endif

#if TEST || DQ_TEST || SPEED_TEST
/** @brief Test durum makinesinde bir önceki adımın zaman damgası. */
uint32_t sweep_last_tick = 0;
/** @brief İlgili testin başlayıp başlamadığını belirten bayrak. */
uint8_t sweep_started = 0;
/** @brief İlgili testin durum makinesi aşaması (0/1/2 = devam, 4 = tamam). */
uint8_t sweep_done = 0;
/** @brief Test fonksiyonlarına ortak arayüz için ayrılmış, kullanılmayan parametre. */
uint16_t new_tim;
#endif
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

/**
 * @brief  0-359° aralığındaki her tam derece için sinüs değerini önceden
 *         hesaplayıp `array` içine yazar (arama tablosu/LUT doldurma).
 *
 * @param  array  En az 360 elemanlı, doldurulacak float dizisi
 *                (tipik olarak `sin_lut`).
 *
 * @note   Kosinüs ayrıca hesaplanmaz; `get_sin_cos_fast()` aynı tablodan
 *         90° kaydırma ile kosinüs değerini de türetir.
 */
static inline void sin_lut_hesapla(float_t *array)
{
    for (int16_t i = 0; i < 360; i++) {
        array[i] = sinf((float_t)i * (float_t)(PI / 180.0f));
    }
}

/**
 * @brief  Önceden hesaplanmış `sin_lut` tablosunu kullanarak verilen açı
 *         için hızlıca sinüs ve kosinüs değerlerini döndürür.
 *
 * @param[in]  angle_deg  Açı [derece]; 360 ve üzeri değerler otomatik
 *                        olarak `% 360` ile normalize edilir.
 * @param[out] sin_val    Hesaplanan sinüs değeri buraya yazılır.
 * @param[out] cos_val    Hesaplanan kosinüs değeri buraya yazılır
 *                        (`sin_lut[angle_deg + 90°]` ile elde edilir).
 *
 * @note   `sin_lut_hesapla()` ile tablo doldurulmadan çağrılırsa geçersiz
 *         (sıfır) değerler döner.
 */
void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val)
{
	if (angle_deg >= 360) angle_deg %= 360;
    uint16_t cos_index = angle_deg + 90U;

    if (cos_index >= 360U)
        cos_index -= 360U;

    *sin_val = sin_lut[angle_deg];
    *cos_val = sin_lut[cos_index];
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	/* Sinüs/kosinüs arama tablosunu (LUT) başlangıçta bir kez hesapla. */
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

  /* Faz PWM çıkışlarını (ve varsa tümleyen N kanallarını) başlat. */
#if (PWM_OUT == true || SVPWM_OUT == true)
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.A);
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.B);
  HAL_TIM_PWM_Start(&htim1, MOTOR_1.OUT.C);


  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.A);
  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.B);
  HAL_TIMEx_PWMN_Start(&htim1, MOTOR_1.OUT.C);
#endif

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); //Vbus adc



  /* PWM çıkışlarını fiziksel olarak etkinleştir (Master Output Enable). */
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* Hall sensör input-capture kesmesini ve enjekte edilmiş ADC dönüşümünü
   * kesme (interrupt) modunda başlat. */
  HAL_TIMEx_HallSensor_Start_IT(&htim3);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
#if (DAC_OUT == true)
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
#endif

  /* Motoru başlangıç Hall pozisyonuna hizala, ardından akım sensörü
   * sıfır-nokta ofsetlerini kalibre et (2000 örnek). */
  Align_Motor(&MOTOR_1);
  Analog_Calibrate_Offsets(&MOTOR_1, 2000);



#if TEST || DQ_TEST || SPEED_TEST
  uint32_t system_start_tick = HAL_GetTick();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Ana döngüde sürekli olarak arıza/acil durdurma koşullarını ve
	   * alan zayıflatma gerekliliğini kontrol et (bkz. acildurum.c).
	   * Gerçek FOC hesaplaması ise ayrı bir kesme içinde (bkz.
	   * foc_interrupt.c / hall_interrupt.c) yürütülür. */
	  acildurum(&MOTOR_1);

	  /* Derleme zamanı seçimine göre (en fazla biri aktif) PI kazanç
	   * tarama testlerinden birini çalıştır. */
#if DQ_TEST
	  test_dq_pi(&MOTOR_1, &sweep_started, &sweep_done, &new_tim, &sweep_last_tick, system_start_tick);
#elif SPEED_TEST
	  test_speed_pi(&MOTOR_1, &sweep_started, &sweep_done, &new_tim, &sweep_last_tick, system_start_tick);
#elif TEST
	  test(&MOTOR_1, &sweep_started, &sweep_done, &new_tim, &sweep_last_tick, system_start_tick);
#endif



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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
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
  sConfigInjected.InjectedNbrOfConversion = 4;
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

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
