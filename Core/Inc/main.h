/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint16_t A;
	uint16_t B;
	uint16_t C;
}pwm;

typedef struct {
	uint16_t A;
	uint16_t B;
	uint16_t C;
}svpwm;

typedef struct{
	volatile float_t Id;
	volatile float_t Iq;
	volatile float_t RPM;
	volatile float_t RPM_cur;
	volatile float_t STEP;
	float_t RPM_lim;
}ref;

typedef struct {
    float_t Id_integral_lim;
    float_t Iq_integral_lim;
	float_t Iq_integral;
	float_t Id_integral;
	float_t Id_kp;
	float_t Id_ki;
	float_t Iq_kp;
	float_t Iq_ki;
	float_t Iq_E;
	float_t Id_E;
}dq_pi_params;

typedef struct {
	uint16_t SPEED_LOOP_PERIOD_MS;
	float_t SPEED_INTEGRAL_LIM;
	float_t IQ_REF_LIMIT;
	float_t  kp;
	float_t  ki;
	float_t  Speed_integral;
	float_t E;
}speed_pi_params;


typedef struct {
	volatile uint16_t rotor_angle;
	uint16_t rotor_angle_interp;
	volatile float_t rotor_rpm;
	volatile float_t kama_rpm;
	float_t Id_curr;
	float_t Iq_curr;
	float_t Ia_curr;
	float_t Ib_curr;
	float_t Ic_curr;
	float_t Ia_curr_map;
	float_t Ib_curr_map;
	float_t Ic_curr_map;
	float_t Va;
	float_t Vb;
	float_t Vc;
	float_t E_d;
	float_t E_q;
	uint16_t HALL_OFSET;
	uint16_t HALL_ERROR_0;
	uint16_t HALL_ERROR_7;
	float_t NUM_OF_POLE_PAIRS;
	int16_t HALL_SECTOR_OFFSET;
	volatile bool ALIGNED;
	volatile uint32_t last_hall_edge_tick;
	uint16_t STOPPED_TIMEOUT;
	volatile bool STOPPED;
	bool READY;
	pwm PWM;
	svpwm SVPWM;
	ref REF;
	dq_pi_params DQ_PI_PARAMS;
	speed_pi_params SPEED_PI_PARAMS;
	uint16_t periodlist[6];
	uint16_t periodnum;
	uint8_t spdcnt;

}motor;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
