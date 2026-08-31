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
	uint32_t A;
	uint32_t B;
	uint32_t C;
	float_t E_d;
	float_t E_q;
	float_t Va;
	float_t Vb;
	float_t Vc;
}out;

typedef struct {
	GPIO_TypeDef *CHANNEL;
	uint32_t A;
	uint32_t B;
	uint32_t C;
}hallinput;

typedef struct {
	hallinput HALL;
	ADC_HandleTypeDef *SHUNT_CH;
}in;

typedef struct {
	volatile bool ALIGNED;
	uint16_t HALL_ERROR_0;
	uint16_t HALL_ERROR_7;
	volatile bool STOPPED_FAULT;
	volatile uint32_t STOPPED_FAULT_COUNT;
	volatile bool STOPPED;
	volatile uint32_t last_hall_edge_tick;
	uint16_t STOPPED_TIMEOUT;
	volatile uint16_t rotor_angle;
	volatile uint16_t rotor_angle_interp;
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
	uint8_t spdcnt;
	bool READY;
	volatile uint16_t tim;
	volatile uint16_t tim_last;
	uint8_t hall_state;
	float_t PWM_A_DUTY;
	float_t PWM_B_DUTY;
	float_t PWM_C_DUTY;
	volatile float_t period;
	bool MOE_ENABLE;
	volatile float_t rotor_accel;
	volatile float_t gecersiz_hall_okumasi;
	volatile bool BRAKE;
}motor_status;

typedef struct {
	int16_t HALL_SECTOR_OFFSET;
	float_t NUM_OF_POLE_PAIRS;
	uint16_t HALL_OFSET;
	volatile bool FW;
	float_t MAX_RPM_ACCEL;
	float_t Ia_offset;
	float_t Ib_offset;
	float_t Ic_offset;
	float_t MIN_RPM;
	float_t MAX_RPM;
	volatile bool CIRCULAR_LIM;
	volatile bool HIGH_Z_BREAK;
	volatile bool FF;
	float_t psi_m;
	float_t Ls;
	float_t omega_e;
	float_t hall_comp_lut[7];
}motor_params;

typedef struct
{
    int8_t hall_direction;
    uint8_t prev_hall;
    float_t prev_rpm;
    float_t prev2_rpm;
    float_t prev3_rpm;
    float_t rpm_filter_stage1;
    float_t filtered_fw_rpm;
    uint16_t prev_angle_interp;

} motor_observer;

typedef struct {
	motor_status STATUS;
	motor_params PARAMS;
	out OUT;
	in IN;
	pwm PWM;
	svpwm SVPWM;
	ref REF;
	dq_pi_params DQ_PI_PARAMS;
	speed_pi_params SPEED_PI_PARAMS;
	motor_observer OBSERVER;
}motor;

typedef struct {
    float kp;
    float ki;
} PI_Test_Params;

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
#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f
#define SQRT3_BY_2   0.866025403f
#define PI 3.14159265359f


#define I_max 33.132f

#define TIM3_CLK_HZ       72000000UL
#define TIM3_PRESCALER       720UL
#define TIM3_CNT_HZ          (TIM3_CLK_HZ / TIM3_PRESCALER)

#define TEST false
#define DQ_TEST false
#define SPEED_TEST false
#define SIMULATE_MOTOR false

#define DAC_OUT false

#define PWM_OUT false

#define SVPWM_OUT true

#if PWM_OUT && SVPWM_OUT
	#error PWM OUTPUT CONFIG ERROR
#endif
#if (TEST && DQ_TEST) || (TEST && SPEED_TEST) || (DQ_TEST && SPEED_TEST)
	#error TEST CONFIG ERROR
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
