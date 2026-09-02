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

/**
 * @brief PWM zamanlayıcı karşılaştırma (compare) kanal değerleri (A/B/C fazları).
 */
typedef struct {
	uint16_t A; /**< A fazı PWM compare değeri */
	uint16_t B; /**< B fazı PWM compare değeri */
	uint16_t C; /**< C fazı PWM compare değeri */
}pwm;

/**
 * @brief Uzay vektörü PWM (SVPWM) çıkış kanal değerleri (A/B/C fazları).
 */
typedef struct {
	uint16_t A; /**< A fazı SVPWM compare değeri */
	uint16_t B; /**< B fazı SVPWM compare değeri */
	uint16_t C; /**< C fazı SVPWM compare değeri */
}svpwm;

/**
 * @brief Hız/akım referans değerleri ve rampa adımı.
 */
typedef struct{
	volatile float_t Id;      /**< Referans d-ekseni akımı [A] */
	volatile float_t Iq;      /**< Referans q-ekseni akımı [A] */
	volatile float_t RPM;     /**< Hedef (nihai) hız referansı [RPM] */
	volatile float_t RPM_cur; /**< Rampalanmış, o anki uygulanan hız referansı [RPM] */
	volatile float_t STEP;    /**< Her hız döngüsünde RPM_cur'un RPM'e yaklaşma adımı */
}ref;

/**
 * @brief D-Q eksenindeki akım PI regülatörlerinin durumu ve katsayıları.
 */
typedef struct {
    float_t Id_integral_lim; /**< Id integral terimi için üst/alt sınır */
    float_t Iq_integral_lim; /**< Iq integral terimi için üst/alt sınır */
	float_t Iq_integral;     /**< Iq PI regülatörü integral biriktiricisi */
	float_t Id_integral;     /**< Id PI regülatörü integral biriktiricisi */
	float_t Id_kp;           /**< Id PI regülatörü oransal (P) kazancı */
	float_t Id_ki;           /**< Id PI regülatörü integral (I) kazancı */
	float_t Iq_kp;           /**< Iq PI regülatörü oransal (P) kazancı */
	float_t Iq_ki;           /**< Iq PI regülatörü integral (I) kazancı */
	float_t Iq_E;            /**< Iq hata sinyali (referans - ölçülen) */
	float_t Id_E;            /**< Id hata sinyali (referans - ölçülen) */
}dq_pi_params;

/**
 * @brief Hız döngüsü PI regülatörünün durumu ve katsayıları.
 */
typedef struct {
	uint16_t SPEED_LOOP_PERIOD_MS; /**< Hız döngüsünün çalışma periyodu [ms] */
	float_t SPEED_INTEGRAL_LIM;    /**< Hız integral teriminin sınırı */
	float_t IQ_REF_LIMIT;          /**< Hız döngüsünün üretebileceği maksimum Iq referansı */
	float_t  kp;                   /**< Hız PI regülatörü oransal (P) kazancı */
	float_t  ki;                   /**< Hız PI regülatörü integral (I) kazancı */
	float_t  Speed_integral;       /**< Hız PI regülatörü integral biriktiricisi */
	float_t E;                     /**< Hız hata sinyali (referans - ölçülen) */
}speed_pi_params;

/**
 * @brief FOC çıkış değerleri: PWM compare kayıtları ve d-q / faz gerilimleri.
 */
typedef struct {
	uint32_t A;   /**< A fazı PWM/compare kanal kaydı referansı */
	uint32_t B;   /**< B fazı PWM/compare kanal kaydı referansı */
	uint32_t C;   /**< C fazı PWM/compare kanal kaydı referansı */
	float_t E_d;  /**< PI çıkışı d-ekseni gerilim komutu [V] */
	float_t E_q;  /**< PI çıkışı q-ekseni gerilim komutu [V] */
	float_t Va;   /**< Ters Clarke/Park sonrası A fazı gerilimi [V] */
	float_t Vb;   /**< Ters Clarke/Park sonrası B fazı gerilimi [V] */
	float_t Vc;   /**< Ters Clarke/Park sonrası C fazı gerilimi [V] */

}out;

/**
 * @brief Motor ile ilişkili donanım zamanlayıcı/ADC tanıtıcıları (handle'lar).
 */
typedef struct{
	TIM_HandleTypeDef *PWM_TIMER;   /**< PWM üretimi için kullanılan zamanlayıcı */
	TIM_HandleTypeDef *HALL_TIMER;  /**< Hall sensör darbe zamanlaması için kullanılan zamanlayıcı */
	ADC_HandleTypeDef *ADC_TIMER;   /**< Enjekte edilmiş (injected) ADC dönüşümleri için kullanılan ADC */
}timer;

/**
 * @brief Hall sensör giriş pinlerinin GPIO port ve pin tanımları.
 */
typedef struct {
	GPIO_TypeDef *CHANNEL; /**< Hall sensörlerinin bağlı olduğu GPIO portu */
	uint32_t A;            /**< Hall A sinyali pin maskesi */
	uint32_t B;            /**< Hall B sinyali pin maskesi */
	uint32_t C;            /**< Hall C sinyali pin maskesi */
}hallinput;

/**
 * @brief Motora ait tüm giriş donanım tanımları (Hall sensörler ve akım şönt ADC'si).
 */
typedef struct {
	hallinput HALL;          /**< Hall sensör giriş pin tanımları */
	ADC_HandleTypeDef *SHUNT_CH; /**< Faz akımı ölçümü için kullanılan ADC (şönt) */
}in;

/**
 * @brief Motorun anlık durumunu (state) tutan yapı: hizalama, hata bayrakları,
 *        rotor açısı/hızı, ölçülen akımlar ve zamanlama bilgileri.
 */
typedef struct {
	volatile bool ALIGNED;               /**< Motor rotor hizalaması tamamlandı mı */
	uint16_t HALL_ERROR_0;               /**< Geçersiz Hall durumu (000) sayaç */
	uint16_t HALL_ERROR_7;               /**< Geçersiz Hall durumu (111) sayaç */
	volatile bool STOPPED_FAULT;         /**< Acil durdurma / arıza bayrağı */
	volatile uint32_t STOPPED_FAULT_COUNT; /**< Motorun beklenmedik şekilde durma sayısı/süresi sayacı */
	volatile bool STOPPED;               /**< Rotor şu anda hareketsiz (durmuş) mu */
	volatile uint32_t last_hall_edge_tick; /**< Son Hall kenar geçişinin HAL_GetTick() zaman damgası */
	uint16_t STOPPED_TIMEOUT;            /**< Hall kenarı gelmezse "durdu" kabul edilecek zaman aşımı [ms] */
	volatile uint16_t rotor_angle;       /**< Hall sektöründen elde edilen ham rotor açısı [derece] */
	volatile uint16_t rotor_angle_interp; /**< İki Hall kenarı arasında ara değerlenmiş (interpolasyonlu) rotor açısı [derece] */
	volatile float_t rotor_rpm;          /**< Filtrelenmiş rotor hızı [RPM] */
	volatile float_t kama_rpm;           /**< Kademe (mekanik/redüktör) hızı türetilmiş değeri [RPM] */
	float_t Id_curr;                     /**< Filtrelenmiş ölçülen d-ekseni akımı [A] */
	float_t Iq_curr;                     /**< Filtrelenmiş ölçülen q-ekseni akımı [A] */
	float_t Ia_curr;                     /**< Ham (ADC) A fazı akım okuması */
	float_t Ib_curr;                     /**< Ham (ADC) B fazı akım okuması */
	float_t Ic_curr;                     /**< Ham (ADC) C fazı akım okuması */
	float_t Ia_curr_map;                 /**< Ampere ölçeklenmiş (map edilmiş) A fazı akımı [A] */
	float_t Ib_curr_map;                 /**< Ampere ölçeklenmiş (map edilmiş) B fazı akımı [A] */
	float_t Ic_curr_map;                 /**< Ampere ölçeklenmiş (map edilmiş) C fazı akımı [A] */
	uint8_t spdcnt;                      /**< Hız döngüsü alt örnekleme (downsampling) sayacı */
	bool READY;                          /**< Akım ofset kalibrasyonu tamamlanıp sistem hazır mı */
	volatile uint16_t tim;               /**< Son ölçülen Hall periyodu (timer sayım değeri) */
	volatile uint16_t tim_last;          /**< Bir önceki Hall periyodu (timer sayım değeri) */
	uint8_t hall_state;                  /**< Güncel Hall sensör durumu (0-7 arası kod) */
	float_t PWM_A_DUTY;                  /**< A fazı PWM görev süresi (kullanım yerine göre) */
	float_t PWM_B_DUTY;                  /**< B fazı PWM görev süresi (kullanım yerine göre) */
	float_t PWM_C_DUTY;                  /**< C fazı PWM görev süresi (kullanım yerine göre) */
	volatile float_t period;             /**< Kompanzasyon uygulanmış Hall periyodu */
	bool MOE_ENABLE;                     /**< Master Output Enable (MOE) durumu */
	volatile float_t rotor_accel;        /**< Filtrelenmiş rotor açısal ivmesi */
	volatile float_t gecersiz_hall_okumasi; /**< Geçersiz Hall okuması ile ilgili yardımcı değişken */
	volatile bool BRAKE;                 /**< Aktif frenleme (rejeneratif/karşı yönlü akım) durumu */
}motor_status;

/**
 * @brief Motora özgü, genelde sabit/az değişen konfigürasyon parametreleri.
 */
typedef struct {
	float_t NUM_OF_POLE_PAIRS; /**< Motorun kutup çifti sayısı */
	uint16_t HALL_OFSET;       /**< Hall sektörü ile elektriksel açı arasındaki ofset [derece] */
	volatile bool FW;          /**< Alan zayıflatma (Field Weakening) aktif mi */
	float_t MAX_RPM_ACCEL;     /**< İzin verilen maksimum RPM ivmesi (rampa hesaplamasından türetilir) */
	float_t Ia_offset;         /**< A fazı akım ADC ofset kalibrasyon değeri */
	float_t Ib_offset;         /**< B fazı akım ADC ofset kalibrasyon değeri */
	float_t Ic_offset;         /**< C fazı akım ADC ofset kalibrasyon değeri */
	float_t MIN_RPM;           /**< Bu değerin altında hız sıfır kabul edilir (deadband) */
	float_t MAX_RPM;           /**< İzin verilen maksimum mutlak hız [RPM] */
	volatile bool CIRCULAR_LIM; /**< Dairesel (1/√3 çarpanlı) gerilim sınırlaması aktif mi */
	volatile bool HIGH_Z_BREAK; /**< Yüksek empedans (high-Z) frenleme modu aktif mi */
	volatile bool FF;           /**< İleri besleme (Feed Forward) kompanzasyonu aktif mi */
	float_t psi_m;              /**< Mıknatıs akı bağlantısı (flux linkage) [Wb] */
	float_t Ls;                 /**< Stator endüktansı [H] */
	float_t omega_e;            /**< Elektriksel açısal hız [rad/s] */
	float_t hall_comp_lut[7];   /**< Hall sektörlerine göre periyot kompanzasyon çarpanları (LUT) */
}motor_params;

/**
 * @brief Rotor açısı/hız gözlemcisi (observer) için ara durum değişkenleri.
 */
typedef struct
{
    int8_t hall_direction;        /**< Algılanan dönüş yönü (+1 / -1) */
    uint8_t prev_hall;            /**< Bir önceki Hall durumu (0-7) */
    float_t prev_rpm;             /**< Bir önceki anlık RPM ölçümü */
    float_t prev2_rpm;            /**< İki önceki anlık RPM ölçümü */
    float_t prev3_rpm;            /**< Üç önceki anlık RPM ölçümü */
    float_t rpm_filter_stage1;    /**< RPM filtrelemesinin ilk kademe çıktısı */
    float_t filtered_fw_rpm;      /**< Alan zayıflatma için filtrelenmiş |RPM| */
    uint16_t prev_angle_interp;   /**< Bir önceki interpolasyonlu rotor açısı [derece] */

} motor_observer;

/**
 * @brief Tek bir motoru temsil eden ana (üst düzey) veri yapısı. Durum,
 *        parametreler, çıkışlar, girişler, PI regülatörleri, gözlemci ve
 *        donanım handle'larının tümünü bir arada tutar.
 */
typedef struct {
	motor_status STATUS;             /**< Motorun anlık çalışma durumu */
	motor_params PARAMS;             /**< Motor konfigürasyon parametreleri */
	out OUT;                         /**< FOC çıkış değerleri */
	in IN;                           /**< Donanım giriş tanımları */
	pwm PWM;                         /**< PWM compare değerleri */
	svpwm SVPWM;                     /**< SVPWM compare değerleri */
	ref REF;                         /**< Hız/akım referansları */
	dq_pi_params DQ_PI_PARAMS;       /**< D-Q akım PI regülatörü durumu */
	speed_pi_params SPEED_PI_PARAMS; /**< Hız PI regülatörü durumu */
	motor_observer OBSERVER;         /**< Rotor açısı/hız gözlemcisi durumu */
	timer TIMER;                     /**< İlişkili zamanlayıcı/ADC handle'ları */
}motor;

/**
 * @brief Test taramalarında (sweep) kullanılan tek bir PI kazanç seti (kp, ki).
 */
typedef struct {
    float kp; /**< Oransal (P) kazanç */
    float ki; /**< İntegral (I) kazanç */
} PI_Test_Params;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/**
 * @brief STM32Cube tarafından üretilen zamanlayıcı MSP post-init fonksiyonu.
 * @param htim Post-init yapılacak zamanlayıcı handle'ı.
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Kurtarılamaz bir hata durumunda çağrılan genel hata işleyicisi.
 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/** @brief 1/√3 sabiti (Clarke dönüşümünde kullanılır). */
#define ONE_BY_SQRT3 0.577350269f
/** @brief 2/√3 sabiti (Clarke dönüşümünde kullanılır). */
#define TWO_BY_SQRT3 1.154700538f
/** @brief √3/2 sabiti (ters Clarke dönüşümünde kullanılır). */
#define SQRT3_BY_2   0.866025403f
/** @brief Pi sayısı. */
#define PI 3.14159265359f


/** @brief Şönt akım ölçümünün tam skala (maksimum) değeri [A]. */
#define I_max 33.132f

/** @brief TIM3 zamanlayıcısının giriş saat frekansı [Hz]. */
#define TIM3_CLK_HZ       72000000UL
/** @brief TIM3 zamanlayıcısının ön bölücü (prescaler) değeri. */
#define TIM3_PRESCALER       720UL
/** @brief TIM3 zamanlayıcısının ön bölücü sonrası sayım frekansı [Hz]. */
#define TIM3_CNT_HZ          (TIM3_CLK_HZ / TIM3_PRESCALER)

/** @brief Genel hız/akım tarama testini (test.c) etkinleştirir. */
#define TEST false
/** @brief D-Q akım PI kazanç tarama testini (test_dq.c) etkinleştirir. */
#define DQ_TEST false
/** @brief Hız PI kazanç tarama testini (test_spd.c) etkinleştirir. */
#define SPEED_TEST false
/** @brief Gerçek donanım yerine motor simülasyonunu etkinleştirir. */
#define SIMULATE_MOTOR false

/** @brief Rotor açısının DAC üzerinden analog olarak izlenmesini etkinleştirir. */
#define DAC_OUT false

/** @brief Klasik (üçgen dalga tabanlı) PWM çıkışını etkinleştirir. */
#define PWM_OUT false

/** @brief Uzay vektörü PWM (SVPWM) çıkışını etkinleştirir. */
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
