/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application,
  *                   hardware mappings, and FOC system struct definitions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
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
 * @brief Klasik PWM zamanlayıcı karşılaştırma (compare) değerleri.
 */
typedef struct {
	uint16_t A; /**< A fazı PWM compare değeri [Varsayılan: 0] */
	uint16_t B; /**< B fazı PWM compare değeri [Varsayılan: 0] */
	uint16_t C; /**< C fazı PWM compare değeri [Varsayılan: 0] */
}pwm;

/**
 * @brief Uzay Vektörü PWM (SVPWM) çıkış kanal değerleri.
 */
typedef struct {
	uint16_t A; /**< A fazı SVPWM compare değeri [Varsayılan: 0] */
	uint16_t B; /**< B fazı SVPWM compare değeri [Varsayılan: 0] */
	uint16_t C; /**< C fazı SVPWM compare değeri [Varsayılan: 0] */
}svpwm;

/**
 * @brief Hız ve akım döngüleri için referans (hedef) komutları.
 */
typedef struct{
	volatile float_t Id;      /**< Referans d-ekseni (Akı) akımı [Varsayılan: 0.0f A] */
	volatile float_t Iq;      /**< Referans q-ekseni (Tork) akımı [Varsayılan: 0.0f A] */
	volatile float_t RPM;     /**< Hedeflenen nihai hız referansı [Varsayılan: 0.0f RPM] */
	volatile float_t RPM_cur; /**< Rampa ile yumuşatılmış, anlık uygulanan hız referansı [Varsayılan: 0.0f RPM] */
	volatile float_t STEP;    /**< Her hız döngüsünde (5ms) RPM_cur'un artış/azalış adımı [Varsayılan: 30.0f] */
}ref;

/**
 * @brief D-Q eksenindeki Akım PI regülatörlerinin durumu ve kazanç katsayıları.
 */
typedef struct {
	float_t Id_integral_lim; /**< Id integral terimi için dinamik sınır [Varsayılan: 2800.0f] */
	float_t Iq_integral_lim; /**< Iq integral terimi için dinamik sınır [Varsayılan: 2800.0f] */
	float_t Iq_integral;     /**< Iq PI regülatörü integral biriktiricisi [Varsayılan: 0.0f] */
	float_t Id_integral;     /**< Id PI regülatörü integral biriktiricisi [Varsayılan: 0.0f] */
	float_t Id_kp;           /**< Id (Akı) regülatörü Oransal (P) kazancı [Varsayılan: 0.06f] */
	float_t Id_ki;           /**< Id (Akı) regülatörü İntegral (I) kazancı [Varsayılan: 0.012f] */
	float_t Iq_kp;           /**< Iq (Tork) regülatörü Oransal (P) kazancı [Varsayılan: 0.06f] */
	float_t Iq_ki;           /**< Iq (Tork) regülatörü İntegral (I) kazancı [Varsayılan: 0.012f] */
	float_t Iq_E;            /**< Iq ekseni anlık hatası (Ref - Ölçülen) [Varsayılan: 0.0f] */
	float_t Id_E;            /**< Id ekseni anlık hatası (Ref - Ölçülen) [Varsayılan: 0.0f] */
	float_t Vq_ff;           /**< Q ekseni İleri Besleme (BEMF) gerilimi [Varsayılan: 0.0f V] */
	float_t Vd_ff;           /**< D ekseni İleri Besleme (BEMF) gerilimi [Varsayılan: 0.0f V] */
}dq_pi_params;

/**
 * @brief Hız (RPM) döngüsü PI regülatörünün durumu ve kazanç katsayıları.
 */
typedef struct {
	uint16_t SPEED_LOOP_PERIOD_MS; /**< Hız döngüsünün çalışma periyodu [Varsayılan: 5 ms] */
	float_t SPEED_INTEGRAL_LIM;    /**< Hız integrali için Anti-Windup sınırı [Varsayılan: 400.0f] */
	float_t IQ_REF_LIMIT;          /**< Motorun çekebileceği maksimum tork akımı [Varsayılan: 20.0f A] */
	float_t kp;                    /**< Hız regülatörü Oransal (P) kazancı [Varsayılan: 0.005f] */
	float_t ki;                    /**< Hız regülatörü İntegral (I) kazancı [Varsayılan: 0.00001f] */
	float_t Speed_integral;        /**< Hız PI regülatörü integral biriktiricisi [Varsayılan: 0.0f] */
	float_t E;                     /**< Hız ekseni anlık hatası (Ref - Ölçülen) [Varsayılan: 0.0f] */
}speed_pi_params;

/**
 * @brief FOC çıkış değerleri: PWM kanalları ve statik faz gerilimleri.
 */
typedef struct {
	uint32_t A;   /**< A fazı donanım kanal kaydı [Varsayılan: TIM_CHANNEL_1] */
	uint32_t B;   /**< B fazı donanım kanal kaydı [Varsayılan: TIM_CHANNEL_2] */
	uint32_t C;   /**< C fazı donanım kanal kaydı [Varsayılan: TIM_CHANNEL_3] */
	float_t E_d;  /**< PI çıkışı D ekseni gerilim komutu [Varsayılan: 0.0f V] */
	float_t E_q;  /**< PI çıkışı Q ekseni gerilim komutu [Varsayılan: 0.0f V] */
	float_t Va;   /**< Ters Dönüşüm sonrası A fazı gerilimi [Varsayılan: 0.0f V] */
	float_t Vb;   /**< Ters Dönüşüm sonrası B fazı gerilimi [Varsayılan: 0.0f V] */
	float_t Vc;   /**< Ters Dönüşüm sonrası C fazı gerilimi [Varsayılan: 0.0f V] */
}out;

/**
 * @brief Motor kontrolünde kullanılan STM32 Çevre Birimi (Peripheral) handle'ları.
 */
typedef struct{
	TIM_HandleTypeDef *PWM_TIMER;   /**< 3-Faz PWM üretimi zamanlayıcısı [Varsayılan: &htim1] */
	TIM_HandleTypeDef *HALL_TIMER;  /**< Hall sensör Input Capture zamanlayıcısı [Varsayılan: &htim3] */
	ADC_HandleTypeDef *ADC_TIMER;   /**< Enjekte Akım/Bara okuma ADC'si [Varsayılan: &hadc1] */
}timer;

/**
 * @brief Hall sensör giriş pinlerinin STM32 donanım (GPIO) eşleştirmesi.
 */
typedef struct {
	GPIO_TypeDef *CHANNEL; /**< Hall sensör GPIO Portu [Varsayılan: GPIOC] */
	uint32_t A;            /**< Hall H1 (A) sinyali [Varsayılan: GPIO_PIN_6] */
	uint32_t B;            /**< Hall H2 (B) sinyali [Varsayılan: GPIO_PIN_7] */
	uint32_t C;            /**< Hall H3 (C) sinyali [Varsayılan: GPIO_PIN_8] */
}hallinput;

/**
 * @brief FOC giriş sensörlerinin donanım eşleştirmeleri.
 */
typedef struct {
	hallinput HALL;              /**< Hall sensör GPIO tanımları */
	ADC_HandleTypeDef *SHUNT_CH; /**< Şönt dirençleri okuma ADC'si [Varsayılan: &hadc1] */
}in;

/**
 * @brief Motorun operasyonel durum (State Machine) değişkenleri ve filtreli sensör verileri.
 */
typedef struct {
	volatile bool ALIGNED;                 /**< Rotor manyetik alana kilitlendi mi? [Varsayılan: false] */
	volatile uint16_t HALL_ERROR_0;        /**< '000' Geçersiz Hall Okuma sayacı [Varsayılan: 0] */
	volatile uint16_t HALL_ERROR_7;        /**< '111' Geçersiz Hall Okuma sayacı [Varsayılan: 0] */
	volatile bool STOPPED_FAULT;           /**< Acil durdurma (Düşük voltaj/Hata) [Varsayılan: false] */
	volatile uint32_t STOPPED_FAULT_COUNT; /**< Beklenmedik durma hata süresi/sayacı [Varsayılan: 0] */
	volatile bool STOPPED;                 /**< Motor fiziksel olarak duruyor mu? [Varsayılan: true] */
	volatile uint32_t last_hall_edge_tick; /**< Son Hall kenarı zaman damgası (ms) */
	volatile uint16_t STOPPED_TIMEOUT;     /**< Motoru 'Durmuş' kabul etmek için zaman aşımı [Varsayılan: 300 ms] */
	volatile uint16_t rotor_angle;         /**< Hall sensöründen alınan ham elektriksel açı [Varsayılan: 0°] */
	volatile uint16_t rotor_angle_interp;  /**< Hibrit sistemle hesaplanan kesintisiz açı [Varsayılan: 0°] */
	volatile float_t rotor_rpm;            /**< Filtrelenmiş anlık motor devri [Varsayılan: 0.0f RPM] */
	volatile float_t kama_rpm;             /**< Redüktör/Mekanik kademe sonrası çıkış devri [Varsayılan: 0.0f RPM] */
	volatile float_t Id_curr;              /**< Clarke/Park sonrası D-Ekseni akımı [Varsayılan: 0.0f A] */
	volatile float_t Iq_curr;              /**< Clarke/Park sonrası Q-Ekseni akımı [Varsayılan: 0.0f A] */
	volatile float_t Ia_curr;              /**< A Fazı Ham ADC (Kalibrasyon öncesi) değeri */
	volatile float_t Ib_curr;              /**< B Fazı Ham ADC (Kalibrasyon öncesi) değeri */
	volatile float_t Ic_curr;              /**< C Fazı Ham ADC (Kalibrasyon öncesi) değeri */
	volatile float_t Ia_curr_map;          /**< A Fazı Ampere (A) ölçeklenmiş akımı */
	volatile float_t Ib_curr_map;          /**< B Fazı Ampere (A) ölçeklenmiş akımı */
	volatile float_t Ic_curr_map;          /**< C Fazı Ampere (A) ölçeklenmiş akımı */
	volatile uint8_t spdcnt;               /**< Hız kontrolcüsü (2kHz) alt örnekleme sayacı [Varsayılan: 0] */
	volatile bool READY;                   /**< ADC Kalibrasyonu bitti / Sistem hazır [Varsayılan: false] */
	volatile uint16_t tim;                 /**< Timer'dan okunan son Hall periyodu [Varsayılan: 0] */
	volatile uint16_t tim_last;            /**< Bir önceki Hall periyodu [Varsayılan: 0] */
	volatile uint8_t hall_state;           /**< Aktif Hall kodu (1-6 arası) [Varsayılan: 0] */
	float_t PWM_A_DUTY;                    /**< A Fazı Duty Cycle değeri */
	float_t PWM_B_DUTY;                    /**< B Fazı Duty Cycle değeri */
	float_t PWM_C_DUTY;                    /**< C Fazı Duty Cycle değeri */
	volatile float_t period;               /**< Kompanzasyon uygulanmış net periyot */
	volatile bool MOE_ENABLE;              /**< Master Output (PWM Sürücü) aktif [Varsayılan: true (1)] */
	volatile float_t rotor_accel;          /**< Filtrelenmiş açısal ivme (RPM/s) [Varsayılan: 0.0f] */
	volatile float_t gecersiz_hall_okumasi;/**< Gürültü/Hatalı Hall değişim bayrağı */
	volatile bool BRAKE;                   /**< Aktif elektronik fren devrede mi? [Varsayılan: false] */
	volatile float_t advance_angle;        /**< Yüksek hız faz ilerletme açısı (Phase Advance) */
	volatile float_t foc_sin;              /**< FOC dönüşümleri için hesaplanmış Rotor Sinüs değeri */
	volatile float_t foc_cos;              /**< FOC dönüşümleri için hesaplanmış Rotor Kosinüs değeri */
	volatile float_t inst_rpm;
}motor_status;

/**
 * @brief Motora ve Mekaniğe özgü kalibrasyon/konfigürasyon (Sabit) parametreleri.
 */
typedef struct {
	float_t NUM_OF_POLE_PAIRS;  /**< Motorun manyetik kutup çifti sayısı [Varsayılan: 2.0f] */
	uint16_t HALL_OFSET;        /**< Sensör ile elektriksel sıfır noktası arası ofset [Varsayılan: 90°] */
	volatile bool FW;           /**< Alan Zayıflatma (Field Weakening) devrede mi? [Varsayılan: false] */
	float_t MAX_RPM_ACCEL;      /**< İzin verilen Max İvme (Rampadan hesaplanır) [Varsayılan: 0.0f] */
	float_t Ia_offset;          /**< A fazı Op-Amp (ADC) sıfır ofseti [Varsayılan: 1990.0f] */
	float_t Ib_offset;          /**< B fazı Op-Amp (ADC) sıfır ofseti [Varsayılan: 1999.0f] */
	float_t Ic_offset;          /**< C fazı Op-Amp (ADC) sıfır ofseti [Varsayılan: 2005.0f] */
	float_t MIN_RPM;            /**< PI regülatörü için ölü bant alt sınırı [Varsayılan: 10.0f RPM] */
	float_t MAX_RPM;            /**< Motorun çıkabileceği maksimum mekanik hız [Varsayılan: 10000.0f RPM] */
	volatile bool CIRCULAR_LIM; /**< SVPWM dairesel gerilim (%86) sınırlaması [Varsayılan: true] */
	volatile bool HIGH_Z_BREAK; /**< Frenlemede Yüksek Empedans (Serbest Duruş) [Varsayılan: true] */
	volatile bool FF;           /**< İleri Besleme (Feed Forward) gerilim kompanzasyonu [Varsayılan: true] */
	float_t psi_m;              /**< Mıknatıs akı (Flux Linkage) sabiti [Varsayılan: 0.007518f Wb] */
	float_t Ls;                 /**< Faz (Stator) endüktansı [Varsayılan: 0.0000321f H] */
	float_t omega_e;            /**< Motorun elektriksel açısal hızı (Radyan/s) */
	float_t hall_comp_lut[7];   /**< 120° Hall asimetrisi için düzeltme (Kompanzasyon) çarpanları */
	uint16_t MAX_WO_FW;         /**< Alan zayıflatma başlamadan önceki tepe hız [Varsayılan: 8500 RPM] */
	dq_pi_params DQ_PI;         /**< Akım (FOC) döngüsü PI parametreleri bloğu */
	speed_pi_params SPEED_PI;   /**< Hız (Devir) döngüsü PI parametreleri bloğu */
	bool FW_main;
}motor_params;

/**
 * @brief Hız, ivme ve dönüş yönü hesabı (Gözlemci) algoritmaları için geçmiş veriler.
 */
typedef struct
{
	int8_t hall_direction;      /**< Tespit edilen rotasyon yönü (+1 / -1) [Varsayılan: 0] */
	uint8_t prev_hall;          /**< Bir önceki okunan Hall durumu [Varsayılan: 0] */
	float_t prev_rpm;           /**< T-1 anındaki anlık RPM [Varsayılan: 0.0f] */
	float_t prev2_rpm;          /**< T-2 anındaki anlık RPM [Varsayılan: 0.0f] */
	float_t prev3_rpm;          /**< T-3 anındaki anlık RPM [Varsayılan: 0.0f] */
	float_t rpm_filter_stage1;  /**< Kademeli hız filtresinin ara değeri [Varsayılan: 0.0f] */
	float_t filtered_fw_rpm;    /**< Alan Zayıflatma (FW) algoritması için filtrelenmiş Mutlak RPM */
	uint16_t prev_angle_interp; /**< Extrapolasyon için bir önceki hesaplanmış açı [Varsayılan: 0°] */
} motor_observer;

/**
 * @brief Sistem sağlığı, hata ayıklama ve performans izleme (Diagnostik/Telemetri) verileri.
 */
typedef struct{
	float_t shunt_akim_kaymasi; /**< KCL yasasına göre (Ia+Ib+Ic=0) 3-Şönt toplamındaki anlık sapma [A] */
	float_t shunt_sagligi;      /**< Şönt ölçüm doğruluğunun tam skalaya (i_max) göre yüzdesel sağlığı [%] */
	float_t speed_error;        /**< Hız (Dış çevrim) PI'sinin anlık hatası (Ref RPM - Gerçek RPM) [RPM] */
	float_t iq_error;           /**< Tork (İç çevrim) PI'sinin anlık akım hatası (Ref Iq - Gerçek Iq) [A] */
	float_t id_error;           /**< Akı (İç çevrim) PI'sinin anlık akım hatası (Ref Id - Gerçek Id) [A] */
	float_t angle_error;        /**< Hall sensör ham açısı ile Serbest İntegratör (Sanal) açısı arasındaki anlık sapma [Derece] */
	float_t mod_index;          /**< Modülasyon İndeksi (Kullanılan Voltaj / Max Bara Voltajı) [%] */
	float_t power_w;            /**< Çekilen anlık tahmini elektriksel güç (V_dc * Iq) [W] */
	uint16_t foc_time_us;       /**< FOC kesme (ISR) fonksiyonunun hesaplama süresi. 20kHz periyot (<50µs) içine sığmalıdır [µs] */
	uint16_t hall_time_us;      /**< Hall sensör kenar tetiklemeli (ISR) fonksiyonunun hesaplama süresi [µs] */
	float_t hall_period_jitter; /**< Ardışık iki Hall periyodu arasındaki farkın (|period - eski_period|) filtrelenmiş ortalaması. Gürültü ve asimetri teşhisi için. */
} diag;

/**
 * @brief Servo sistemi oluşturan tüm donanım, durum ve algoritma değişkenlerini
 *        kapsayan ana (Top-Level) veri yapısı.
 */
typedef struct {
	motor_status STATUS;     /**< Motor çalışma (Run-Time) durum bayrakları ve sensörler */
	motor_params PARAMS;     /**< Kalibrasyon, sınırlar ve motor mekanik parametreleri */
	out OUT;                 /**< Hesaplanan FOC çıkış voltajları ve donanım hedefleri */
	in IN;                   /**< Giriş okuma (ADC/Hall) donanım çevre birimleri */
	pwm PWM;                 /**< Klasik PWM Duty Cycle compare değerleri */
	svpwm SVPWM;             /**< Uzay Vektörü (SVPWM) Duty Cycle compare değerleri */
	ref REF;                 /**< Hedeflenen Hız (RPM) ve Akım (Id/Iq) referansları */
	motor_observer OBSERVER; /**< Hız filtreleme ve yön tespiti geçmiş (History) buffer'ı */
	timer TIMER;             /**< İşlemci Zamanlayıcı (TIM) ve ADC Peripheral handle'ları */
	diag DIAG;				 /**< Sistem sağlığı ve doğruluğunu izlemek adına verileri tutan veri yapısı */
} motor;

/**
 * @brief Oto-Tuning (Tarama) yazılımları için geçici PI test yapısı.
 */
typedef struct {
    float kp; /**< Test edilen Oransal (P) kazanç */
    float ki; /**< Test edilen İntegral (I) kazanç */
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
 * @brief Kurtarılamaz bir donanım/yazılım hatası durumunda çağrılan işleyici (Kilitlenir).
 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/** @brief \f$1/\sqrt{3}\f$ sabiti (3-Faz Clarke Dönüşüm Katsayısı). */
#define ONE_BY_SQRT3 0.577350269f
/** @brief \f$2/\sqrt{3}\f$ sabiti (SVPWM ve Clarke Dönüşüm Katsayısı). */
#define TWO_BY_SQRT3 1.154700538f
/** @brief \f$\sqrt{3}/2\f$ sabiti (Ters Clarke Dönüşüm Katsayısı). */
#define SQRT3_BY_2   0.866025403f
/** @brief Pi sayısı (Açısal Hız/Radyan hesaplamaları için). */
#define PI 3.14159265359f

/** @brief Donanım Şönt direnci ve Op-Amp kazancına göre okunan maksimum akım sınırı [A]. */
#define I_max 33.132f

/** @brief TIM3 Input Capture zamanlayıcısının ana osilatör frekansı [Hz]. */
#define TIM3_CLK_HZ       72000000UL
/** @brief TIM3 zamanlayıcısı prescaler (ön bölücü) değeri. */
#define TIM3_PRESCALER       720UL
/** @brief TIM3 zamanlayıcısının çalışma periyodu (Çözünürlük) [Hz]. */
#define TIM3_CNT_HZ          (TIM3_CLK_HZ / TIM3_PRESCALER)

/** @brief Genel hız/akım tarama testini (test.c) etkinleştirir. */
#define TEST true
/** @brief D-Q akım PI kazanç Oto-Tuning testini (test_dq.c) etkinleştirir. */
#define DQ_TEST false
/** @brief Hız PI kazanç Oto-Tuning testini (test_spd.c) etkinleştirir. */
#define SPEED_TEST false
/** @brief Donanım olmadan sanal motor simülasyonunu etkinleştirir (Sadece yazılım testi). */
#define SIMULATE_MOTOR false

/** @brief Gerçek zamanlı rotor açısının DAC kanallarından osiloskoba aktarılmasını açar. */
#define DAC_OUT false

/** @brief Standart Üçgen dalga (Sinüzoidal) PWM modülasyonu. */
#define PWM_OUT false

/** @brief Uzay Vektörü (Space Vector - SVPWM) modülasyonu (%15 bara kazancı sağlar). */
#define SVPWM_OUT true

#if PWM_OUT && SVPWM_OUT
	#error PWM OUTPUT CONFIG ERROR: Hem klasik PWM hem SVPWM ayni anda secilemez!
#endif
#if (TEST && DQ_TEST) || (TEST && SPEED_TEST) || (DQ_TEST && SPEED_TEST)
	#error TEST CONFIG ERROR: Ayni anda yalnizca bir Oto-Tuning testi aktif edilebilir!
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
