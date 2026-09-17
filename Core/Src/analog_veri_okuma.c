/**
 * @file    analog_veri_okuma.c
 * @brief   Faz akımı okuma ve akım sensörü ofset kalibrasyonu
 *          fonksiyonlarının uygulaması.
 */

#include "analog_veri_okuma.h"







extern TIM_HandleTypeDef htim1;


/**
 * @brief  Üç faz akımını okur ve önceden hesaplanmış bir ölçek katsayısı ile
 *         Amper (A) cinsine dönüştürür.
 *
 * @details `SIMULATE_MOTOR` derleme zamanı seçeneği etkinse gerçek ADC
 *          donanımı yerine sabit orta nokta (2048) ham değerleri kullanılır;
 *          aksi halde `m->IN.SHUNT_CH` üzerindeki enjekte edilmiş ADC
 *          kanallarından (Rank 1/2/3) A/B/C faz akımlarının ham değerleri
 *          (`Ia_curr`, `Ib_curr`, `Ic_curr`) okunur.
 *
 *          Ham değerler, kalibre edilmiş sıfır-akım ofsetlerine
 *          (`PARAMS.Ia_offset`/`Ib_offset`/`Ic_offset`) göre 2048 orta
 *          noktasına taşınır ve `PARAMS.SHUNT_MULT` ölçek katsayısı ile
 *          çarpılıp `PARAMS.SHUNT_DIVIDER_RATIO` eklenerek nihai Amper
 *          değerlerine (`Ia_curr_map`, `Ib_curr_map`, `Ic_curr_map`)
 *          dönüştürülür.
 *
 * @param  m  Ham ve ölçeklenmiş akım değerlerinin okunup yazılacağı motor
 *            yapısına işaretçi.
 */
void Analog_Read_Currents(motor *m)
{
#if SIMULATE_MOTOR
        m->STATUS.Ia_curr = 2048.0f;
        m->STATUS.Ib_curr = 2048.0f;
        m->STATUS.Ic_curr = 2048.0f;
#else
        m->STATUS.Ia_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_1);
        m->STATUS.Ib_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_2);
        m->STATUS.Ic_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_3);
#endif

    float_t multiplier = m->PARAMS.SHUNT_MULT;

    m->STATUS.Ia_curr_map = (m->STATUS.Ia_curr + (2048.0f - m->PARAMS.Ia_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;
    m->STATUS.Ib_curr_map = (m->STATUS.Ib_curr + (2048.0f - m->PARAMS.Ib_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;
    m->STATUS.Ic_curr_map = (m->STATUS.Ic_curr + (2048.0f - m->PARAMS.Ic_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;


}
/**
 * @brief  Akım sensörü (şönt) sıfır-akım ofsetlerini kalibre eder.
 *
 * @details PWM çıkışları önce 0'a çekilir ve rotorun/sürücünün oturması için
 *          10 ms beklenir. Ardından `calib_samples` adet örnek boyunca,
 *          her örnek arasında 1 ms bekleyerek, üç fazın enjekte ADC ham
 *          değerleri toplanır. Toplamların örnek sayısına bölünmesiyle elde
 *          edilen ortalamalar `PARAMS.Ia_offset`/`Ib_offset`/`Ic_offset`
 *          alanlarına yazılır. Kalibrasyon tamamlandığında `STATUS.READY`
 *          bayrağı `true` yapılır.
 *
 * @param  m              Ofset değerlerinin yazılacağı motor yapısına
 *                         işaretçi.
 * @param  calib_samples  Ortalaması alınacak örnek sayısı.
 */
void Analog_Calibrate_Offsets(motor *m, uint16_t calib_samples){
	__HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, 0.0f);
	__HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, 0.0f);
	__HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, 0.0f);
    HAL_Delay(10);
	    uint32_t sum_Ia = 0, sum_Ib = 0, sum_Ic = 0;
	    for(int i = 0; i < calib_samples; i++) {
	        sum_Ia += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_1);
	        sum_Ib += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_2);
	        sum_Ic += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_3);
	        HAL_Delay(1);
	    }
	    m->PARAMS.Ia_offset = (float_t)sum_Ia / calib_samples;
	    m->PARAMS.Ib_offset = (float_t)sum_Ib / calib_samples;
	    m->PARAMS.Ic_offset = (float_t)sum_Ic / calib_samples;

	    m->STATUS.READY = true;
}
