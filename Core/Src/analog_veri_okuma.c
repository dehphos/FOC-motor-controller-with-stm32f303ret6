/**
 * @file    analog_veri_okuma.c
 * @brief   Faz akımı okuma ve akım sensörü ofset kalibrasyonu
 *          fonksiyonlarının uygulaması.
 */

#include "analog_veri_okuma.h"







extern TIM_HandleTypeDef htim1;


/**
 * @brief  Üç faz akımını okur (veya simüle eder), Amper cinsine ölçekler ve KCL sağlığını hesaplar.
 *
 * @details ADC'den okunan ham değerler (0-4095) kalibrasyon ofsetleri
 *          çıkarılarak Amper (A) seviyesine `map()` fonksiyonu ile dönüştürülür.
 *          Dönüşüm sonrası Kirchhoff Akım Yasası (KCL) gereği üç fazın toplamının
 *          sıfır olması beklenir. Bu ideal toplamdan sapma miktarı `shunt_akim_kaymasi`
 *          olarak kaydedilir ve tam skalaya (m->PARAMS.SHUNT_DIVIDER_RATIO) oranlanarak `%100` (Kusursuz)
 *          ile `%0` (Bozuk) arasında bir `shunt_sagligi` değeri üretilir.
 *
 * @param  m         Akımları ve diagnostik verileri güncellenecek motor yapısına işaretçi.
 * @param  simulate  `true` ise sabit orta nokta değerleri (2048) kullanılarak donanım simüle edilir.
 * @param  m->PARAMS.SHUNT_DIVIDER_RATIO     ADC tam skalasının denk geldiği maksimum akım kapasitesi [A].
 */
void Analog_Read_Currents(motor *m, bool simulate)
{
    if (simulate) {
        m->STATUS.Ia_curr = 2048.0f;
        m->STATUS.Ib_curr = 2048.0f;
        m->STATUS.Ic_curr = 2048.0f;
    } else {
        m->STATUS.Ia_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_1);
        m->STATUS.Ib_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_2);
        m->STATUS.Ic_curr = (float_t)HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_3);
    }

    float_t multiplier = (-2.0f * m->PARAMS.SHUNT_DIVIDER_RATIO) * 0.00024420024f;

    m->STATUS.Ia_curr_map = (m->STATUS.Ia_curr + (2048.0f - m->PARAMS.Ia_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;
    m->STATUS.Ib_curr_map = (m->STATUS.Ib_curr + (2048.0f - m->PARAMS.Ib_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;
    m->STATUS.Ic_curr_map = (m->STATUS.Ic_curr + (2048.0f - m->PARAMS.Ic_offset)) * multiplier + m->PARAMS.SHUNT_DIVIDER_RATIO;

    m->DIAG.shunt_akim_kaymasi = (m->STATUS.Ia_curr_map + m->STATUS.Ib_curr_map + m->STATUS.Ic_curr_map);
    m->DIAG.shunt_sagligi = (1.0f - (fabsf(m->DIAG.shunt_akim_kaymasi) / m->PARAMS.SHUNT_DIVIDER_RATIO)) * 100.0f;
}
/**
 * @brief  Akım sensörü (şönt) sıfır-akım ofsetlerini kalibre eder.
 *
 * @param  m              Ofset değerlerinin yazılacağı motor yapısına
 *                         işaretçi.
 * @param  calib_samples  Ortalaması alınacak örnek sayısı.
 *
 * @see    analog_veri_okuma.h dosyasındaki fonksiyon açıklamasına bakınız.
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
