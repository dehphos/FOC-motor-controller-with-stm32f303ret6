#include "analog_veri_okuma.h"
#include "stdbool.h"





float_t Ia_offset = 1990.0f;
float_t Ib_offset = 1999.0f;
float_t Ic_offset = 2005.0f;


static inline float_t analog_map(float_t variable, float_t min_fm, float_t max_fm, float_t min_to, float_t max_to)
{
    float_t percentage = (variable - min_fm) / (max_fm - min_fm);
    return percentage * (max_to - min_to) + min_to;

}



void Analog_Read_Currents(ADC_HandleTypeDef *hadc, bool simulate,
                          float_t *raw_Ia, float_t *raw_Ib, float_t *raw_Ic,
                          float_t *Ia_map, float_t *Ib_map, float_t *Ic_map,
                          float_t i_max)
{

    if (simulate) {

        *raw_Ia = 2048.0f;
        *raw_Ib = 2048.0f;
        *raw_Ic = 2048.0f;
    } else {


        *raw_Ia = (float_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        *raw_Ib = (float_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        *raw_Ic = (float_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    }


    *Ia_map = analog_map(*raw_Ia + (2048 - Ia_offset), 0.0f, 4095.0f, i_max, -i_max);
    *Ib_map = analog_map(*raw_Ib + (2048 - Ib_offset), 0.0f, 4095.0f, i_max, -i_max);
    *Ic_map = analog_map(*raw_Ic + (2048 - Ic_offset), 0.0f, 4095.0f, i_max, -i_max);
}

void Analog_Calibrate_Offsets(ADC_HandleTypeDef *hadc, uint16_t calib_samples, motor *MOTOR){

	    uint32_t sum_Ia = 0, sum_Ib = 0, sum_Ic = 0;

	    for(int i = 0; i < calib_samples; i++) {

	        sum_Ia += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	        sum_Ib += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	        sum_Ic += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

	        HAL_Delay(1);
	    }


	    Ia_offset = (float_t)sum_Ia / calib_samples;
	    Ib_offset = (float_t)sum_Ib / calib_samples;
	    Ic_offset = (float_t)sum_Ic / calib_samples;

	    MOTOR->READY = true;

}
