#include "analog_veri_okuma.h"
#include "stdbool.h"
#include "map.h"





float_t Ia_offset = 1990.0f;
float_t Ib_offset = 1999.0f;
float_t Ic_offset = 2005.0f;



void Analog_Read_Currents(motor *m, bool simulate, float_t i_max)
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


    m->STATUS.Ia_curr_map = map(m->STATUS.Ia_curr + (2048 - Ia_offset), 0.0f, 4095.0f, i_max, -i_max);
    m->STATUS.Ib_curr_map = map(m->STATUS.Ib_curr + (2048 - Ib_offset), 0.0f, 4095.0f, i_max, -i_max);
    m->STATUS.Ic_curr_map = map(m->STATUS.Ic_curr + (2048 - Ic_offset), 0.0f, 4095.0f, i_max, -i_max);
}

void Analog_Calibrate_Offsets(motor *m, uint16_t calib_samples){

	    uint32_t sum_Ia = 0, sum_Ib = 0, sum_Ic = 0;

	    for(int i = 0; i < calib_samples; i++) {

	        sum_Ia += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_1);
	        sum_Ib += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_2);
	        sum_Ic += HAL_ADCEx_InjectedGetValue(m->IN.SHUNT_CH, ADC_INJECTED_RANK_3);

	        HAL_Delay(1);
	    }


	    Ia_offset = (float_t)sum_Ia / calib_samples;
	    Ib_offset = (float_t)sum_Ib / calib_samples;
	    Ic_offset = (float_t)sum_Ic / calib_samples;

	    m->STATUS.READY = true;

}
