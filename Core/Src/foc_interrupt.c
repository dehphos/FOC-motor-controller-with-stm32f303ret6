#include "foc_interrupt.h"
#include "main.h"
#include "control.h"
#include "math.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

extern void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val);

void Foc_Loop(motor *m, ADC_HandleTypeDef *hadc, float_t V_dc){


	if(m->STATUS.READY){
		if (m->STATUS.spdcnt == 10){


		  static uint32_t last_speed_tick = 0;
		  uint32_t now = HAL_GetTick();

	#if (SIMULATE_MOTOR == true)
		  	  	static uint32_t last_sim_tick = 0;
		  	    static float_t a = 0;
		        static float_t b = 0;

		        if ((now - last_sim_tick) >= (10000/sim_rpm))
		        {
		            get_sin_cos_fast(timer, &a, &b);
	//	            m->REF.RPM = ((a * salinim) + 2000);
		            timer++;
		            if(timer == 360) timer = 0;
		            last_sim_tick = now;

		            tim_last = tim;
		            tim = __HAL_TIM_GET_COUNTER(&htim3);

		            m->STOPPED = false;

		            m->STATUS.rotor_angle +=60;
		            if (m->STATUS.rotor_angle >= 360) {
		                m->STATUS.rotor_angle = 0;
		            }

		            m->last_hall_edge_tick = now;
		        }
	#endif

			  if ((now - last_speed_tick) >= m->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS)
			  		  	  {
							last_speed_tick = now;
							calculate_speed_pi(m);
			  		  	  }

	#if (SIMULATE_MOTOR)
		            float_t dt = m->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS / 1000.0f;

		            sim_rpm += (K_TORQUE * m->REF.Iq - FRICTION * sim_rpm) * dt;
		            m->STATUS.rotor_rpm = (int16_t)sim_rpm;

	#endif
		m->STATUS.spdcnt = 0;

		} else {m->STATUS.spdcnt += 1 ;};
	};

		if (!m->STATUS.ALIGNED) return;
		if (hadc->Instance == ADC1)
		{


			Analog_Read_Currents(m, SIMULATE_MOTOR, I_max);
			if (V_dc < 5.0f) {
			m->STATUS.STOPPED_FAULT = true;
			}

			// ------------------------ FOC --------------------------
					  uint32_t current_cnt;
					  uint16_t current_tim;

					  current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
					  current_tim = m->STATUS.tim;

			          // -----------------------------------------

					  if ((HAL_GetTick() - m->STATUS.last_hall_edge_tick) >= m->STATUS.STOPPED_TIMEOUT) {
						m->STATUS.STOPPED = true;
						m->STATUS.rotor_rpm = 0;
					    if(m->REF.RPM > 100){
					    m->STATUS.STOPPED_FAULT_COUNT++;}
					}

					if (m->STATUS.STOPPED) {
					    m->STATUS.rotor_angle_interp = m->STATUS.rotor_angle;
					} else {
					    m->STATUS.STOPPED_FAULT_COUNT = 0;
					    m->REF.RPM_cur = clampf(m->REF.RPM_cur, -m->REF.RPM_lim, m->REF.RPM_lim);
					    if (current_tim == 0) current_tim = 65535;

					    // 1. Oran Orantı ile İnterpolasyon
					    float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
					    if (interp_ratio > 1.0f) interp_ratio = 1.0f;

					    if (m->REF.RPM_cur >= 0.0f) {
					        m->STATUS.rotor_angle_interp = m->STATUS.rotor_angle + (uint16_t)(60.0f * interp_ratio);
					        if (m->STATUS.rotor_angle_interp >= 360) m->STATUS.rotor_angle_interp -= 360;
					    } else {
					        int16_t temp_angle = (m->STATUS.rotor_angle + 60) - (int16_t)(60.0f * interp_ratio);
					        if (temp_angle < 0) temp_angle += 360;
					        else if (temp_angle >= 360) temp_angle -= 360;
					        m->STATUS.rotor_angle_interp = (uint16_t)temp_angle;
					    }

					    // 2. Yarış Durumu Koruması (Mandal / Latch)
					    static uint16_t prev_angle_interp = 0;
					    if (m->REF.RPM_cur >= 0.0f) {
					        if ((m->STATUS.rotor_angle_interp < prev_angle_interp) &&
					            ((prev_angle_interp - m->STATUS.rotor_angle_interp) < 180)) {
					            m->STATUS.rotor_angle_interp = prev_angle_interp;
					        }
					    } else {
					        if ((m->STATUS.rotor_angle_interp > prev_angle_interp) &&
					            ((m->STATUS.rotor_angle_interp - prev_angle_interp) < 180)) {
					            m->STATUS.rotor_angle_interp = prev_angle_interp;
					        }
					    }
					    prev_angle_interp = m->STATUS.rotor_angle_interp;
					}
					  float_t sin_angle;
					  float_t cos_angle;

	//				  get_sin_cos_fast(m->STATUS.rotor_angle_interp + m->PARAMS.HALL_OFSET, &sin_angle, &cos_angle);
					  // faz avansı
					  float_t advance_angle = (m->STATUS.rotor_rpm / 7500.0f) * 15.0f;
					  get_sin_cos_fast(m->STATUS.rotor_angle_interp + m->PARAMS.HALL_OFSET + advance_angle, &sin_angle, &cos_angle);

			          float_t Ia_foc = m->STATUS.Ia_curr_map;
					  float_t Ib_foc = m->STATUS.Ib_curr_map;
					  float_t Id_raw, Iq_raw;
					  clarke_park(Ia_foc, Ib_foc, sin_angle, cos_angle, &Id_raw, &Iq_raw);

					  m->STATUS.Id_curr = (m->STATUS.Id_curr * 0.8f) + (Id_raw * 0.2f);
					  m->STATUS.Iq_curr = (m->STATUS.Iq_curr * 0.8f) + (Iq_raw * 0.2f);


					  // -------------------- FIELD WEAKENİNG ---------------------
					if(m->PARAMS.FW){
						float_t abs_rpm = fabsf(m->STATUS.rotor_rpm);
						if (abs_rpm > 5000.0f) {
						  m->REF.Id = -0.0008f * (abs_rpm - 5000.0f);
						} else {
						  m->REF.Id = 0.0f;
						}
					}
					  //--------------------------------------------------------
					  // -------------------- PI döngüsü ------------------

					  // Iq
					  m->DQ_PI_PARAMS.Iq_E = (m->REF.Iq - m->STATUS.Iq_curr);
					  m->DQ_PI_PARAMS.Iq_integral_lim = V_dc / m->DQ_PI_PARAMS.Iq_ki;
					  m->DQ_PI_PARAMS.Iq_integral += m->DQ_PI_PARAMS.Iq_E;
					  m->DQ_PI_PARAMS.Iq_integral = clampf(m->DQ_PI_PARAMS.Iq_integral, - m->DQ_PI_PARAMS.Iq_integral_lim, m->DQ_PI_PARAMS.Iq_integral_lim);

					  m->OUT.E_q = m->DQ_PI_PARAMS.Iq_kp * m->DQ_PI_PARAMS.Iq_E + m->DQ_PI_PARAMS.Iq_ki * m->DQ_PI_PARAMS.Iq_integral;

					  // Id
					  m->DQ_PI_PARAMS.Id_E = (m->REF.Id - m->STATUS.Id_curr);
					  m->DQ_PI_PARAMS.Id_integral_lim = V_dc / m->DQ_PI_PARAMS.Id_ki;
					  m->DQ_PI_PARAMS.Id_integral += m->DQ_PI_PARAMS.Id_E;
					  m->DQ_PI_PARAMS.Id_integral = clampf(m->DQ_PI_PARAMS.Id_integral, - m->DQ_PI_PARAMS.Id_integral_lim, m->DQ_PI_PARAMS.Id_integral_lim);

					  m->OUT.E_d = m->DQ_PI_PARAMS.Id_kp * m->DQ_PI_PARAMS.Id_E + m->DQ_PI_PARAMS.Id_ki * m->DQ_PI_PARAMS.Id_integral;


					  // ------------------- Feed Forward ----------------------
					  float_t Ls = 0.0000321f;
					  float_t psi_m = 0.00936f;
					  float_t omega_e = m->STATUS.rotor_rpm * (PI / 30.0f) * m->PARAMS.NUM_OF_POLE_PAIRS;
					  float_t Vd_ff = -omega_e * Ls * m->STATUS.Iq_curr;
					  float_t Vq_ff = (omega_e * Ls * m->STATUS.Id_curr) + (omega_e * psi_m);

					  m->OUT.E_d += Vd_ff;
					  m->OUT.E_q += Vq_ff;
					  //--------------------------------------------------------

					  //----------------------BARA LİMİTİ-----------------------


					  float_t V_rms = V_dc;
					  m->OUT.E_d = clampf(m->OUT.E_d, -V_rms, V_rms);
					  float_t Eq_max = sqrtf((V_rms * V_rms) - (m->OUT.E_d * m->OUT.E_d));
					  m->OUT.E_q = clampf(m->OUT.E_q, -Eq_max, Eq_max);

			          // --------------------------------------------------



					  inv_clarke_park(m->OUT.E_d, m->OUT.E_q, sin_angle, cos_angle, &m->OUT.Va, &m->OUT.Vb, &m->OUT.Vc);

			  if(SVPWM_OUT){
			  //------------------- SVPWM -------------------------

				  if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
					  m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
					  m->SPEED_PI_PARAMS.Speed_integral = 0;
					  m->DQ_PI_PARAMS.Id_integral = 0;
					  m->DQ_PI_PARAMS.Iq_integral = 0;
				  }
				  float_t V_max = m->OUT.Va;
				  float_t V_min = m->OUT.Va;

				  if (m->OUT.Vb > V_max) {V_max = m->OUT.Vb;}
				  if (m->OUT.Vc > V_max) {V_max = m->OUT.Vc;}

				  if (m->OUT.Vb < V_min) {V_min = m->OUT.Vb;}
				  if (m->OUT.Vc < V_min) {V_min = m->OUT.Vc;}

				  float_t V_com = -(V_max + V_min) / 2.0f;



				  m->SVPWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
				  m->SVPWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
				  m->SVPWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc + V_com, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, m->SVPWM.A );
				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, m->SVPWM.B );
				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, m->SVPWM.C );
	//		  //------------------- SVPWM -------------------------
			  } else {
				  if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
					  m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
					  m->SPEED_PI_PARAMS.Speed_integral = 0;
					  m->DQ_PI_PARAMS.Id_integral = 0;
					  m->DQ_PI_PARAMS.Iq_integral = 0;
				  }
				  m->PWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
				  m->PWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
				  m->PWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, m->PWM.A );
				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, m->PWM.B );
				  __HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, m->PWM.C );

			  };

		};
};

