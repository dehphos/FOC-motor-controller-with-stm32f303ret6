/**
 * @file    acildurum.c
 * @brief   Acil durum / arıza izleme ve alan zayıflatma (Field Weakening)
 *          eşiği kontrolünün uygulaması.
 */

#include "acildurum.h"



/**
 * @brief  Motorun hizalama durumunu, arıza/acil durdurma koşullarını ve
 *         alan zayıflatma gerekliliğini denetler; gerekirse motoru güvenli
 *         duruma sokar.
 *
 * @details İşleyiş sırasıyla şu adımlardan oluşur:
 *          1. Motor `ALIGNED` değilse `Align_Motor()` çağrılır.
 *          2. `STOPPED_FAULT`, `STOPPED_FAULT_COUNT` 100000'i aşması veya
 *             Hall hata sayaçlarından (`HALL_ERROR_0`/`HALL_ERROR_7`)
 *             herhangi biri sıfırdan büyükse: PWM çıkışları 900 (nötr)
 *             değerine ayarlanır ve `STOPPED_FAULT` ile Hall hata
 *             sayaçlarının tümü sıfırlanana kadar `while(1)` içinde
 *             `HAL_Delay(100)` ile beklenir. Koşul sağlandığında
 *             `Align_Motor()` yeniden çağrılır; `STOPPED_FAULT_COUNT`,
 *             `RPM_cur`, hız/akım PI integral biriktiricileri ve hata
 *             terimleri sıfırlanır, `STOPPED_FAULT` `false` yapılır ve
 *             döngüden çıkılır.
 *          3. `PARAMS.FW_main` izni açıksa: hız `MAX_WO_FW` eşiğini
 *             aşınca `PARAMS.FW` etkinleştirilir; hız `(MAX_WO_FW - 200)`
 *             değerinin altına düşünce (histerezisli olarak) kapatılır.
 *             `FW_main` kapalıysa `PARAMS.FW` doğrudan `false` yapılır.
 *          4. CPU boş zamanı (`cpu_freetime`), redüktör sonrası çıkış hızı
 *             (`kama_rpm`), üç fazın toplamından hesaplanan şönt akım
 *             kayması (`shunt_akim_kaymasi`) ve buna bağlı şönt ölçüm
 *             sağlığı yüzdesi (`shunt_sagligi`) güncellenir.
 *
 * @param  m  Kontrol edilecek motor yapısına işaretçi.
 *
 * @warning Arıza durumunda içerdiği `while(1)` + `HAL_Delay(100)` döngüsü
 *          bloklayıcıdır (blocking); bu fonksiyon zaman kritik bir
 *          interrupt içinden çağrılmamalıdır.
 */
void acildurum(motor *m){


		if(!m->STATUS.ALIGNED){
		  Align_Motor(m);
		};


		if(m->STATUS.STOPPED_FAULT || m->STATUS.STOPPED_FAULT_COUNT > 100000 || m->STATUS.HALL_ERROR_0 > 0 || m->STATUS.HALL_ERROR_7 > 0)
		{
			m->STATUS.STOPPED_FAULT = true;
			m->STATUS.ALIGNED = false;

			__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.A, 900);
			__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.B, 900);
			__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.C, 900);
			while(1) {
				if((!m->STATUS.STOPPED_FAULT && m->STATUS.HALL_ERROR_0 == 0 && m->STATUS.HALL_ERROR_7 == 0)){
					Align_Motor(m);
					m->STATUS.STOPPED_FAULT_COUNT = 0;
					m->REF.RPM_cur = 0;
					m->PARAMS.SPEED_PI.Speed_integral = 0;
					m->PARAMS.DQ_PI.Id_integral = 0;
					m->PARAMS.DQ_PI.Iq_integral = 0;
					m->PARAMS.SPEED_PI.E = 0;
					m->STATUS.HALL_ERROR_0 = 0;
					m->STATUS.HALL_ERROR_7 = 0;
					m->STATUS.STOPPED_FAULT = false;
					break;
				}
				HAL_Delay(100);
			}

		}



		if(m->PARAMS.FW_main){
			// Hız MAX_WO_FW'yi geçerse Field Weakening (Alan Zayıflatma) açılır
			if (fabsf(m->REF.RPM_cur) > m->PARAMS.MAX_WO_FW) {
				m->PARAMS.FW = true;
			}
			// Hız (MAX_WO_FW - 200)'ün altına düşerse kapanır (Histerezis)
			else if (fabsf(m->REF.RPM_cur) < (m->PARAMS.MAX_WO_FW - 200.0f)) {
				m->PARAMS.FW = false;
			}

		} else {
			m->PARAMS.FW = false;
		}


		m->DIAG.cpu_freetime = (50-(m->DIAG.foc_time_us + m->DIAG.hall_time_us)) * 2;
	    m->STATUS.kama_rpm = m->STATUS.rotor_rpm * 0.2222222f;
	    m->DIAG.shunt_akim_kaymasi = (m->STATUS.Ia_curr_map + m->STATUS.Ib_curr_map + m->STATUS.Ic_curr_map);
	    m->DIAG.shunt_sagligi = (1.0f - (fabsf(m->DIAG.shunt_akim_kaymasi) / m->PARAMS.SHUNT_DIVIDER_RATIO)) * 100.0f;
}
