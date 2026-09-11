/**
 * @file    acildurum.c
 * @brief   Acil durum / arıza izleme ve alan zayıflatma (Field Weakening)
 *          eşiği kontrolünün uygulaması.
 */

#include "acildurum.h"



/**
 * @brief  Motorun hizalama durumunu, arıza/acil durdurma koşullarını ve
 *         alan zayıflatma gerekliliğini kontrol eder; gerekirse motoru
 *         güvenli duruma sokar.
 *
 * İşleyiş:
 *  - Motor hizalanmamışsa `Align_Motor()` çağrılır.
 *  - `STOPPED_FAULT`, aşırı `STOPPED_FAULT_COUNT` veya Hall hata sayaçları
 *    eşik değerini aşarsa: PWM çıkışları güvenli (nötr) değere ayarlanır ve
 *    hata durumu manuel olarak temizlenene kadar (Hall hataları ve
 *    `STOPPED_FAULT` sıfırlanana kadar) sonsuz döngüde beklenir; bu sırada
 *    tüm PI integral biriktiricileri ve hız referansı sıfırlanır.
 *  - Hız `MAX_WITHOUT_FW` eşiğini aşıyorsa alan zayıflatma (`PARAMS.FW`)
 *    etkinleştirilir ve dairesel gerilim sınırlaması (`CIRCULAR_LIM`)
 *    devre dışı bırakılır; aksi halde tam tersi uygulanır.
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
			// Hız (MAX_WO_FW - 200)'ün altına düşerse kapanır (Doğru Histerezis)
			else if (fabsf(m->REF.RPM_cur) < (m->PARAMS.MAX_WO_FW - 200.0f)) {
				m->PARAMS.FW = false;
			}

			// DİKKAT: BEMF Gözlemcisinin "Voltaj Yalanına" düşmemesi için
			// Dairesel Limitasyon (Circular Lim) ASLA kapatılamaz!
			m->PARAMS.CIRCULAR_LIM = true;
		} else {
			m->PARAMS.FW = false;
			m->PARAMS.CIRCULAR_LIM = true;
		}


		m->DIAG.cpu_freetime = (50-(m->DIAG.foc_time_us + m->DIAG.hall_time_us)) * 2;


}
