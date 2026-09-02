/**
 * @file    test_spd.c
 * @brief   Hız döngüsü PI regülatörü kazanç tarama (sweep) testi: hız
 *          referansını 0 → `MAX_RPM` → 0 → `-MAX_RPM` → 0 arasında tarayıp,
 *          her tam çevrim sonunda listedeki bir sonraki hız PI kazanç
 *          setine geçer. Sadece `SPEED_TEST` makrosu tanımlıysa derlenir.
 */

#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test_spd.h"


#if SPEED_TEST
/**
 * @brief  Taranacak hız döngüsü PI regülatörü kazanç setleri (kp, ki)
 *         listesi. Yumuşak/hantal ayarlardan agresif/sınır zorlayan
 *         ayarlara doğru gruplandırılmıştır (bkz. dosya içi Türkçe
 *         yorumlar).
 */
PI_Test_Params pi_test_array[] = {
    // GRUP 1: Hantal ve Yumuşak (Overshoot hiç olmamalı ama hedefe geç ulaşmalı)
    {0.0005f, 0.00001f},
    {0.0005f, 0.00005f},

    // GRUP 2: Mevcut Referans Noktası ve Çevresi
    {0.0010f, 0.00002f},
    {0.0010f, 0.00005f}, // Senin şu anki ayarın
    {0.0010f, 0.00010f}, // İntegrali biraz hızlandıralım

    // GRUP 3: Agresif (Hedefe zımba gibi oturmalı, hafif overshoot olabilir)
    {0.0020f, 0.00005f},
    {0.0020f, 0.00010f},
    {0.0030f, 0.00010f},

    // GRUP 4: Çok Agresif / Sınırları Zorlayan (Motor titreyebilir veya rezonansa girebilir)
    {0.0050f, 0.00020f},
    {0.0080f, 0.00050f}
};
/** @brief `pi_test_array` içinde şu anda test edilen kazanç setinin indeksi. */
uint8_t current_test_index = 0;

/**
 * @brief  Hız PI regülatörü kazançlarını sırayla tarayarak her biri için
 *         tam bir hız tarama çevrimi (0 → +MAX_RPM → -MAX_RPM → 0)
 *         uygulayan test fonksiyonu.
 *
 * Durum makinesi (`*sweep_done` üzerinden):
 *  - **State 0:** `system_start_tick`'ten 5000 ms sonra başlar; her 20 ms'de
 *    bir `REF.RPM` 100 RPM artırılarak `MAX_RPM`'e çıkarılır.
 *  - **State 1:** Her 1000 ms'de bir `REF.RPM`, `MAX_RPM/10` kadar
 *    azaltılarak `-MAX_RPM`'e indirilir.
 *  - **State 2:** Her 20 ms'de bir `REF.RPM` 100 RPM artırılarak tekrar
 *    0'a çekilir; ardından listede bir sonraki kazanç seti varsa yüklenir
 *    ve tarama state 0'dan tekrar başlar (`Speed_integral` sıfırlanır),
 *    liste tükendiyse test tamamlanmış sayılır (`*sweep_done = 4`).
 *
 * @param  m                  Test edilecek motor yapısına işaretçi.
 * @param  sweep_started      Taramanın başlayıp başlamadığını tutan bayrak.
 * @param  sweep_done         Tarama durum makinesinin aşaması
 *                             (0/1/2 = devam ediyor, 4 = tamamlandı).
 * @param  new_tim            Kullanılmıyor (arayüz uyumluluğu için tutulan
 *                             parametre).
 * @param  sweep_last_tick    Bir önceki durum geçişinin zaman damgası.
 * @param  system_start_tick  Sistemin/testin başlangıç zaman damgası.
 *
 * @note   Her bir durum bloğu yalnızca motor hizalıyken ve arıza yokken
 *         (`ALIGNED && !STOPPED_FAULT`) işlenir.
 */
void test_speed_pi(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{

	static uint16_t NUM_TESTS = sizeof(pi_test_array)/sizeof(pi_test_array[0]);

	  if (*sweep_done == 0 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	        {

	            if (!*sweep_started)
	            {
	                if (HAL_GetTick() - system_start_tick >= 5000)
	                {
	                    *sweep_started = 1;
	                    m->REF.RPM = 0.0f;
	                    *sweep_last_tick = HAL_GetTick();
	                }
	            }

	            else
	            {

	                if (HAL_GetTick() - *sweep_last_tick >= 20)
	                {
	                    *sweep_last_tick = HAL_GetTick();

	                    if (m->REF.RPM < m->PARAMS.MAX_RPM)
	                    {
	                        m->REF.RPM += 100.0f;
	                    }
	                    else
	                    {
	                        *sweep_done = 1;
	                    }
	                }
	            }
	        }
	  if (*sweep_done == 1 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	        {
	                if (HAL_GetTick() - *sweep_last_tick >= 1000)
	                {
	                    *sweep_last_tick = HAL_GetTick();

	                    if (m->REF.RPM > -m->PARAMS.MAX_RPM)
	                    {
	                        m->REF.RPM -= m->PARAMS.MAX_RPM/10;
	                    }
	                    else
	                    {

	                        *sweep_done = 2;
	                    }
	                }
	            }
	  if (*sweep_done == 2 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	 	        {

	 	                if (HAL_GetTick() - *sweep_last_tick >= 20)
	 	                {
	 	                    *sweep_last_tick = HAL_GetTick();

	 	                    if (m->REF.RPM < 0.0f)
	 	                    {
	 	                        m->REF.RPM += 100.0f;
	 	                    }
	 	                    else
	 	                    {
	 	                    	if(current_test_index < NUM_TESTS){
	 	                        m->REF.RPM = 0.0f;
	 	                        *sweep_done = 0;
	 	                        m->SPEED_PI_PARAMS.kp = pi_test_array[current_test_index].kp;
	 	                        m->SPEED_PI_PARAMS.ki = pi_test_array[current_test_index].ki;
	 	                        m->SPEED_PI_PARAMS.Speed_integral = 0.0f;
	 	                        current_test_index++;
	 	                    	}else{
		 	                        m->REF.RPM = 0.0f;
		 	                        *sweep_done = 4;
	 	                    	}
	 	                    }
	 	                }
	 	            }
}
#endif
