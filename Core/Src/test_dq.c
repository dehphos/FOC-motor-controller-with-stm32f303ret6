/**
 * @file    test_dq.c
 * @brief   D-Q akım PI regülatörü kazanç tarama (sweep) testi: sabit bir
 *          hızda (5000 RPM) çalışırken önceden tanımlı bir kazanç listesi
 *          üzerinde gezinip her kazanç seti için basamak (step) referans
 *          uygulayarak akım döngüsü tepkisini karakterize eder.
 *          Sadece `DQ_TEST` makrosu tanımlıysa derlenir.
 */

#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test_dq.h"

#if DQ_TEST
/**
 * @brief  Taranacak D-Q akım PI regülatörü kazanç setleri (kp, ki) listesi.
 *         Sırayla artan agresiflikte 20 farklı kazanç kombinasyonu içerir.
 */
PI_Test_Params pi_test_array[] = {
		    {0.0100f, 0.0020f},
		    {0.0200f, 0.0040f},
		    {0.0300f, 0.0060f},
		    {0.0400f, 0.0080f},
		    {0.0500f, 0.0100f},
		    {0.0550f, 0.0110f},
		    {0.0600f, 0.0120f},
		    {0.0642f, 0.0126f},
		    {0.0700f, 0.0150f},
		    {0.0750f, 0.0175f},
		    {0.0780f, 0.0180f},
		    {0.0800f, 0.0200f},
		    {0.0820f, 0.0210f},
		    {0.0850f, 0.0220f},
		    {0.0900f, 0.0250f},
		    {0.0950f, 0.0280f},
		    {0.1000f, 0.0300f},
		    {0.1100f, 0.0350f},
		    {0.1300f, 0.0400f},
		    {0.1500f, 0.0500f}
};

/** @brief `pi_test_array` içinde şu anda test edilen kazanç setinin indeksi. */
uint8_t current_test_index = 0;
/** @brief Tek bir kazanç seti için basamak testi alt durum makinesi (0/1/2). */
uint8_t step_test_state = 0;

/**
 * @brief  D-Q akım PI regülatörü kazançlarını sırayla tarayarak her biri
 *         için bir basamak (step) referans testi uygulayan fonksiyon.
 *
 * İşleyiş:
 *  - Motor hizalı ve arızasızsa, hız referansı sabit 5000 RPM'e ayarlanır.
 *  - Tarama henüz başlamadıysa (`*sweep_started == 0`), `system_start_tick`
 *    üzerinden 2000 ms geçtiğinde ilk kazanç seti yüklenir ve tarama
 *    başlatılır.
 *  - Tarama başladıktan sonra her kazanç seti için üç alt durumdan
 *    (`step_test_state`) geçilir:
 *      -# **0:** `REF.Iq = 0`, 500 ms bekle.
 *      -# **1:** `REF.Iq = 2.0f` (basamak uygulanır), 650 ms bekle.
 *      -# **2:** `REF.Iq = 0`, 1000 ms bekle; ardından bir sonraki kazanç
 *         setine geçilir (integral terimleri sıfırlanarak) veya liste
 *         tükendiyse tarama tamamlanır (`*sweep_done = 4`).
 *
 * @param  m                  Test edilecek motor yapısına işaretçi.
 * @param  sweep_started      Taramanın başlayıp başlamadığını tutan bayrak.
 * @param  sweep_done         Tarama tamamlandığında 4 olarak ayarlanır.
 * @param  new_tim            Kullanılmıyor (arayüz uyumluluğu için tutulan
 *                             parametre).
 * @param  sweep_last_tick    Bir önceki alt durum geçişinin zaman damgası.
 * @param  system_start_tick  Sistemin/testin başlangıç zaman damgası.
 *
 * @note   Motor hizalanmamışsa veya arıza durumundaysa hiçbir işlem
 *         yapılmaz.
 */
void test_dq_pi(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{
	static uint16_t NUM_TESTS = sizeof(pi_test_array)/sizeof(pi_test_array[0]);
	  m->REF.RPM = 5000.0f;
	  m->REF.RPM_cur = 5000.0f;
  uint32_t now = HAL_GetTick();

  if (m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
  {

      if (!*sweep_started)
      {
          if (now - system_start_tick >= 2000)
          {
              *sweep_started = 1;
              step_test_state = 0;

              m->DQ_PI_PARAMS.Iq_kp = pi_test_array[current_test_index].kp;
              m->DQ_PI_PARAMS.Iq_ki = pi_test_array[current_test_index].ki;
              m->DQ_PI_PARAMS.Id_kp = pi_test_array[current_test_index].kp;
              m->DQ_PI_PARAMS.Id_ki = pi_test_array[current_test_index].ki;

              m->DQ_PI_PARAMS.Iq_integral = 0.0f;
              m->DQ_PI_PARAMS.Id_integral = 0.0f;
              m->REF.Iq = 0.0f;

              *sweep_last_tick = now;
          }
      }
      else if (*sweep_done == 0)
      {

          if (step_test_state == 0)
          {
              m->REF.Iq = 0.0f;
              if (now - *sweep_last_tick >= 500)
              {
                  step_test_state = 1;
                  m->REF.Iq = 2.0f;
              }
          }

          else if (step_test_state == 1)
          {
              if (now - *sweep_last_tick >= 650)
              {
                  step_test_state = 2;
                  m->REF.Iq = 0.0f;
              }
          }

          else if (step_test_state == 2)
          {
              if (now - *sweep_last_tick >= 1000)
              {
                  current_test_index++;

                  if (current_test_index < NUM_TESTS)
                  {

                      m->DQ_PI_PARAMS.Iq_kp = pi_test_array[current_test_index].kp;
                      m->DQ_PI_PARAMS.Iq_ki = pi_test_array[current_test_index].ki;
                      m->DQ_PI_PARAMS.Id_kp = pi_test_array[current_test_index].kp;
                      m->DQ_PI_PARAMS.Id_ki = pi_test_array[current_test_index].ki;

                      m->DQ_PI_PARAMS.Iq_integral = 0.0f;
                      m->DQ_PI_PARAMS.Id_integral = 0.0f;

                      *sweep_last_tick = now;
                      step_test_state = 0;
                  }
                  else
                  {

                      m->REF.Iq = 0.0f;
                      *sweep_done = 4;
                  }
              }
          }
      }
  }
}
#endif
