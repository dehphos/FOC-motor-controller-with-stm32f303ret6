/**
 * @file    acildurum.h
 * @brief   Acil durum / arıza kontrol mantığı ve alan zayıflatma (Field
 *          Weakening) eşiğinin belirlenmesi için arayüz.
 */

#ifndef ACILDURUM_H
#define ACILDURUM_H

#include "main.h"
#include "math.h"
#include "control.h"

/** @brief Bu RPM değerinin üzerinde alan zayıflatma (FW) devreye girer. */
#define MAX_WITHOUT_FW 8500

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
void acildurum(motor *m);

#endif /* ACILDURUM_H */
