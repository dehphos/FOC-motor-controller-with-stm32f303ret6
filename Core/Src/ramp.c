#include "ramp.h"
#include "math.h"

void ramp(motor *MOTOR) {

    if (MOTOR->REF.RPM > MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur += MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur > MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
        }
    }
    else if (MOTOR->REF.RPM < MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur -= MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur < MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
        }
    }
}
