#ifndef CLAMPF_H
#define CLAMPF_H

#include "main.h"
#include "math.h"
#include "control.h"

static inline float_t clampf(float_t x, float_t lo, float_t hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}


#endif /* CLAMPF_H */
