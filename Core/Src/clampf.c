#include "ramp.h"
#include "math.h"

float_t clampf(float_t x, float_t lo, float_t hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}


