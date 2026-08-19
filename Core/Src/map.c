#include "map.h"
#include "math.h"

float_t map(float_t variable, float_t min_fm, float_t max_fm, float_t min_to, float_t max_to)
{
		  return ((variable - min_fm)/(max_fm - min_fm)*(max_to - min_to) + min_to);
}
