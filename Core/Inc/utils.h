#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

int16_t clip(int16_t x, int16_t min, int16_t max);
int32_t min3(int32_t x, int32_t y, int32_t z);
int32_t max3(int32_t x, int32_t y, int32_t z);
int16_t abs16(int16_t val);
int32_t abs32(int32_t val);
int16_t pad14(int32_t val);

#endif // GLOBAL_H