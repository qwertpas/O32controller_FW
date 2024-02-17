#include "utils.h" 


// int16_t clip16(int16_t x, int16_t min, int16_t max) {
//     return (x > max ? (max) : (x < min ? min : x));
// }

int32_t clip(int32_t x, int32_t min, int32_t max) {
    return (x > max ? (max) : (x < min ? min : x));
}

int32_t min3(int32_t x, int32_t y, int32_t z){
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

int32_t max3(int32_t x, int32_t y, int32_t z){
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

int16_t abs16(int16_t val) {
    return val<0 ? -val : val;
}

int32_t abs32(int32_t val) {
    if(val < 0) return -val;
    else return val;
}

int16_t pad14(int32_t val) { //sign extend to 16 bits
    return (val & 0x2000) ? (val | 0xC000) : val;
}

