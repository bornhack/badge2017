#include "common.h"
#include "i2c.h"

#define I2Cn I2C0
#define i2cn_(name, ...) i2c0_##name(__VA_ARGS__)
#include "i2cn.c"
#undef i2cn_
#undef I2Cn
