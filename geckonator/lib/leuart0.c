#include "common.h"
#include "leuart.h"

#define LEUARTn LEUART0
#define leuartn_(name, ...) leuart0_##name(__VA_ARGS__)
#include "leuartn.c"
#undef leuartn_
#undef LEUARTn
