#include "common.h"
#include "timer.h"

#define TIMERn TIMER0
#define timern_(name, ...) timer0_##name(__VA_ARGS__)
#include "timern.c"
#undef timern_
#undef TIMERn
