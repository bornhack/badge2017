#include "common.h"
#include "timer.h"

#define TIMERn TIMER2
#define timern_(name, ...) timer2_##name(__VA_ARGS__)
#include "timern.c"
#undef timern_
#undef TIMERn
