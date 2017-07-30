#include "common.h"
#include "timer.h"

#define TIMERn TIMER1
#define timern_(name, ...) timer1_##name(__VA_ARGS__)
#include "timern.c"
#undef timern_
#undef TIMERn
