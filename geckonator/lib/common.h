#ifndef _COMMON_H
#define _COMMON_H

#include "em_device.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#endif

#ifndef __pure
#define __pure __attribute__((pure))
#endif

#ifndef __unused
#define __unused __attribute__((unused))
#endif

#ifndef __used
#define __used __attribute__((used))
#endif

#ifndef __noreturn
#define __noreturn __attribute__((noreturn))
#endif

#ifndef __uninitialized
#define __uninitialized __attribute__((section(".uninit")))
#endif

#ifndef __align
#define __align(x) __attribute__((aligned(x)))
#endif

#endif
