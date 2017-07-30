#include "common.h"

/* RTC_CTRL */
enum rtc_config {
	RTC_COMP0TOP = RTC_CTRL_COMP0TOP,
	RTC_DEBUGRUN = RTC_CTRL_DEBUGRUN,
	RTC_ENABLE   = RTC_CTRL_EN,
};

static inline void
rtc_config(uint32_t v)          { RTC->CTRL = v; }
static inline void
rtc_comp0top_disable(void)      { RTC->CTRL &= ~RTC_CTRL_COMP0TOP; }
static inline void
rtc_comp0top_enable(void)       { RTC->CTRL |= RTC_CTRL_COMP0TOP; }
static inline void
rtc_debugrun_disable(void)      { RTC->CTRL &= ~RTC_CTRL_DEBUGRUN; }
static inline void
rtc_debugrun_enable(void)       { RTC->CTRL |= RTC_CTRL_DEBUGRUN; }
static inline void
rtc_disable(void)               { RTC->CTRL &= ~RTC_CTRL_EN; }
static inline void
rtc_enable(void)                { RTC->CTRL |= RTC_CTRL_EN; }

/* RTC_CNT */
static inline uint32_t
rtc_counter(void)               { return RTC->CNT; }

/* RTC_COMP0 */
static inline uint32_t
rtc_comp0(void)                 { return RTC->COMP0; }
static inline void
rtc_comp0_set(uint32_t v)       { RTC->COMP0 = v; }

/* RTC_COMP1 */
static inline uint32_t
rtc_comp1(void)                 { return RTC->COMP1; }
static inline void
rtc_comp1_set(uint32_t v)       { RTC->COMP1 = v; }

/* RTC_IF */
static inline uint32_t
rtc_flags(void)                 { return RTC->IF; }
static inline uint32_t
rtc_flag_comp1(uint32_t v)      { return v & RTC_IF_COMP1; }
static inline uint32_t
rtc_flag_comp0(uint32_t v)      { return v & RTC_IF_COMP0; }
static inline uint32_t
rtc_flag_overflow(uint32_t v)   { return v & RTC_IF_OF; }

/* RTC_IFS */
static inline void
rtc_flags_set(uint32_t v)       { RTC->IFC = v; }
static inline void
rtc_flag_comp1_set(void)        { RTC->IFS = RTC_IFS_COMP1; }
static inline void
rtc_flag_comp0_set(void)        { RTC->IFS = RTC_IFS_COMP0; }
static inline void
rtc_flag_overflow_set(void)     { RTC->IFS = RTC_IFS_OF; }

/* RTC_IFC */
static inline void
rtc_flags_clear(uint32_t v)     { RTC->IFC = v; }
static inline void
rtc_flag_comp1_clear(void)      { RTC->IFC = RTC_IFC_COMP1; }
static inline void
rtc_flag_comp0_clear(void)      { RTC->IFC = RTC_IFC_COMP0; }
static inline void
rtc_flag_overflow_clear(void)   { RTC->IFC = RTC_IFC_OF; }

/* RTC_IEN */
static inline uint32_t
rtc_flag_comp1_enabled(void)    { return RTC->IEN & RTC_IEN_COMP1; }
static inline void
rtc_flag_comp1_disable(void)    { RTC->IEN &= ~RTC_IEN_COMP1; }
static inline void
rtc_flag_comp1_enable(void)     { RTC->IEN |= RTC_IEN_COMP1; }
static inline uint32_t
rtc_flag_comp0_enabled(void)    { return RTC->IEN & RTC_IEN_COMP0; }
static inline void
rtc_flag_comp0_disable(void)    { RTC->IEN &= ~RTC_IEN_COMP0; }
static inline void
rtc_flag_comp0_enable(void)     { RTC->IEN |= RTC_IEN_COMP0; }
static inline uint32_t
rtc_flag_overflow_enabled(void) { return RTC->IEN & RTC_IEN_OF; }
static inline void
rtc_flag_overflow_disable(void) { RTC->IEN &= ~RTC_IEN_OF; }
static inline void
rtc_flag_overflow_enable(void)  { RTC->IEN |= RTC_IEN_OF; }

/* RTC_FREEZE */
static inline void
rtc_freeze(void)                { RTC->FREEZE = RTC_FREEZE_REGFREEZE_FREEZE; }
static inline void
rtc_update(void)                { RTC->FREEZE = RTC_FREEZE_REGFREEZE_UPDATE; }

/* RTC_SYNCBUSY */
static inline uint32_t
rtc_syncbusy(void)              { return RTC->SYNCBUSY; }
