/* TIMERn_CTRL */
static inline void
timern_(config, uint32_t v)              { TIMERn->CTRL = v; }

/* TIMERn_CMD */
static inline void
timern_(stop, void)                      { TIMERn->CMD = TIMER_CMD_STOP; }
static inline void
timern_(start, void)                     { TIMERn->CMD = TIMER_CMD_START; }

/* TIMERn_STATUS */
static inline uint32_t
timern_(status, void)                    { return TIMERn->STATUS; }
static inline uint32_t
timern_(cc2_polarity, void)              { return TIMERn->STATUS & TIMER_STATUS_CCPOL2; }
static inline uint32_t
timern_(cc1_polarity, void)              { return TIMERn->STATUS & TIMER_STATUS_CCPOL1; }
static inline uint32_t
timern_(cc0_polarity, void)              { return TIMERn->STATUS & TIMER_STATUS_CCPOL0; }
static inline uint32_t
timern_(cc2_capture_valid, void)         { return TIMERn->STATUS & TIMER_STATUS_ICV2; }
static inline uint32_t
timern_(cc1_capture_valid, void)         { return TIMERn->STATUS & TIMER_STATUS_ICV1; }
static inline uint32_t
timern_(cc0_capture_valid, void)         { return TIMERn->STATUS & TIMER_STATUS_ICV0; }
static inline uint32_t
timern_(cc2_buffer_valid, void)          { return TIMERn->STATUS & TIMER_STATUS_CCVBV2; }
static inline uint32_t
timern_(cc1_buffer_valid, void)          { return TIMERn->STATUS & TIMER_STATUS_CCVBV1; }
static inline uint32_t
timern_(cc0_buffer_valid, void)          { return TIMERn->STATUS & TIMER_STATUS_CCVBV0; }
static inline uint32_t
timern_(top_buffer_valid, void)          { return TIMERn->STATUS & TIMER_STATUS_TOPBV; }
static inline uint32_t
timern_(direction_down, void)            { return TIMERn->STATUS & TIMER_STATUS_DIR; }
static inline uint32_t
timern_(running, void)                   { return TIMERn->STATUS & TIMER_STATUS_RUNNING; }

/* TIMERn_IEN */
static inline void
timern_(flag_cc2_overflow_disable, void) { TIMERn->IEN &= ~TIMER_IEN_ICBOF2; }
static inline void
timern_(flag_cc2_overflow_enable, void)  { TIMERn->IEN |= TIMER_IEN_ICBOF2; }
static inline void
timern_(flag_cc1_overflow_disable, void) { TIMERn->IEN &= ~TIMER_IEN_ICBOF1; }
static inline void
timern_(flag_cc1_overflow_enable, void)  { TIMERn->IEN |= TIMER_IEN_ICBOF1; }
static inline void
timern_(flag_cc0_overflow_disable, void) { TIMERn->IEN &= ~TIMER_IEN_ICBOF0; }
static inline void
timern_(flag_cc0_overflow_enable, void)  { TIMERn->IEN |= TIMER_IEN_ICBOF0; }
static inline void
timern_(flag_cc2_disable, void)          { TIMERn->IEN &= ~TIMER_IEN_CC2; }
static inline void
timern_(flag_cc2_enable, void)           { TIMERn->IEN |= TIMER_IEN_CC2; }
static inline void
timern_(flag_cc1_disable, void)          { TIMERn->IEN &= ~TIMER_IEN_CC1; }
static inline void
timern_(flag_cc1_enable, void)           { TIMERn->IEN |= TIMER_IEN_CC1; }
static inline void
timern_(flag_cc0_disable, void)          { TIMERn->IEN &= ~TIMER_IEN_CC0; }
static inline void
timern_(flag_cc0_enable, void)           { TIMERn->IEN |= TIMER_IEN_CC0; }
static inline void
timern_(flag_underflow_disable, void)    { TIMERn->IEN &= ~TIMER_IEN_UF; }
static inline void
timern_(flag_underflow_enable, void)     { TIMERn->IEN |= TIMER_IEN_UF; }
static inline void
timern_(flag_overflow_disable, void)     { TIMERn->IEN &= ~TIMER_IEN_OF; }
static inline void
timern_(flag_overflow_enable, void)      { TIMERn->IEN |= TIMER_IEN_OF; }

/* TIMERn_IF */
static inline uint32_t
timern_(flags, void)                     { return TIMERn->IF; }
static inline uint32_t
timern_(flag_cc2_overflow, uint32_t v)   { return v & TIMER_IF_ICBOF2; }
static inline uint32_t
timern_(flag_cc1_overflow, uint32_t v)   { return v & TIMER_IF_ICBOF1; }
static inline uint32_t
timern_(flag_cc0_overflow, uint32_t v)   { return v & TIMER_IF_ICBOF0; }
static inline uint32_t
timern_(flag_cc2, uint32_t v)            { return v & TIMER_IF_CC2; }
static inline uint32_t
timern_(flag_cc1, uint32_t v)            { return v & TIMER_IF_CC1; }
static inline uint32_t
timern_(flag_cc0, uint32_t v)            { return v & TIMER_IF_CC0; }
static inline uint32_t
timern_(flag_underflow, uint32_t v)      { return v & TIMER_IF_UF; }
static inline uint32_t
timern_(flag_overflow, uint32_t v)       { return v & TIMER_IF_OF; }

/* TIMERn_IFS */
static inline void
timern_(flags_set, uint32_t v)           { TIMERn->IFS = v; }
static inline void
timern_(flag_cc2_overflow_set, void)     { TIMERn->IFS = TIMER_IFS_ICBOF2; }
static inline void
timern_(flag_cc1_overflow_set, void)     { TIMERn->IFS = TIMER_IFS_ICBOF1; }
static inline void
timern_(flag_cc0_overflow_set, void)     { TIMERn->IFS = TIMER_IFS_ICBOF0; }
static inline void
timern_(flag_cc2_set, void)              { TIMERn->IFS = TIMER_IFS_CC2; }
static inline void
timern_(flag_cc1_set, void)              { TIMERn->IFS = TIMER_IFS_CC1; }
static inline void
timern_(flag_cc0_set, void)              { TIMERn->IFS = TIMER_IFS_CC0; }
static inline void
timern_(flag_underflow_set, void)        { TIMERn->IFS = TIMER_IFS_UF; }
static inline void
timern_(flag_overflow_set, void)         { TIMERn->IFS = TIMER_IFS_OF; }

/* TIMERn_IFC */
static inline void
timern_(flags_clear, uint32_t v)         { TIMERn->IFC = v; }
static inline void
timern_(flag_cc2_overflow_clear, void)   { TIMERn->IFC = TIMER_IFC_ICBOF2; }
static inline void
timern_(flag_cc1_overflow_clear, void)   { TIMERn->IFC = TIMER_IFC_ICBOF1; }
static inline void
timern_(flag_cc0_overflow_clear, void)   { TIMERn->IFC = TIMER_IFC_ICBOF0; }
static inline void
timern_(flag_cc2_clear, void)            { TIMERn->IFC = TIMER_IFC_CC2; }
static inline void
timern_(flag_cc1_clear, void)            { TIMERn->IFC = TIMER_IFC_CC1; }
static inline void
timern_(flag_cc0_clear, void)            { TIMERn->IFC = TIMER_IFC_CC0; }
static inline void
timern_(flag_underflow_clear, void)      { TIMERn->IFC = TIMER_IFC_UF; }
static inline void
timern_(flag_overflow_clear, void)       { TIMERn->IFC = TIMER_IFC_OF; }

/* TIMERn_TOP */
static inline uint32_t
timern_(top, void)                        { return TIMERn->TOP; }
static inline void
timern_(top_set, uint32_t v)              { TIMERn->TOP = v; }

/* TIMERn_TOPB */
static inline uint32_t
timern_(top_buffer, void)                 { return TIMERn->TOPB; }
static inline void
timern_(top_buffer_set, uint32_t v)       { TIMERn->TOPB = v; }

/* TIMERn_CNT */
static inline uint32_t
timern_(counter, void)                    { return TIMERn->CNT; }
static inline void
timern_(counter_set, uint32_t v)          { TIMERn->CNT = v; }

/* TIMERn_ROUTE */
static inline void
timern_(pins, uint32_t v)                 { TIMERn->ROUTE = v; }
static inline void
timern_(pin_cc2_disable, void)            { TIMERn->ROUTE &= ~(TIMER_ROUTE_CC2PEN); }
static inline void
timern_(pin_cc2_enable, void)             { TIMERn->ROUTE |= TIMER_ROUTE_CC2PEN; }
static inline void
timern_(pin_cc1_disable, void)            { TIMERn->ROUTE &= ~(TIMER_ROUTE_CC1PEN); }
static inline void
timern_(pin_cc1_enable, void)             { TIMERn->ROUTE |= TIMER_ROUTE_CC1PEN; }
static inline void
timern_(pin_cc0_disable, void)            { TIMERn->ROUTE &= ~(TIMER_ROUTE_CC0PEN); }
static inline void
timern_(pin_cc0_enable, void)             { TIMERn->ROUTE |= TIMER_ROUTE_CC0PEN; }

/* TIMERn_CCx_CTRL */
static inline void
timern_(cc_config, unsigned int i, uint32_t v)
{
	TIMERn->CC[i].CTRL = v;
}

/* TIMERn_CCx_CCV */
static inline uint32_t
timern_(cc_value, unsigned int i)         { return TIMERn->CC[i].CCV; }
static inline void
timern_(cc_value_set, unsigned int i, uint32_t v)
{
	TIMERn->CC[i].CCV = v;
}

/* TIMERn_CCx_CCVP */
static inline uint32_t
timern_(cc_value_peek, unsigned int i)    { return TIMERn->CC[i].CCVP; }

/* TIMERn_CCx_CCVB */
static inline uint32_t
timern_(cc_buffer, unsigned int i)        { return TIMERn->CC[i].CCVB; }
static inline void
timern_(cc_buffer_set, unsigned int i, uint32_t v)
{
	TIMERn->CC[i].CCV = v;
}

/* TIMERn_DTCTRL */

/* TIMERn_DTTIME */

/* TIMERn_DTFC */

/* TIMERn_DTOGEN */

/* TIMERn_DTFAULT */

/* TIMERn_DTFAULTC */

/* TIMERn_DTLOCK */
