#include "common.h"

/* MSC_CTRL */
static inline void
flash_busfault_generate(void)       { MSC->CTRL = MSC_CTRL_BUSFAULT_GENERATE; }
static inline void
flash_busfault_ignore(void)         { MSC->CTRL = MSC_CTRL_BUSFAULT_IGNORE; }

/* MSC_READCTRL */
static inline void
flash_ramcache_disable(void)        { MSC->READCTRL &= ~MSC_READCTRL_RAMCEN; }
static inline void
flash_ramcache_enable(void)         { MSC->READCTRL |= MSC_READCTRL_RAMCEN; }
static inline void
flash_auto_invalidate_disable(void) { MSC->READCTRL |= MSC_READCTRL_AIDIS; }
static inline void
flash_auto_invalidate_enable(void)  { MSC->READCTRL &= ~MSC_READCTRL_AIDIS; }
static inline void
flash_cache_disable(void)           { MSC->READCTRL |= MSC_READCTRL_IFCDIS; }
static inline void
flash_cache_enable(void)            { MSC->READCTRL &= ~MSC_READCTRL_IFCDIS; }
static inline void
flash_read_mode_0ws(void)           { MSC->READCTRL &= ~_MSC_READCTRL_MODE_MASK; }
static inline void
flash_read_mode_1ws(void)           { MSC->READCTRL |= MSC_READCTRL_MODE_WS1; }

/* MSC_WRITECTRL */
static inline void
flash_erase_abort_on_int(void)      { MSC->WRITECTRL |= MSC_WRITECTRL_IRQERASEABORT; }
static inline void
flash_write_disable(void)           { MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN; }
static inline void
flash_write_enable(void)            { MSC->WRITECTRL |= MSC_WRITECTRL_WREN; }

/* MSC_WRITECMD */
static inline void
flash_wdata_clear(void)             { MSC->WRITECMD = MSC_WRITECMD_CLEARWDATA; }
static inline void
flash_erase_all(void)               { MSC->WRITECMD = MSC_WRITECMD_ERASEMAIN0; }
static inline void
flash_erase_abort(void)             { MSC->WRITECMD = MSC_WRITECMD_ERASEABORT; }
static inline void
flash_write_trigger(void)           { MSC->WRITECMD = MSC_WRITECMD_WRITETRIG; }
static inline void
flash_write_once(void)              { MSC->WRITECMD = MSC_WRITECMD_WRITEONCE; }
static inline void
flash_write_end(void)               { MSC->WRITECMD = MSC_WRITECMD_WRITEEND; }
static inline void
flash_erase_page(void)              { MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE; }
static inline void
flash_address_load(void)            { MSC->WRITECMD = MSC_WRITECMD_LADDRIM; }

/* MSC_ADDRB */
static inline void
flash_address_set(uint32_t v)       { MSC->ADDRB = v; }

/* MSC_WDATA */
static inline void
flash_wdata(uint32_t v)             { MSC->WDATA = v; }

/* MSC_STATUS */
static inline uint32_t
flash_pcounters_running(void)       { return MSC->STATUS & MSC_STATUS_PCRUNNING; }
static inline uint32_t
flash_erase_aborted(void)           { return MSC->STATUS & MSC_STATUS_ERASEABORTED; }
static inline uint32_t
flash_wdata_timeout(void)           { return MSC->STATUS & MSC_STATUS_WORDTIMEOUT; }
static inline uint32_t
flash_wdata_ready(void)             { return MSC->STATUS & MSC_STATUS_WDATAREADY; }
static inline uint32_t
flash_address_invalid(void)         { return MSC->STATUS & MSC_STATUS_INVADDR; }
static inline uint32_t
flash_address_locked(void)          { return MSC->STATUS & MSC_STATUS_LOCKED; }
static inline uint32_t
flash_busy(void)                    { return MSC->STATUS & MSC_STATUS_BUSY; }

/* MSC_LOCK */
static inline void
flash_lock(void)                    { MSC->LOCK = MSC_LOCK_LOCKKEY_LOCK; }
static inline void
flash_unlock(void)                  { MSC->LOCK = MSC_LOCK_LOCKKEY_UNLOCK; }
static inline uint32_t
flash_locked(void)                  { return MSC->LOCK; }

/* MSC_CMD */
static inline void
flash_pcounters_stop(void)          { MSC->CMD = MSC_CMD_STOPPC; }
static inline void
flash_pcounters_start(void)         { MSC->CMD = MSC_CMD_STARTPC; }
static inline void
flash_cache_invalidate(void)        { MSC->CMD = MSC_CMD_INVCACHE; }

/* MSC_CACHEHITS */
static inline uint32_t
flash_cache_hits(void)              { return MSC->CACHEHITS; }
