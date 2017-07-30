#include "common.h"

/* EMU_CTRL */
static inline void
emu_em2_block(void)               { EMU->CTRL |= EMU_CTRL_EM2BLOCK; }
static inline void
emu_em2_unblock(void)             { EMU->CTRL &= ~EMU_CTRL_EM2BLOCK; }
static inline uint32_t
emu_em2_blocked(void)             { return EMU->CTRL & EMU_CTRL_EM2BLOCK; }
static inline void
emu_vreg_reduced(void)            { EMU->CTRL |= EMU_CTRL_EMVREG; }
static inline void
emu_vreg_full(void)               { EMU->CTRL &= ~EMU_CTRL_EMVREG; }

static void __noreturn __unused
emu_em4_enter(void)
{
	unsigned int i;

	/* do the EM4 handshake */
	EMU->CTRL = 0;
	for (i = 0; i < 4; i++) {
		EMU->CTRL = 0x2U << 2;
		EMU->CTRL = 0x3U << 2;
	}
	EMU->CTRL = 0x2U << 2;

	/* enter EM4 */
	__WFI();
	__builtin_unreachable();
}


/* EMU_LOCK */
static inline void
emu_lock(void)                    { EMU->LOCK = 0; }
static inline void
emu_unlock(void)                  { EMU->LOCK = 0xADE8U; }
static inline uint32_t
emu_locked(void)                  { return EMU->LOCK; }

/* EMU_AUXCTRL */
static void __unused
emu_reset_cause_clear(void)
{
	EMU->AUXCTRL = 1;
	EMU->AUXCTRL = 0;
}
