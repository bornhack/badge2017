/*
 * This file is part of geckonator.
 * Copyright 2017 Emil Renner Berthing <esmil@esmil.dk>
 *
 * geckonator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * geckonator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with geckonator. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "lib/clock.c"
#include "lib/gpio.c"
#include "lib/rtc.c"

#define LED       GPIO_PA0
#define DELAY_MS  500

void
RTC_IRQHandler(void)
{
	gpio_toggle(LED);

	rtc_comp0_set(rtc_comp0() + DELAY_MS);
	rtc_flags_clear(rtc_flags());
}

void __noreturn
main(void)
{
	/* disable auxfrco, only needed to program flash */
	clock_auxhfrco_disable();

	/* enable gpio clock and configure LED pin */
	clock_gpio_enable();
	gpio_set(LED);
	gpio_mode(LED, GPIO_MODE_WIREDAND);

	/* enable low energy clock and 1kHz ULFRCO rtc clock */
	clock_le_enable();
	clock_lf_config(CLOCK_LFA_ULFRCO | CLOCK_LFB_DISABLED | CLOCK_LFC_DISABLED);
	clock_rtc_div1();
	clock_rtc_enable();

	/* enable comp0 interrupt and start the rtc */
	/* NVIC_SetPriority(RTC_IRQn, 3); */
	NVIC_EnableIRQ(RTC_IRQn);
	rtc_flag_comp0_enable();
	rtc_config(RTC_ENABLE);

	/* sleep when not interrupted */
	while (1) {
		__WFI();
	}
}
