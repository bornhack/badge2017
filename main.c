/*
 * This file is part of Bornhack Badge 2017.
 * Copyright 2017 Emil Renner Berthing <esmil@labitat.dk>
 *
 * Bornhack Badge 2017 is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Bornhack Badge 2017 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Bornhack Badge 2017. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "lib/clock.c"
#include "lib/emu.c"
#include "lib/gpio.c"
#include "lib/rtc.c"
#include "lib/timer2.c"
#include "lib/i2c0.c"

#include "font8x8.c"

static struct {
	const uint8_t *data;
	size_t len;
	volatile int ret;
} i2c0;

typedef struct pendulum {
	double fx, fy; /*position of fix*/
  double ux, uy; /*position of upper ball*/
  double lx, ly; /*position of lower ball*/
	double vux, vuy; /*velocity of upper ball*/
	double vlx, vly; /*velocity of lower ball*/
	double aux, auy; /*acceleration of upper ball*/
	double alx, aly; /*acceleration of lower ball*/
	double deltax,deltay; /*acceleration deltas for debugging purposes*/
	double distlu; /*distance between the lower and the upper ball*/
	double distuf; /*distance between the upper ball and fix*/
	double maxdistlu;
	double maxdistuf;

	double g; /*grativity*/
} pendulum;
double norm(double x, double y){
	return sqrt(x*x+y*y);
}
double unitvecx(double x, double y){
	return x/norm(x,y);
}
double unitvecy(double x, double y){
	return y/norm(x,y);
}
double normvecx(double x1,double y1,double x2,double y2){
	if (unitvecy(x2-x1,y2-y1)<0){
		return -unitvecy(x2-x1,y2-y1);
	}else{
		return unitvecy(x2-x1,y2-y1);
	}
}
double normvecy(double x1,double y1,double x2,double y2){
	if (unitvecy(x2-x1,y2-y1)<0){
		return unitvecx(x2-x1,y2-y1);
	}else{
		return -unitvecx(x2-x1,y2-y1);
	}
}

void
I2C0_IRQHandler(void)
{
	uint32_t flags = i2c0_flags();

	i2c0_flags_clear(flags);

	if (i2c0_flag_nack(flags)) {
		i2c0_flag_tx_buffer_level_disable();
		i2c0_stop();
		i2c0_clear_tx();
		i2c0.len = 1;
		return;
	}

	if (i2c0_flag_master_stop(flags)) {
		i2c0.ret = -i2c0.len;
		return;
	}

	if (i2c0_flag_tx_buffer_level(flags)) {
		size_t len = i2c0.len;

		if (len > 0) {
			i2c0_txdata(*i2c0.data++);
			i2c0.len = len - 1;
		} else {
			i2c0_flag_tx_buffer_level_disable();
			i2c0_stop();
		}
	}
}

static int
i2c0_write(const uint8_t *ptr, size_t len)
{
	int ret;

	i2c0.data = ptr;
	i2c0.len = len;
	i2c0.ret = 1;
	emu_em2_block();
	i2c0_flag_tx_buffer_level_enable();
	i2c0_start();
	while ((ret = i2c0.ret) > 0)
		__WFI();
	emu_em2_unblock();
	return ret;
}

struct display {
	uint8_t tx;
	uint8_t ty;
	/* reset holds the pre-ample to send the display
	 * controller before sending the databits in the
	 * frame buffer. must be placed just before
	 * the framebuffer, so we can just send all bytes
	 * starting from reset and continuing into
	 * the frame buffer. */
	uint8_t reset[8];
	uint8_t framebuf[128 * 64 / 8];
};

static struct display dp;

static const uint8_t display_init_data[] = {
	0x78, /* I2C address, write                                   */
	0x00, /* Co = 0, D/C = 0: the rest is command bytes           */
	0xAE, /* display off                                          */
	0xD5, /* set display clock divide ratio / osc freq ..         */
	0x80, /*   Osc Freq = 8, DCLK = 0                             */
	0xA8, /* set multiplex ratio ..                               */
	0x3F, /*   64MUX                                              */
	0xD3, /* set display offset ..                                */
	0x00, /*   0                                                  */
	0x40, /* set display start line = 0                           */
	0x8D, /* enable change pump regulator ..                      */
	0x14, /*   (only mentioned in next to last page of datasheet) */
	0x20, /* set memory addressing mode ..                        */
	0x01, /*   0b01 = vertical addressing mode                    */
	0xA1, /* set segment re-map = 0                               */
	0xC8, /* set COM output scan direction = 1: from N-1 to 0     */
	0xDA, /* set COM pins hardware configuration ..               */
	0x12, /*   sequential, disable left/right remap               */
	0x81, /* set contrast control                                 */
	0x01, /*   0x01 (reset = 0x7F)                                */
	0xD9, /* set pre-charge period ..                             */
	0xF1, /*   phase 2 = 15 (reset 2), phase 1 = 1 (reset 2)      */
	0xDB, /* set Vcom deselect level ..                           */
	0x40, /*   0b100                                              */
	0x2E, /* deactivate scroll                                    */
	0xA4, /* entire display on: resume to ram content display     */
	0xA6, /* set normal/inverse display: normal                   */
	0x10, /* set higher column start address = 0                  */
	0x00, /* set lower column start address = 0                   */
	0xB0, /* set page address = 0                                 */
};

static int
display_init(struct display *dp)
{
	memset(dp, 0, sizeof(struct display));
	dp->reset[0] = 0x78; /* address                             */
	dp->reset[1] = 0x80; /* next byte is control                */
	dp->reset[2] = 0x00; /* set higher column start address = 0 */
	dp->reset[3] = 0x80; /* next byte is control                */
	dp->reset[4] = 0x10; /* set lower column start address = 0; */
	dp->reset[5] = 0x80; /* next byte is control                */
	dp->reset[6] = 0xB0, /* set page address = 0                */
	dp->reset[7] = 0x40; /* the remaining bytes are data bytes  */
	return i2c0_write(display_init_data, sizeof(display_init_data));
}

static int __unused
display_off(struct display *dp)
{
	static const uint8_t data[] = {
		0x78, /* I2C address, write                                   */
		0x00, /* Co = 0, D/C = 0: the rest is command bytes           */
		0xAE, /* display off                                          */
	};
	return i2c0_write(data, sizeof(data));
};

static int __unused
display_on(struct display *dp)
{
	static const uint8_t data[] = {
		0x78, /* I2C address, write                                   */
		0x00, /* Co = 0, D/C = 0: the rest is command bytes           */
		0xAF, /* display on                                           */
	};
	return i2c0_write(data, sizeof(data));
};

static int __unused
display_contrast(struct display *dp, uint8_t val)
{
	uint8_t data[4];

	data[0] = 0x78; /* I2C address, write                         */
	data[1] = 0x00; /* Co = 0, D/C = 0: the rest is command bytes */
	data[2] = 0x81; /* set contrast                               */
	data[3] = val;

	return i2c0_write(data, sizeof(data));
};

static int
display_update(struct display *dp)
{
	return i2c0_write(dp->reset, sizeof(dp->reset) + sizeof(dp->framebuf));
}

static void __unused
display_clear(struct display *dp)
{
	dp->tx = 0;
	dp->ty = 0;
	memset(dp->framebuf, 0, sizeof(dp->framebuf));
}

/* set a single pixel in the frame buffer */
static void __unused
display_set(struct display *dp, unsigned int x, unsigned int y)
{
	unsigned int idx = 8*x + y/8;
	uint8_t mask = 1 << (y & 0x7U);

	dp->framebuf[idx] |= mask;
}

static void __unused
display_text_location(struct display *dp, uint8_t x, uint8_t y)
{
	dp->tx = x;
	dp->ty = y;
}

static void __unused
display_write(struct display *dp, const uint8_t *ptr, size_t len)
{
	for (; len; len--) {
		const uint8_t *glyph;
		const uint8_t *glyph_end;
		uint8_t *dest;
		unsigned int c = *ptr++;

		if (c == '\r') {
			dp->tx = 0;
			continue;
		}
		if (c == '\n') {
			dp->tx = 0;
			goto inc_ty;
		}

		if (c < 32 || c > 0x7F)
			c = 0x7F;

		c -= 32;

		glyph = &font8x8[c][0];
		glyph_end = glyph + 8;
		dest = &dp->framebuf[dp->ty + 8*8*dp->tx];
		while (glyph < glyph_end) {
			*dest |= *glyph++;
			dest += 8;
		}

		dp->tx++;
		if (dp->tx == 16) {
			dp->tx = 0;
inc_ty:
			dp->ty++;
			if (dp->ty == 8)
				dp->ty = 0;
		}
	}
}

static void __unused
display_puts(struct display *dp, const char *str)
{
	display_write(dp, (const uint8_t *)str, strlen(str));
}

/* this function is called by newlib's stdio implementation
 * (eg. printf) and must be public */
ssize_t
_write(int fd, const uint8_t *ptr, size_t len)
{
	if (fd == 1)
		display_write(&dp, ptr, len);

	return len;
}

/* the rtc is a 24bit counter */
#define RTC_MASK 0xFFFFFFU

static inline uint32_t
rtc_lessthan(uint32_t a, uint32_t b)
{
	return (a - b) & (1U << 23);
}

struct plist_node {
	struct plist_node *next;
	uint32_t deadline;
	void (*fn)(struct plist_node *n);
};

static struct plist_node *plist_head;

static void
plist_add(struct plist_node *m, uint32_t ms)
{
	uint32_t deadline = (rtc_counter() + ms) & RTC_MASK;
	struct plist_node **n;

	m->deadline = deadline;

	__disable_irq();
	for (n = &plist_head; *n; n = &(*n)->next) {
		if (!rtc_lessthan((*n)->deadline, deadline))
			break;
	}
	m->next = *n;
	*n = m;
	__enable_irq();
	rtc_flag_comp0_set();
	rtc_flag_comp0_enable();
}

void
RTC_IRQHandler(void)
{
	struct plist_node *n;
	uint32_t now;
	uint32_t later;

	now = rtc_counter();
restart:
	while ((n = plist_head)) {
		if (rtc_lessthan(now, n->deadline))
			break;

		plist_head = n->next;
		n->fn(n);
	}

	rtc_flag_comp0_clear();

	if (plist_head == NULL) {
		rtc_flag_comp0_disable();
		return;
	}

	rtc_comp0_set(plist_head->deadline);
	later = rtc_counter();
	if (now != later) {
		now = later;
		goto restart;
	}
}

struct msleep_data {
	struct plist_node n;
	volatile bool done;
};

static void
msleep_callback(struct plist_node *n)
{
	struct msleep_data *d = (struct msleep_data *)n;

	d->done = true;
}

static void __unused
msleep(unsigned int msecs)
{
	struct msleep_data d = {
		.n.fn = msleep_callback,
		.done = false,
	};

	plist_add(&d.n, msecs);
	while (!d.done)
		__WFI();
}

enum event {
	EVENT_FIRST,
	EVENT_LAST,
	EVENT_BUTTON_A_DOWN,
	EVENT_BUTTON_A_UP,
	EVENT_BUTTON_B_DOWN,
	EVENT_BUTTON_B_UP,
	EVENT_BUTTON_X_DOWN,
	EVENT_BUTTON_X_UP,
	EVENT_BUTTON_Y_DOWN,
	EVENT_BUTTON_Y_UP,
	EVENT_BUTTON_POWER_DOWN,
	EVENT_BUTTON_POWER_UP,
	EVENT_TICK500,
};

static struct {
	volatile uint8_t first;
	volatile uint8_t last;
	bool empty;
	uint8_t queue[8];
} events;

static void
event_push(enum event ev)
{
	unsigned int first = events.first;
	unsigned int last = events.last;

	while (first != last) {
		if (events.queue[first++] == ev)
			return;
		first %= ARRAY_SIZE(events.queue);
	}

	events.queue[last++] = ev;
	events.last = last % ARRAY_SIZE(events.queue);
}

static enum event
event_pop(void)
{
	unsigned int first = events.first;
	enum event ev;

	if (first == events.last && !events.empty) {
		events.empty = true;
		return EVENT_LAST;
	}

	while (first == events.last)
		__WFI();

	if (events.empty) {
		events.empty = false;
		return EVENT_FIRST;
	}

	ev = events.queue[first++];
	events.first = first % ARRAY_SIZE(events.queue);

	return ev;
}

struct ticker_data {
	struct plist_node n;
	enum event event;
	unsigned int delay;
};

static void
ticker_callback(struct plist_node *n)
{
	struct ticker_data *d = (struct ticker_data *)n;

	if (d->delay > 0) {
		event_push(d->event);
		plist_add(n, d->delay);
	}
}

static void
ticker_run(struct ticker_data *d, enum event event, unsigned int delay)
{
	d->n.fn = ticker_callback;
	d->event = event;
	d->delay = delay;
	plist_add(&d->n, delay);
}

static void __unused
ticker_stop(struct ticker_data *d)
{
	d->delay = 0;
}

struct button {
	struct plist_node n;
	gpio_pin_t pin;
	uint8_t event;
	uint8_t repeat;
	uint16_t delay;
	uint16_t delay_left;
};

static void
button_callback(struct plist_node *n)
{
	struct button *b = (struct button *)n;

	if (b->delay_left > 50)
		b->delay_left -= 50;
	else
		b->delay_left = 0;

	if (gpio_in(b->pin)) {
		event_push(b->event+1);
		gpio_flag_clear(b->pin);
		gpio_flag_enable(b->pin);
	} else if (b->delay_left > 0) {
		if (b->delay_left > 50)
			plist_add(&b->n, 50);
		else
			plist_add(&b->n, b->delay_left);
	} else if (b->repeat > 0) {
		event_push(b->event);
		plist_add(&b->n, b->repeat);
	} else
		plist_add(&b->n, 50);
}

static void
button_firstclick(struct button *b)
{
	gpio_flag_disable(b->pin);
	event_push(b->event);
	b->delay_left = b->delay;
	if (b->delay_left > 50)
		plist_add(&b->n, 50);
	else
		plist_add(&b->n, b->delay_left);
}

static struct button buttons[] = {
	{ .n.fn = button_callback, .pin = GPIO_PF2,  .event = EVENT_BUTTON_A_DOWN,
		.delay = 350, .repeat = 80 },
	{ .n.fn = button_callback, .pin = GPIO_PF3,  .event = EVENT_BUTTON_B_DOWN,
		.delay = 350, .repeat = 80 },
	{ .n.fn = button_callback, .pin = GPIO_PE11, .event = EVENT_BUTTON_X_DOWN,
		.delay = 350, .repeat = 80 },
	{ .n.fn = button_callback, .pin = GPIO_PE10, .event = EVENT_BUTTON_Y_DOWN,
		.delay = 350, .repeat = 80 },
	{ .n.fn = button_callback, .pin = GPIO_PC4,  .event = EVENT_BUTTON_POWER_DOWN,
		.delay = 50, .repeat = 0 },
};

void
GPIO_EVEN_IRQHandler(void)
{
	unsigned int flags = gpio_flags_enabled(gpio_flags());

	if (gpio_flag(flags, GPIO_PF2))
		button_firstclick(&buttons[0]);
	if (gpio_flag(flags, GPIO_PE10))
		button_firstclick(&buttons[3]);
	if (gpio_flag(flags, GPIO_PC4))
		button_firstclick(&buttons[4]);
}

void
GPIO_ODD_IRQHandler(void)
{
	unsigned int flags = gpio_flags_enabled(gpio_flags());

	if (gpio_flag(flags, GPIO_PF3))
		button_firstclick(&buttons[1]);
	if (gpio_flag(flags, GPIO_PE11))
		button_firstclick(&buttons[2]);
}

static void
i2c0_init(void)
{
	clock_peripheral_div1();
	clock_i2c0_enable();
	i2c0_clock_div(3); /* Thigh = Tlow = (4*(3 + 1) + 4) / (14MHz) = 1.43 us */
	i2c0_pins(I2C_PINS_ENABLE6); /* SDA -> PE12, SCL -> PE13 */
	i2c0_config(I2C_CONFIG_ENABLE);
	i2c0_abort(); /* we're the only master, so start transmitting immediately */
	i2c0_flags_enable(I2C_FLAG_NACK | I2C_FLAG_MSTOP);

	NVIC_SetPriority(I2C0_IRQn, 1);
	NVIC_EnableIRQ(I2C0_IRQn);
}

static const uint16_t rgb_steps[] = {
	0, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610,
	987, 1597, 2584, 4181, 6765, 10946, 17711, 28657, 46368
};
#define RGB_STEPS ARRAY_SIZE(rgb_steps)

static void __unused
rgb_init(void)
{
	clock_timer2_enable();
	timer2_config(TIMER_CONFIG_UP);
	timer2_pins(TIMER_PINS_LOCATION0
	          | TIMER_PINS_CC2_ENABLE
	          | TIMER_PINS_CC1_ENABLE
	          | TIMER_PINS_CC0_ENABLE);
	timer2_top_set(rgb_steps[ARRAY_SIZE(rgb_steps)-1]);
	timer2_cc_config(0, TIMER_CC_CONFIG_PWM | TIMER_CC_CONFIG_INVERT);
	timer2_cc_config(1, TIMER_CC_CONFIG_PWM | TIMER_CC_CONFIG_INVERT);
	timer2_cc_config(2, TIMER_CC_CONFIG_PWM | TIMER_CC_CONFIG_INVERT);
}

static inline void
rgb_on(void)
{
	timer2_start();
}

static inline void
rgb_off(void)
{
	timer2_stop();
}

static void __unused
rgb_set(unsigned int r, unsigned int g, unsigned int b)
{
	if (r >= RGB_STEPS)
		r = RGB_STEPS - 1;
	if (g >= RGB_STEPS)
		g = RGB_STEPS - 1;
	if (b >= RGB_STEPS)
		b = RGB_STEPS - 1;
	timer2_cc_buffer_set(0, rgb_steps[r]);
	timer2_cc_buffer_set(1, rgb_steps[g]);
	timer2_cc_buffer_set(2, rgb_steps[b]);
}

static const struct {
	gpio_pin_t pin;
	uint8_t state;
	uint8_t mode;
} gpio_pins[] = {
	{ GPIO_PA8,  1, GPIO_MODE_WIREDAND }, /* red   LED */
	{ GPIO_PA9,  1, GPIO_MODE_WIREDAND }, /* green LED */
	{ GPIO_PA10, 1, GPIO_MODE_WIREDAND }, /* blue  LED */
	{ GPIO_PC4,  1, GPIO_MODE_INPUTPULLFILTER }, /* POWER button */
	{ GPIO_PE10, 1, GPIO_MODE_INPUTPULLFILTER }, /* Y button */
	{ GPIO_PE11, 1, GPIO_MODE_INPUTPULLFILTER }, /* X button */
	{ GPIO_PE12, 1, GPIO_MODE_WIREDAND }, /* I2C0 SDA */
	{ GPIO_PE13, 1, GPIO_MODE_WIREDAND }, /* I2C0 SCL */
	{ GPIO_PF2,  1, GPIO_MODE_INPUTPULLFILTER }, /* A button */
	{ GPIO_PF3,  1, GPIO_MODE_INPUTPULLFILTER }, /* B button */
};

static void
gpio_init(void)
{
	unsigned int i;

	clock_gpio_enable();
	for (i = 0; i < ARRAY_SIZE(gpio_pins); i++) {
		gpio_pin_t pin = gpio_pins[i].pin;

		if (gpio_pins[i].state)
			gpio_set(pin);
		else
			gpio_clear(pin);
		gpio_mode(pin, gpio_pins[i].mode);
	}
}

static void
gpio_uninit(void)
{
	unsigned int i;

	/* turn off all initialised pins except the POWER button */
	for (i = 0; i < ARRAY_SIZE(gpio_pins); i++) {
		gpio_pin_t pin = gpio_pins[i].pin;

		if (gpio_pin_eq(pin, GPIO_PC4))
			continue;

		gpio_mode(pin, GPIO_MODE_DISABLED);
		gpio_clear(pin);
	}
}

static void
buttons_init(void)
{
	unsigned int i;

	NVIC_SetPriority(GPIO_EVEN_IRQn, 3);
	NVIC_SetPriority(GPIO_ODD_IRQn, 3);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		gpio_pin_t pin = buttons[i].pin;

		gpio_flag_select(pin);
		gpio_flag_falling_enable(pin);
		gpio_flag_clear(pin);
		gpio_flag_enable(pin);
	}
}

static void __noreturn
enter_em4(void)
{
	/* clear all wake-up requests */
	gpio_wakeup_clear();
	/* enable EM4 retention */
	gpio_retention_enable();
	/* wake up when pin goes low */
	gpio_wakeup_rising(0);
	/* enable EM4 wake-up from PC4 */
	gpio_wakeup_pins(GPIO_WAKEUP_PC4);

	/* do the EM4 handshake */
	emu_em4_enter();
}
static void
display_fish(struct display *dp, int wherex, int wherey, int fishsize, bool headleft)
{
	int fishsign=1;
	int w = 128;
	int h = 64;
	if (headleft){
		fishsign=-1;
		wherex+=6*fishsize;
	}

	for (int indy=0 ; indy<fishsize ; indy++)
	{
		/*Topflin*/
		for (int ind=1*fishsize ; ind <3*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,(wherey+indy)%h);
		}
		/*Above the eye*/
		for (int ind=0*fishsize ; ind <4*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+1*fishsize+indy)%h);
		}
		for (int ind=5*fishsize ; ind <6*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+1*fishsize+indy)%h);
		}
		/*The Eye one*/
		for (int ind=0*fishsize ; ind <1*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+2*fishsize+indy)%h);
		}
		for (int ind=2*fishsize ; ind <6*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+2*fishsize+indy)%h);
		}
		/*The Eye two*/
		for (int ind=0*fishsize ; ind <1*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			  (wherey+3*fishsize+indy)%h);
		}
		for (int ind=2*fishsize ; ind <6*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+3*fishsize+indy)%h);
		}
		/*Below the eye*/
		for (int ind=0*fishsize ; ind <4*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+4*fishsize+indy)%h);
		}
		for (int ind=5*fishsize ; ind <6*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+4*fishsize+indy)%h);
		}
		/*Lowerfin*/
		for (int ind=1*fishsize ; ind <2*fishsize ; ind++)
		{
			display_set(dp,(wherex+fishsign*ind)%w,
			(wherey+5*fishsize+indy)%h);
		}
	}

}

void renderLine(struct display *dp, double x1, double y1,
	double x2, double y2){
		double dist12x=x2-x1;
		double dist12y=y2-y1;
		int sign12x=(int) (dist12x/fabs(dist12x));
		int sign12y=(int) (dist12y/fabs(dist12y));
		double a=1.0;
		if (fabs(dist12x)>fabs(dist12y)){
			a=dist12y/dist12x;
			for (int ii=0;ii!=(int) dist12x+sign12x; ii+=sign12x){
				double y=y1+a*((double) ii);
				if ((x1+(double) ii)>0 && (x1+(double) ii)<128
				&& y>0 && y<64)
				display_set(dp,ii+(int) x1,(int)y);
			}
		}
		else
		{
			if (fabs(dist12y)>fabs(dist12x)){
				a=dist12x/dist12y;
			}
			for (int ii=0;ii!=(int) dist12y+sign12y; ii+=sign12y){
				double x=x1+a*((double) ii);
				if (x>0 && x<128 &&
					 (y1+(double) ii) >0 && (y1+(double) ii)<64){
					display_set(dp,(int) x, ii+(int) y1);
				}
			}
		}
}
void renderCoordinateSystem(struct display *dp){
	renderLine(dp,0.0,32.0,128.0,32.0);
	renderLine(dp,0.0,31.0,128.0,31.0);
	renderLine(dp,64.0,0.0,64.0,64.0);
	renderLine(dp,63.0,0.0,63.0,64.0);
}
void renderRectangle(struct display *dp, double x1, double y1, double x2, double y2){
	renderLine(dp,x1,y1,x1,y2);
	renderLine(dp,x1,y2,x2,y2);
	renderLine(dp,x2,y2,x2,y1);
	renderLine(dp,x2,y1,x1,y1);
}
void pend_updatedist(pendulum *pend) {
	pend->distlu=norm(pend->lx-pend->ux,pend->ly-pend->uy);
	pend->distuf=norm(pend->ux-pend->fx,pend->uy-pend->fy);
}
pendulum doublependulum (double fx, double fy, double ux, double uy, double lx, double ly){
	pendulum pend;
	pend.fx=fx;
	pend.fy=fy;
	pend.ux=ux;
	pend.uy=uy;
	pend.lx=lx;
	pend.ly=ly;
	pend.vux=0.0;
	pend.vuy=0.0;
	pend.vlx=0.0;
	pend.vly=0.0;
	pend.aux=0.0;
	pend.auy=0.0;
	pend.alx=0.0;
	pend.aly=0.0;
	pend.deltax=0.0;
	pend.deltay=0.0;
	pend.maxdistlu=15.0;
	pend.maxdistuf=15.0;
	pend.g=-10.0;
	pend.distlu=0.0;
	pend.distuf=0.0;
	pend_updatedist(&pend);
	if (pend.distlu>pend.maxdistlu) {
		pend.maxdistlu=pend.distlu;
	}
	if (pend.distuf>pend.maxdistuf) {
		pend.maxdistuf=pend.distuf;
	}
	return pend;
}
void pend_render(struct display *dp,pendulum * pend){
	int lx=(int) pend->lx;
	int ly=64 - (int) pend->ly;
	int ux=(int) pend->ux;
	int uy=64 - (int) pend->uy;
	for (int ii=-1;ii<2;ii++){
		for (int jj=-1;jj<2;jj++){
			if(lx+ii>0 && ly+jj>0 && lx+ii<128 && ly+jj<64){
				display_set(dp,lx+ii,ly+jj);
			}
			if(ux+ii>0 && uy+jj>0 && ux+ii<128 && uy+jj<64){
				display_set(dp,ux+ii,uy+jj);
			}
		}
	}
	renderLine(dp,pend->ux,64.0-pend->uy,pend->lx,64.0-pend->ly);
	renderLine(dp,pend->fx,64.0-pend->fy,pend->ux,64.0-pend->uy);
}
void pend_updatetime(pendulum *pend,double t){
	pend_updatedist(pend);
	if (pend->distuf<pend->maxdistuf-0.1){
		pend->auy+=pend->g*t;
		pend->vuy+=pend->auy*t;
		pend->uy +=pend->vuy*t;
		pend_updatedist(pend);
		if (pend->distuf>=pend->maxdistuf){
			pend->ux
			=(pend->maxdistuf/pend->distuf)*(pend->ux-pend->fx)+pend->fx;
			pend->uy
			=(pend->maxdistuf/pend->distuf)*(pend->uy-pend->fy)+pend->fy;
		}
	}
	else{
		pend->ux
		=(pend->maxdistuf/pend->distuf)*(pend->ux-pend->fx)+pend->fx;
		pend->uy
		=(pend->maxdistuf/pend->distuf)*(pend->uy-pend->fy)+pend->fy;

		double normvectx=normvecx(pend->fx,pend->fy,pend->ux,pend->uy);
		double normvecty=normvecy(pend->fx,pend->fy,pend->ux,pend->uy);
		double ax=pend->g*normvecty*normvectx;
		double ay=pend->g*normvecty*normvecty;
		pend->aux=//(pend->aux*normvectx+pend->auy*normvecty)*normvectx;
					norm(pend->aux,pend->auy)*
					(pend->aux*normvectx+pend->auy*normvecty)/
					fabs(pend->aux*normvectx+pend->auy*normvecty)
					*normvectx;
		pend->auy=//(pend->aux*normvectx+pend->auy*normvecty)*normvecty;
					norm(pend->aux,pend->auy)*
					(pend->aux*normvectx+pend->auy*normvecty)/
					fabs(pend->aux*normvectx+pend->auy*normvecty)
					*normvecty;
		pend->aux+=ax;
		pend->auy+=ay;
		pend->vux+=pend->aux*t;
		pend->vuy+=pend->auy*t;
		pend->ux+=pend->vux*t+0.5*pend->aux*t*t;
		pend->uy+=pend->vuy*t+0.5*pend->auy*t*t;
		pend->deltax=pend->vux;
		pend->deltay=pend->vuy;

		pend_updatedist(pend);
		pend->ux
		=(pend->maxdistuf/pend->distuf)*(pend->ux-pend->fx)+pend->fx;
		pend->uy
		=(pend->maxdistuf/pend->distuf)*(pend->uy-pend->fy)+pend->fy;
	}


	pend_updatedist(pend);
	if (pend->distlu<pend->maxdistlu-0.5){
		pend->aly+=pend->g*t;
		pend->vly+=pend->aly*t;
		pend->ly +=pend->vly*t;
		pend_updatedist(pend);
		if (pend->distlu>=pend->maxdistlu){
			pend->lx
			=(pend->maxdistlu/pend->distlu)*(pend->lx-pend->ux)+pend->ux;
			pend->ly
			=(pend->maxdistlu/pend->distlu)*(pend->ly-pend->uy)+pend->uy;
		}
	}
	else{
		pend->lx
		=(pend->maxdistlu/pend->distlu)*(pend->lx-pend->ux)+pend->ux;
		pend->ly
		=(pend->maxdistlu/pend->distlu)*(pend->ly-pend->uy)+pend->uy;

		double normvectx=normvecx(pend->ux,pend->uy,pend->lx,pend->ly);
		double normvecty=normvecy(pend->ux,pend->uy,pend->lx,pend->ly);
		double ax=pend->g*normvecty*normvectx;
		double ay=pend->g*normvecty*normvecty;
		pend->alx=//(pend->alx*normvectx+pend->aly*normvecty)*normvectx;
					norm(pend->alx,pend->aly)*
					(pend->alx*normvectx+pend->aly*normvecty)/
					fabs(pend->alx*normvectx+pend->aly*normvecty)
					*normvectx;
		pend->aly=//(pend->alx*normvectx+pend->aly*normvecty)*normvecty;
					norm(pend->alx,pend->aly)*
					(pend->alx*normvectx+pend->aly*normvecty)/
					fabs(pend->alx*normvectx+pend->aly*normvecty)
					*normvecty;


		pend->alx+=ax;
		pend->aly+=ay;
		pend->vlx+=pend->alx*t;
		pend->vly+=pend->aly*t;
		pend->lx+=pend->vlx*t+0.5*pend->alx*t*t;
		pend->ly+=pend->vly*t+0.5*pend->aly*t*t;

		pend_updatedist(pend);
		pend->lx
		=(pend->maxdistlu/pend->distlu)*(pend->lx-pend->ux)+pend->ux;
		pend->ly
		=(pend->maxdistlu/pend->distlu)*(pend->ly-pend->uy)+pend->uy;
	}
}
void __noreturn
main(void)
{
	unsigned int i = 0;
	unsigned int rgb[3] = { 0, 0, 0 };
	unsigned int pict = 0;
	int fishTime=59;
	int randomfishhight=1;
	int randomfishpos=6;
	struct ticker_data tick500;
	pendulum pend=doublependulum(64.0,63.0,54.0,59.0,54.0,53.0);
//	bool rgb_enabled;

	/* auxhfrco is only needed when programming flash */
	clock_auxhfrco_disable();

	/* enable low-energy clock and configure low-frequency clocks */
	clock_le_enable();
	clock_lf_config(CLOCK_LFA_ULFRCO | CLOCK_LFB_DISABLED | CLOCK_LFC_DISABLED);
	clock_rtc_div1();
	clock_rtc_enable();
	while (clock_lf_syncbusy())
		/* wait */;

	/* enable rtc */
	NVIC_SetPriority(RTC_IRQn, 2);
	NVIC_EnableIRQ(RTC_IRQn);
	rtc_config(RTC_ENABLE);

	/* enable and configure GPIOs */
	gpio_init();

	/* initialize i2c for display */
	i2c0_init();

	/* don't buffer stdout */
	setbuf(stdout, NULL);

	/* initialize and turn on display */
	display_init(&dp);
	display_update(&dp);
	display_on(&dp);

	/* initialize RGB diode */
	rgb_init();
	rgb_on();
//	rgb_enabled = true;

	/* make sure the POWER button is released */
	while (!gpio_in(GPIO_PC4))
		msleep(50);
	msleep(10);

	/* now we can start listening for button presses */
	buttons_init();

	ticker_run(&tick500, EVENT_TICK500, 50);

	while (1) {
		switch (event_pop()) {
		case EVENT_LAST:
			if (pict==1){
				display_clear(&dp);
				printf("\n\n"
						"    Bornhaxx\n"
						" Make Tradition\n"
						"      2017\n"
						"   bornhack.dk\n"
						"    %2d %2d %2d\n"
						"    %cR %cG %cB",
						rgb[0], rgb[1], rgb[2],
						(i==0) ? '*' : ' ',
						(i==1) ? '*' : ' ',
						(i==2) ? '*' : ' ');
				display_update(&dp);
			}
			if (pict==0){
				display_clear(&dp);
				printf("\n\n"
						"    Bornhack\n"
						" Make Tradition\n"
						"      2017\n"
						"   bornhack.dk\n"
						"    %2d %2d %2d\n"
						"    %cR %cG %cB",
						rgb[0], rgb[1], rgb[2],
						(i==0) ? '*' : ' ',
						(i==1) ? '*' : ' ',
						(i==2) ? '*' : ' ');
				display_update(&dp);
			}
			break;
		case EVENT_TICK500:
		if (pict==2) {
				display_clear(&dp);
				fishTime--;
				if (fishTime<=0)
				{
					fishTime=128;
					randomfishhight = rand() % 8+1;
					randomfishpos = rand() % (64-6*randomfishhight);
				}
				display_fish(&dp,fishTime, randomfishpos, randomfishhight, false);
				display_update(&dp);
		}
		if (pict==3){
			display_clear(&dp);
			renderCoordinateSystem(&dp);
//			renderRectangle(&dp,64-26,32-13,64+26,32+13);
			pend_render(&dp,&pend);
		/*	if ((pend.aux)/16.0+64.0>0 && (pend.aux)/16.0+64.0<128){
				display_set(&dp,((int) (pend.aux/16.0))+64,
				64-((int) (pend.auy/16.0))-32);
			}*/
			if ((pend.deltax)/16.0+64.0>0 && (pend.deltax)/16.0+64.0<128){
				display_set(&dp,((int) (pend.deltax/16.0))+64,
				64-((int) (pend.deltay/16.0))-32);
			}
			display_update(&dp);
			pend_updatetime(&pend,0.05);
		}
		break;
		case EVENT_BUTTON_A_DOWN:
			if (i > 0)
				i--;
			break;
		case EVENT_BUTTON_B_DOWN:
			if (i < 2)
				i++;
			break;
		case EVENT_BUTTON_X_DOWN:
			if (pict<3)
				pict++;
			else
				pict=0;
			break;
		case EVENT_BUTTON_Y_DOWN:
			if (rgb[i] < RGB_STEPS-1)
				rgb[i]++;
			else
				rgb[i]=0;
			rgb_set(rgb[0], rgb[1], rgb[2]);
			break;
		case EVENT_BUTTON_POWER_UP:
			NVIC_DisableIRQ(RTC_IRQn);
			NVIC_DisableIRQ(GPIO_EVEN_IRQn);
			NVIC_DisableIRQ(GPIO_ODD_IRQn);

			display_off(&dp);
			gpio_uninit();
			enter_em4();
			break;
		default:
			/* do nothing */
			break;
		}
	}
}
