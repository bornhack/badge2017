#include "common.h"

#ifdef GPIO_TYPE_STRUCT

typedef struct {
#ifdef GPIO_TYPE_8BIT
	unsigned port : 4;
	unsigned nr   : 4;
#else
	uint8_t port;
	uint8_t nr;
#endif
} gpio_pin_t;
#define GPIO_PIN(p, n) ((gpio_pin_t){ p, n })

static inline unsigned int
gpio_port(gpio_pin_t pin)          { return pin.port; }
static inline unsigned int
gpio_nr(gpio_pin_t pin)            { return pin.nr; }
static inline bool
gpio_pin_eq(gpio_pin_t a, gpio_pin_t b)
{
	return (a.port == b.port) && (a.nr == b.nr);
}

#else
#ifdef GPIO_TYPE_8BIT

typedef uint8_t gpio_pin_t;
#define GPIO_PIN(p,n) (((gpio_pin_t)(p)<<4)|(gpio_pin_t)(n))

static inline unsigned int
gpio_port(gpio_pin_t pin)          { return pin >> 4; }
static inline unsigned int
gpio_nr(gpio_pin_t pin)            { return pin & 0xFU; }

#else

typedef uint16_t gpio_pin_t;
#define GPIO_PIN(p,n) (((gpio_pin_t)(p)<<8)|(gpio_pin_t)(n))

static inline unsigned int
gpio_port(gpio_pin_t pin)          { return pin >> 8; }
static inline unsigned int
gpio_nr(gpio_pin_t pin)            { return pin & 0xFFU; }

#endif

static inline bool
gpio_pin_eq(gpio_pin_t a, gpio_pin_t b)
{
	return a == b;
}
#endif

#define GPIO_PA0  GPIO_PIN(0,  0)
#define GPIO_PA1  GPIO_PIN(0,  1)
#define GPIO_PA2  GPIO_PIN(0,  2)
#define GPIO_PA3  GPIO_PIN(0,  3)
#define GPIO_PA4  GPIO_PIN(0,  4)
#define GPIO_PA5  GPIO_PIN(0,  5)
#define GPIO_PA6  GPIO_PIN(0,  6)
#define GPIO_PA7  GPIO_PIN(0,  7)
#define GPIO_PA8  GPIO_PIN(0,  8)
#define GPIO_PA9  GPIO_PIN(0,  9)
#define GPIO_PA10 GPIO_PIN(0, 10)
#define GPIO_PA11 GPIO_PIN(0, 11)
#define GPIO_PA12 GPIO_PIN(0, 12)
#define GPIO_PA13 GPIO_PIN(0, 13)
#define GPIO_PA14 GPIO_PIN(0, 14)
#define GPIO_PA15 GPIO_PIN(0, 15)

#define GPIO_PB0  GPIO_PIN(1,  0)
#define GPIO_PB1  GPIO_PIN(1,  1)
#define GPIO_PB2  GPIO_PIN(1,  2)
#define GPIO_PB3  GPIO_PIN(1,  3)
#define GPIO_PB4  GPIO_PIN(1,  4)
#define GPIO_PB5  GPIO_PIN(1,  5)
#define GPIO_PB6  GPIO_PIN(1,  6)
#define GPIO_PB7  GPIO_PIN(1,  7)
#define GPIO_PB8  GPIO_PIN(1,  8)
#define GPIO_PB9  GPIO_PIN(1,  9)
#define GPIO_PB10 GPIO_PIN(1, 10)
#define GPIO_PB11 GPIO_PIN(1, 11)
#define GPIO_PB12 GPIO_PIN(1, 12)
#define GPIO_PB13 GPIO_PIN(1, 13)
#define GPIO_PB14 GPIO_PIN(1, 14)
#define GPIO_PB15 GPIO_PIN(1, 15)

#define GPIO_PC0  GPIO_PIN(2,  0)
#define GPIO_PC1  GPIO_PIN(2,  1)
#define GPIO_PC2  GPIO_PIN(2,  2)
#define GPIO_PC3  GPIO_PIN(2,  3)
#define GPIO_PC4  GPIO_PIN(2,  4)
#define GPIO_PC5  GPIO_PIN(2,  5)
#define GPIO_PC6  GPIO_PIN(2,  6)
#define GPIO_PC7  GPIO_PIN(2,  7)
#define GPIO_PC8  GPIO_PIN(2,  8)
#define GPIO_PC9  GPIO_PIN(2,  9)
#define GPIO_PC10 GPIO_PIN(2, 10)
#define GPIO_PC11 GPIO_PIN(2, 11)
#define GPIO_PC12 GPIO_PIN(2, 12)
#define GPIO_PC13 GPIO_PIN(2, 13)
#define GPIO_PC14 GPIO_PIN(2, 14)
#define GPIO_PC15 GPIO_PIN(2, 15)

#define GPIO_PD0  GPIO_PIN(3,  0)
#define GPIO_PD1  GPIO_PIN(3,  1)
#define GPIO_PD2  GPIO_PIN(3,  2)
#define GPIO_PD3  GPIO_PIN(3,  3)
#define GPIO_PD4  GPIO_PIN(3,  4)
#define GPIO_PD5  GPIO_PIN(3,  5)
#define GPIO_PD6  GPIO_PIN(3,  6)
#define GPIO_PD7  GPIO_PIN(3,  7)
#define GPIO_PD8  GPIO_PIN(3,  8)
#define GPIO_PD9  GPIO_PIN(3,  9)
#define GPIO_PD10 GPIO_PIN(3, 10)
#define GPIO_PD11 GPIO_PIN(3, 11)
#define GPIO_PD12 GPIO_PIN(3, 12)
#define GPIO_PD13 GPIO_PIN(3, 13)
#define GPIO_PD14 GPIO_PIN(3, 14)
#define GPIO_PD15 GPIO_PIN(3, 15)

#define GPIO_PE0  GPIO_PIN(4,  0)
#define GPIO_PE1  GPIO_PIN(4,  1)
#define GPIO_PE2  GPIO_PIN(4,  2)
#define GPIO_PE3  GPIO_PIN(4,  3)
#define GPIO_PE4  GPIO_PIN(4,  4)
#define GPIO_PE5  GPIO_PIN(4,  5)
#define GPIO_PE6  GPIO_PIN(4,  6)
#define GPIO_PE7  GPIO_PIN(4,  7)
#define GPIO_PE8  GPIO_PIN(4,  8)
#define GPIO_PE9  GPIO_PIN(4,  9)
#define GPIO_PE10 GPIO_PIN(4, 10)
#define GPIO_PE11 GPIO_PIN(4, 11)
#define GPIO_PE12 GPIO_PIN(4, 12)
#define GPIO_PE13 GPIO_PIN(4, 13)
#define GPIO_PE14 GPIO_PIN(4, 14)
#define GPIO_PE15 GPIO_PIN(4, 15)

#define GPIO_PF0  GPIO_PIN(5,  0)
#define GPIO_PF1  GPIO_PIN(5,  1)
#define GPIO_PF2  GPIO_PIN(5,  2)
#define GPIO_PF3  GPIO_PIN(5,  3)
#define GPIO_PF4  GPIO_PIN(5,  4)
#define GPIO_PF5  GPIO_PIN(5,  5)
#define GPIO_PF6  GPIO_PIN(5,  6)
#define GPIO_PF7  GPIO_PIN(5,  7)
#define GPIO_PF8  GPIO_PIN(5,  8)
#define GPIO_PF9  GPIO_PIN(5,  9)
#define GPIO_PF10 GPIO_PIN(5, 10)
#define GPIO_PF11 GPIO_PIN(5, 11)
#define GPIO_PF12 GPIO_PIN(5, 12)
#define GPIO_PF13 GPIO_PIN(5, 13)
#define GPIO_PF14 GPIO_PIN(5, 14)
#define GPIO_PF15 GPIO_PIN(5, 15)

/* GPIO_Px_CTRL */
static void
gpio_drive_strength(unsigned int port, uint32_t v)
{
	GPIO->P[port].CTRL = v;
}
static void __unused
gpio_drive_strength_standard(gpio_pin_t pin)
{
	gpio_drive_strength(gpio_port(pin), GPIO_P_CTRL_DRIVEMODE_STANDARD);
}
static void __unused
gpio_drive_strength_lowest(gpio_pin_t pin)
{
	gpio_drive_strength(gpio_port(pin), GPIO_P_CTRL_DRIVEMODE_LOWEST);
}
static void __unused
gpio_drive_strength_high(gpio_pin_t pin)
{
	gpio_drive_strength(gpio_port(pin), GPIO_P_CTRL_DRIVEMODE_HIGH);
}
static void __unused
gpio_drive_strength_low(gpio_pin_t pin)
{
	gpio_drive_strength(gpio_port(pin), GPIO_P_CTRL_DRIVEMODE_LOW);
}

/* GPIO_Px_MODEy */
enum gpio_mode {
	GPIO_MODE_DEFAULT                   = 0x00U,
	GPIO_MODE_DISABLED                  = 0x00U,
	GPIO_MODE_INPUT                     = 0x01U,
	GPIO_MODE_INPUTPULL                 = 0x02U,
	GPIO_MODE_INPUTPULLFILTER           = 0x03U,
	GPIO_MODE_PUSHPULL                  = 0x04U,
	GPIO_MODE_PUSHPULLDRIVE             = 0x05U,
	GPIO_MODE_WIREDOR                   = 0x06U,
	GPIO_MODE_WIREDORPULLDOWN           = 0x07U,
	GPIO_MODE_WIREDAND                  = 0x08U,
	GPIO_MODE_WIREDANDFILTER            = 0x09U,
	GPIO_MODE_WIREDANDPULLUP            = 0x0AU,
	GPIO_MODE_WIREDANDPULLUPFILTER      = 0x0BU,
	GPIO_MODE_WIREDANDDRIVE             = 0x0CU,
	GPIO_MODE_WIREDANDDRIVEFILTER       = 0x0DU,
	GPIO_MODE_WIREDANDDRIVEPULLUP       = 0x0EU,
	GPIO_MODE_WIREDANDDRIVEPULLUPFILTER = 0x0FU,
};

static void __unused
gpio_mode(gpio_pin_t pin, uint32_t mode)
{
	volatile uint32_t *reg;
	uint32_t pinshift;
	uint32_t mask;
	unsigned int port = gpio_port(pin);

	if (gpio_nr(pin) & 0x8U)
		reg = &GPIO->P[port].MODEH;
	else
		reg = &GPIO->P[port].MODEL;

	pinshift = 4*(gpio_nr(pin) & 0x7U);
	mask = 0xFU << pinshift;
	*reg = (*reg & ~mask) | (mode << pinshift);
}

/* GPIO_Px_DOUT */

/* GPIO_Px_DOUTSET */
static inline void
gpio_set(gpio_pin_t pin)
{
	GPIO->P[gpio_port(pin)].DOUTSET = 1U << gpio_nr(pin);
}

/* GPIO_Px_DOUTCLR */
static inline void
gpio_clear(gpio_pin_t pin)
{
	GPIO->P[gpio_port(pin)].DOUTCLR = 1U << gpio_nr(pin);
}

/* GPIO_Px_DOUTTGL */
static inline void
gpio_toggle(gpio_pin_t pin)
{
	GPIO->P[gpio_port(pin)].DOUTTGL = 1U << gpio_nr(pin);
}

/* GPIO_Px_DIN */
static inline uint32_t
gpio_in(gpio_pin_t pin)
{
	return GPIO->P[gpio_port(pin)].DIN & (1U << gpio_nr(pin));
}

/* GPIO_Px_PINLOCKN */

/* GPIO_EXTIPSELy */
static void __unused
gpio_flag_select(gpio_pin_t pin)
{
	volatile uint32_t *reg;
	uint32_t pinshift;
	uint32_t mask;
	unsigned int port = gpio_port(pin);

	if (gpio_nr(pin) & 0x8U)
		reg = &GPIO->EXTIPSELH;
	else
		reg = &GPIO->EXTIPSELL;

	pinshift = 4*(gpio_nr(pin) & 0x7U);
	mask = 0xFU << pinshift;
	*reg = (*reg & ~mask) | (port << pinshift);
}

/* GPIO_EXTIRISE */
static void __unused
gpio_flag_rising_disable(gpio_pin_t pin)
{
	GPIO->EXTIRISE &= ~(1U << gpio_nr(pin));
}
static void __unused
gpio_flag_rising_enable(gpio_pin_t pin)
{
	GPIO->EXTIRISE |= 1U << gpio_nr(pin);
}

/* GPIO_EXTIFALL */
static void __unused
gpio_flag_falling_disable(gpio_pin_t pin)
{
	GPIO->EXTIFALL &= ~(1U << gpio_nr(pin));
}
static void __unused
gpio_flag_falling_enable(gpio_pin_t pin)
{
	GPIO->EXTIFALL |= 1U << gpio_nr(pin);
}

/* GPIO_IEN */
static inline void
gpio_flags_disable_all(void)       { GPIO->IEN = 0; }
static inline uint32_t
gpio_flags_enabled(uint32_t v)     { return v & GPIO->IEN; }
static void __unused
gpio_flag_disable(gpio_pin_t pin)  { GPIO->IEN &= ~(1U << gpio_nr(pin)); }
static void __unused
gpio_flag_enable(gpio_pin_t pin)   { GPIO->IEN |= 1U << gpio_nr(pin); }
static uint32_t __unused
gpio_flag_enabled(gpio_pin_t pin)  { return GPIO->IEN & (1U << gpio_nr(pin)); }

/* GPIO_IF */
static inline uint32_t
gpio_flags(void)                   { return GPIO->IF; }
static inline uint32_t
gpio_flags_even(void)              { return GPIO->IF & 0x5555U; }
static inline uint32_t
gpio_flags_odd(void)               { return GPIO->IF & 0xAAAAU; }
static uint32_t __unused
gpio_flag(uint32_t flags, gpio_pin_t pin)
{
	return flags & (1U << gpio_nr(pin));
}

/* GPIO_IFS */
static inline void
gpio_flags_set(uint32_t v)         { GPIO->IFS = v; }
static void __unused
gpio_flag_set(gpio_pin_t pin)      { GPIO->IFS = 1U << gpio_nr(pin); }

/* GPIO_IFC */
static inline void
gpio_flags_clear(uint32_t v)       { GPIO->IFC = v; }
static inline void
gpio_flags_clear_all(void)         { GPIO->IFC = 0xFFFFFU; }
static inline void
gpio_flags_clear_even(uint32_t v)  { GPIO->IFC = v & 0x5555U; }
static inline void
gpio_flags_clear_odd(uint32_t v)   { GPIO->IFC = v & 0xAAAAU; }
static void __unused
gpio_flag_clear(gpio_pin_t pin)    { GPIO->IFC = 1U << gpio_nr(pin); }

/* GPIO_ROUTE */
static inline void
gpio_swd_disable(void)             { GPIO->ROUTE = 0; }
static inline void
gpio_swd_enable(void)              { GPIO->ROUTE = GPIO_ROUTE_SWDIOPEN | GPIO_ROUTE_SWCLKPEN; }

/* GPIO_INSENSE */
static inline void
gpio_sense_disable(void)           { GPIO->INSENSE = 0; }
static inline void
gpio_sense_enable(void)            { GPIO->INSENSE = GPIO_INSENSE_PRS | GPIO_INSENSE_INT; }
static inline void
gpio_sense_prs_disable(void)       { GPIO->INSENSE &= ~GPIO_INSENSE_PRS; }
static inline void
gpio_sense_prs_enable(void)        { GPIO->INSENSE |= GPIO_INSENSE_PRS; }
static inline void
gpio_sense_interrupt_disable(void) { GPIO->INSENSE &= ~GPIO_INSENSE_INT; }
static inline void
gpio_sense_interrupt_enable(void)  { GPIO->INSENSE |= GPIO_INSENSE_INT; }

/* GPIO_LOCK */

/* GPIO_CTRL */
static inline void
gpio_retention_disable(void)       { GPIO->CTRL = 0; }
static inline void
gpio_retention_enable(void)        { GPIO->CTRL = GPIO_CTRL_EM4RET; }

/* GPIO_CMD */
static inline void
gpio_wakeup_clear(void)            { GPIO->CMD = GPIO_CMD_EM4WUCLR; }

/* GPIO_EM4WUEN */
enum gpio_wakeup {
	GPIO_WAKEUP_PA0  = GPIO_EM4WUEN_EM4WUEN_A0,
	GPIO_WAKEUP_PC9  = GPIO_EM4WUEN_EM4WUEN_C9,
	GPIO_WAKEUP_PF1  = GPIO_EM4WUEN_EM4WUEN_F1,
	GPIO_WAKEUP_PF2  = GPIO_EM4WUEN_EM4WUEN_F2,
	GPIO_WAKEUP_PE13 = GPIO_EM4WUEN_EM4WUEN_E13,
	GPIO_WAKEUP_PC4  = GPIO_EM4WUEN_EM4WUEN_C4,
};

static inline void
gpio_wakeup_pins(uint32_t v)       { GPIO->EM4WUEN = v; }

/* GPIO_EM4WUPOL */
static inline void
gpio_wakeup_rising(uint32_t v)     { GPIO->EM4WUPOL = v; }

/* GPIO_EM4WUCAUSE */
static inline uint32_t
gpio_wakeup_cause(void)            { return GPIO->EM4WUCAUSE; }
