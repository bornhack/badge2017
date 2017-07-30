#include "common.h"

/* CMU_CMD */
static inline void
clock_usbc_select_lfxo(void)      { CMU->CMD = CMU_CMD_USBCCLKSEL_LFXO; }
static inline void
clock_usbc_select_lfrco(void)     { CMU->CMD = CMU_CMD_USBCCLKSEL_LFRCO; }
static inline void
clock_usbc_select_ushfrco(void)   { CMU->CMD = CMU_CMD_USBCCLKSEL_USHFRCO; }

static inline void
clock_calibration_stop(void)      { CMU->CMD = CMU_CMD_CALSTOP; }
static inline void
clock_calibration_start(void)     { CMU->CMD = CMU_CMD_CALSTART; }

static inline void
clock_hfclk_select_hfrco(void)    { CMU->CMD = CMU_CMD_HFCLKSEL_HFRCO; }
static inline void
clock_hfclk_select_hfxo(void)     { CMU->CMD = CMU_CMD_HFCLKSEL_HFXO; }
static inline void
clock_hfclk_select_lfrco(void)    { CMU->CMD = CMU_CMD_HFCLKSEL_LFRCO; }
static inline void
clock_hfclk_select_ushfrco(void)  { CMU->CMD = CMU_CMD_HFCLKSEL_USHFRCODIV2; }

/* CMU_HFCORECLKDIV */
static inline void
clock_le_div2(void)               { CMU->HFCORECLKDIV &= ~CMU_HFCORECLKDIV_HFCORECLKLEDIV; }
static inline void
clock_le_div4(void)               { CMU->HFCORECLKDIV |= CMU_HFCORECLKDIV_HFCORECLKLEDIV; }

/* CMU_HFPERCLKDIV */
static inline void
clock_peripheral_disable(void)    { CMU->HFPERCLKDIV = 0; }
static inline void
clock_peripheral_div1(void)       { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK; }
static inline void
clock_peripheral_div2(void)       { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK2; }
static inline void
clock_peripheral_div4(void)       { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4; }
static inline void
clock_peripheral_div8(void)       { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK8; }
static inline void
clock_peripheral_div16(void)      { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK16; }
static inline void
clock_peripheral_div32(void)      { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK32; }
static inline void
clock_peripheral_div64(void)      { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK64; }
static inline void
clock_peripheral_div128(void)     { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK128; }
static inline void
clock_peripheral_div256(void)     { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK256; }
static inline void
clock_peripheral_div512(void)     { CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN | CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK512; }

/* CMU_OSCENCMD */
static inline void
clock_ushfrco_disable(void)       { CMU->OSCENCMD = CMU_OSCENCMD_USHFRCODIS; }
static inline void
clock_ushfrco_enable(void)        { CMU->OSCENCMD = CMU_OSCENCMD_USHFRCOEN; }
static inline void
clock_lfxo_disable(void)          { CMU->OSCENCMD = CMU_OSCENCMD_LFXODIS; }
static inline void
clock_lfxo_enable(void)           { CMU->OSCENCMD = CMU_OSCENCMD_LFXOEN; }
static inline void
clock_lfrco_disable(void)         { CMU->OSCENCMD = CMU_OSCENCMD_LFRCODIS; }
static inline void
clock_lfrco_enable(void)          { CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN; }
static inline void
clock_auxhfrco_disable(void)      { CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCODIS; }
static inline void
clock_auxhfrco_enable(void)       { CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN; }
static inline void
clock_hfxo_disable(void)          { CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS; }
static inline void
clock_hfxo_enable(void)           { CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN; }
static inline void
clock_hfrco_disable(void)         { CMU->OSCENCMD = CMU_OSCENCMD_HFRCODIS; }
static inline void
clock_hfrco_enable(void)          { CMU->OSCENCMD = CMU_OSCENCMD_HFRCOEN; }

/* CMU_LFCLKSEL */
enum clock_lf_config {
	CLOCK_LFA_DISABLED = CMU_LFCLKSEL_LFA_DISABLED        | CMU_LFCLKSEL_LFAE_DISABLED,
	CLOCK_LFA_ULFRCO   = CMU_LFCLKSEL_LFA_DISABLED        | CMU_LFCLKSEL_LFAE_ULFRCO,
	CLOCK_LFA_LFRCO    = CMU_LFCLKSEL_LFA_LFRCO           | CMU_LFCLKSEL_LFAE_DISABLED,
	CLOCK_LFA_LFXO     = CMU_LFCLKSEL_LFA_LFXO            | CMU_LFCLKSEL_LFAE_DISABLED,
	CLOCK_LFA_CORECLK  = CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2 | CMU_LFCLKSEL_LFAE_DISABLED,

	CLOCK_LFB_DISABLED = CMU_LFCLKSEL_LFB_DISABLED        | CMU_LFCLKSEL_LFBE_DISABLED,
	CLOCK_LFB_ULFRCO   = CMU_LFCLKSEL_LFB_DISABLED        | CMU_LFCLKSEL_LFBE_ULFRCO,
	CLOCK_LFB_LFRCO    = CMU_LFCLKSEL_LFB_LFRCO           | CMU_LFCLKSEL_LFBE_DISABLED,
	CLOCK_LFB_LFXO     = CMU_LFCLKSEL_LFB_LFXO            | CMU_LFCLKSEL_LFBE_DISABLED,
	CLOCK_LFB_CORECLK  = CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2 | CMU_LFCLKSEL_LFBE_DISABLED,

	CLOCK_LFC_DISABLED = CMU_LFCLKSEL_LFC_DISABLED,
	CLOCK_LFC_LFRCO    = CMU_LFCLKSEL_LFC_LFRCO,
	CLOCK_LFC_LFXO     = CMU_LFCLKSEL_LFC_LFXO,
};
static inline void
clock_lf_config(uint32_t v)
{
	CMU->LFCLKSEL = v;
}

static inline void clock_lfa_disable(void)
{
	CMU->LFCLKSEL &= ~(_CMU_LFCLKSEL_LFA_MASK | CMU_LFCLKSEL_LFAE);
}
static inline void clock_lfa_select_ulfrco(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFA_MASK)) | CMU_LFCLKSEL_LFAE;
}
static inline void clock_lfa_select_lfrco(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFA_MASK | CMU_LFCLKSEL_LFAE))
	              | CMU_LFCLKSEL_LFA_LFRCO;
}
static inline void clock_lfa_select_lfxo(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFA_MASK | CMU_LFCLKSEL_LFAE))
	              | CMU_LFCLKSEL_LFA_LFXO;
}
static inline void clock_lfa_select_hfclk(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFA_MASK | CMU_LFCLKSEL_LFAE))
	              | CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2;
}

static inline void clock_lfb_disable(void)
{
	CMU->LFCLKSEL &= ~(_CMU_LFCLKSEL_LFB_MASK | CMU_LFCLKSEL_LFBE);
}
static inline void clock_lfb_select_ulfrco(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK)) | CMU_LFCLKSEL_LFBE;
}
static inline void clock_lfb_select_lfrco(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK | CMU_LFCLKSEL_LFBE))
	              | CMU_LFCLKSEL_LFB_LFRCO;
}
static inline void clock_lfb_select_lfxo(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK | CMU_LFCLKSEL_LFBE))
	              | CMU_LFCLKSEL_LFB_LFXO;
}
static inline void clock_lfb_select_hfclk(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK | CMU_LFCLKSEL_LFBE))
	              | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;
}

static inline void clock_lfc_disable(void)
{
	CMU->LFCLKSEL &= ~_CMU_LFCLKSEL_LFC_MASK;
}
static inline void clock_lfc_select_lfrco(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFC_MASK)) | CMU_LFCLKSEL_LFC_LFRCO;
}
static inline void clock_lfc_select_lfxo(void)
{
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK)) | CMU_LFCLKSEL_LFC_LFXO;
}

/* CMU_STATUS */
static inline uint32_t
clock_status(void)                { return CMU->STATUS; }
static inline uint32_t
clock_ushfrco_selected(void)      { return CMU->STATUS & CMU_STATUS_USHFRCODIV2SEL; }
static inline uint32_t
clock_ushfrco_suspended(void)     { return CMU->STATUS & CMU_STATUS_USHFRCOSUSPEND; }
static inline uint32_t
clock_ushfrco_ready(void)         { return CMU->STATUS & CMU_STATUS_USHFRCORDY; }
static inline uint32_t
clock_ushfrco_enabled(void)       { return CMU->STATUS & CMU_STATUS_USHFRCOENS; }
static inline uint32_t
clock_usbc_hfclk_sync(void)       { return CMU->STATUS & CMU_STATUS_USBCHFCLKSYNC; }
static inline uint32_t
clock_usbc_ushfrco_selected(void) { return CMU->STATUS & CMU_STATUS_USBCUSHFRCOSEL; }
static inline uint32_t
clock_usbc_lfrco_selected(void)   { return CMU->STATUS & CMU_STATUS_USBCLFRCOSEL; }
static inline uint32_t
clock_usbc_lfxo_selected(void)    { return CMU->STATUS & CMU_STATUS_USBCLFXOSEL; }
static inline uint32_t
clock_calibration_busy(void)      { return CMU->STATUS & CMU_STATUS_CALBSY; }
static inline uint32_t
clock_lfxo_selected(void)         { return CMU->STATUS & CMU_STATUS_LFXOSEL; }
static inline uint32_t
clock_lfrco_selected(void)        { return CMU->STATUS & CMU_STATUS_LFRCOSEL; }
static inline uint32_t
clock_hfxo_selected(void)         { return CMU->STATUS & CMU_STATUS_HFXOSEL; }
static inline uint32_t
clock_hfrco_selected(void)        { return CMU->STATUS & CMU_STATUS_HFRCOSEL; }
static inline uint32_t
clock_lfxo_ready(void)            { return CMU->STATUS & CMU_STATUS_LFXORDY; }
static inline uint32_t
clock_lfxo_enabled(void)          { return CMU->STATUS & CMU_STATUS_LFXOENS; }
static inline uint32_t
clock_lfrco_ready(void)            { return CMU->STATUS & CMU_STATUS_LFRCORDY; }
static inline uint32_t
clock_lfrco_enabled(void)          { return CMU->STATUS & CMU_STATUS_LFRCOENS; }
static inline uint32_t
clock_auxhfrco_ready(void)         { return CMU->STATUS & CMU_STATUS_AUXHFRCORDY; }
static inline uint32_t
clock_auxhfrco_enabled(void)       { return CMU->STATUS & CMU_STATUS_AUXHFRCOENS; }
static inline uint32_t
clock_hfxo_ready(void)             { return CMU->STATUS & CMU_STATUS_HFXORDY; }
static inline uint32_t
clock_hfxo_enabled(void)           { return CMU->STATUS & CMU_STATUS_HFXOENS; }
static inline uint32_t
clock_hfrco_ready(void)            { return CMU->STATUS & CMU_STATUS_HFRCORDY; }
static inline uint32_t
clock_hfrco_enabled(void)          { return CMU->STATUS & CMU_STATUS_HFRCOENS; }

/* CMU_HFCORECLKEN0 */
static inline void
clock_usb_disable(void)            { CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_USB; }
static inline void
clock_usb_enable(void)             { CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_USB; }
static inline void
clock_usbc_disable(void)           { CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_USBC; }
static inline void
clock_usbc_enable(void)            { CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_USB | CMU_HFCORECLKEN0_USBC; }
static inline void
clock_le_disable(void)             { CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_LE; }
static inline void
clock_le_enable(void)              { CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE; }
static inline void
clock_dma_disable(void)            { CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_DMA; }
static inline void
clock_dma_enable(void)             { CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_DMA; }
static inline void
clock_aes_disable(void)            { CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_AES; }
static inline void
clock_aes_enable(void)             { CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_AES; }

/* CMU_HFPERCLKEN0 */
static inline void
clock_i2c0_disable(void)           { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_I2C0; }
static inline void
clock_i2c0_enable(void)            { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_I2C0; }
static inline void
clock_adc0_disable(void)           { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_ADC0; }
static inline void
clock_adc0_enable(void)            { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC0; }
static inline void
clock_vcmp_disable(void)           { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_VCMP; }
static inline void
clock_vcmp_enable(void)            { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_VCMP; }
static inline void
clock_gpio_disable(void)           { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_GPIO; }
static inline void
clock_gpio_enable(void)            { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO; }
static inline void
clock_idac0_disable(void)          { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_IDAC0; }
static inline void
clock_idac0_enable(void)           { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_IDAC0; }
static inline void
clock_prs_disable(void)            { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_PRS; }
static inline void
clock_prs_enable(void)             { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_PRS; }
static inline void
clock_acmp0_disable(void)          { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_ACMP0; }
static inline void
clock_acmp0_enable(void)           { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ACMP0; }
static inline void
clock_usart1_disable(void)         { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_USART1; }
static inline void
clock_usart1_enable(void)          { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_USART1; }
static inline void
clock_usart0_disable(void)         { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_USART0; }
static inline void
clock_usart0_enable(void)          { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_USART0; }
static inline void
clock_timer2_disable(void)         { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_TIMER2; }
static inline void
clock_timer2_enable(void)          { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER2; }
static inline void
clock_timer1_disable(void)         { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_TIMER1; }
static inline void
clock_timer1_enable(void)          { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER1; }
static inline void
clock_timer0_disable(void)         { CMU->HFPERCLKEN0 &= ~CMU_HFPERCLKEN0_TIMER0; }
static inline void
clock_timer0_enable(void)          { CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0; }

/* CMU_SYNCBUSY */
static inline uint32_t
clock_lfc_syncbusy(void)           { return CMU->SYNCBUSY & CMU_SYNCBUSY_LFCCLKEN0; }
static inline uint32_t
clock_lfb_syncbusy(void)
{
	return CMU->SYNCBUSY & (CMU_SYNCBUSY_LFBPRESC0 | CMU_SYNCBUSY_LFBCLKEN0);
}
static inline uint32_t
clock_lfa_syncbusy(void)
{
	return CMU->SYNCBUSY & (CMU_SYNCBUSY_LFAPRESC0 | CMU_SYNCBUSY_LFACLKEN0);
}
static inline uint32_t
clock_lf_syncbusy(void)            { return CMU->SYNCBUSY; }

/* CMU_FREEZE */
static inline void
clock_freeze(void)                 { CMU->FREEZE = CMU_FREEZE_REGFREEZE_FREEZE; }
static inline void
clock_update(void)                 { CMU->FREEZE = CMU_FREEZE_REGFREEZE_UPDATE; }

/* CMU_LFACLKEN0 */
static inline void
clock_rtc_disable(void)            { CMU->LFACLKEN0 = 0; }
static inline void
clock_rtc_enable(void)             { CMU->LFACLKEN0 = CMU_LFACLKEN0_RTC; }

/* CMU_LFBCLKEN0 */
static inline void
clock_leuart0_disable(void)        { CMU->LFBCLKEN0 = 0; }
static inline void
clock_leuart0_enable(void)         { CMU->LFBCLKEN0 = CMU_LFBCLKEN0_LEUART0; }

/* CMU_LFCCLKEN0 */
static inline void
clock_usble_disable(void)          { CMU->LFCCLKEN0 = 0; }
static inline void
clock_usble_enable(void)           { CMU->LFCCLKEN0 = CMU_LFCCLKEN0_USBLE; }

/* CMU_LFAPRESC0 */
static inline void
clock_rtc_div1(void)               { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV1; }
static inline void
clock_rtc_div2(void)               { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV2; }
static inline void
clock_rtc_div4(void)               { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV4; }
static inline void
clock_rtc_div8(void)               { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV8; }
static inline void
clock_rtc_div16(void)              { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV16; }
static inline void
clock_rtc_div32(void)              { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV32; }
static inline void
clock_rtc_div64(void)              { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV64; }
static inline void
clock_rtc_div128(void)             { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV128; }
static inline void
clock_rtc_div256(void)             { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV256; }
static inline void
clock_rtc_div512(void)             { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV512; }
static inline void
clock_rtc_div1024(void)            { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV1024; }
static inline void
clock_rtc_div2048(void)            { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV2048; }
static inline void
clock_rtc_div4096(void)            { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV4096; }
static inline void
clock_rtc_div8192(void)            { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV8192; }
static inline void
clock_rtc_div16384(void)           { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV16384; }
static inline void
clock_rtc_div32768(void)           { CMU->LFAPRESC0 = CMU_LFAPRESC0_RTC_DIV32768; }

/* CMU_LFBPRESC0 */
static inline void
clock_leuart0_div1(void)           { CMU->LFBPRESC0 = CMU_LFBPRESC0_LEUART0_DIV1; }
static inline void
clock_leuart0_div2(void)           { CMU->LFBPRESC0 = CMU_LFBPRESC0_LEUART0_DIV2; }
static inline void
clock_leuart0_div4(void)           { CMU->LFBPRESC0 = CMU_LFBPRESC0_LEUART0_DIV4; }
static inline void
clock_leuart0_div8(void)           { CMU->LFBPRESC0 = CMU_LFBPRESC0_LEUART0_DIV8; }

/* CMU_PCNTCTRL */

/* CMU_ROUTE */

/* CMU_LOCK */

/* CMU_USBCRCTRL */
static inline void
clock_ushfrco_recovery_enable(void)
{
	CMU->USBCRCTRL = CMU_USBCRCTRL_EN;
}
static inline void
clock_ushfrco_recovery_enable_low_speed(void)
{
	CMU->USBCRCTRL = CMU_USBCRCTRL_LSMODE | CMU_USBCRCTRL_EN;
}
static inline void
clock_ushfrco_recovery_disable(void)
{
	CMU->USBCRCTRL = 0;
}

/* CMU_USHFRCOCONF */
static inline void clock_ushfrco_48mhz_div2(void)
{
	CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_48MHZ;
}
static inline void clock_ushfrco_48mhz(void)
{
	CMU->USHFRCOCONF = CMU_USHFRCOCONF_USHFRCODIV2DIS | CMU_USHFRCOCONF_BAND_48MHZ;
}
static inline void clock_ushfrco_24mhz_div2(void)
{
	CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_24MHZ;
}
static inline void clock_ushfrco_24mhz(void)
{
	CMU->USHFRCOCONF = CMU_USHFRCOCONF_USHFRCODIV2DIS | CMU_USHFRCOCONF_BAND_24MHZ;
}


