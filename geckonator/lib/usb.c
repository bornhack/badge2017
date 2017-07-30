#include "common.h"

/* USB_CTRL */
static inline void
usb_vreg_sense_disable(void)             { USB->CTRL &= ~USB_CTRL_VREGOSEN; }
static inline void
usb_vreg_sense_enable(void)              { USB->CTRL |= USB_CTRL_VREGOSEN; }
static inline void
usb_vreg_enable(void)                    { USB->CTRL &= ~USB_CTRL_VREGDIS; }
static inline void
usb_vreg_disable(void)                   { USB->CTRL |= USB_CTRL_VREGDIS; }

/* USB_STATUS */
static inline uint32_t
usb_status(void)                         { return USB->STATUS; }
static inline uint32_t
usb_low_energy_active(void)              { return USB->STATUS & USB_STATUS_LEMACTIVE; }
static inline uint32_t
usb_vreg_sense(void)                     { return USB->STATUS & USB_STATUS_VREGOS; }

/* USB_IF */
static inline uint32_t
usb_vreg_flags(void)                     { return USB->IF; }
static inline uint32_t
usb_vreg_flag_low(uint32_t v)            { return v & USB_IF_VREGOSL; }
static inline uint32_t
usb_vreg_flag_high(uint32_t v)           { return v & USB_IF_VREGOSH; }

/* USB_IFS */
static inline void
usb_vreg_flags_set(uint32_t v)           { USB->IFS = v; }
static inline void
usb_vreg_flag_low_set(void)              { USB->IFS = USB_IFS_VREGOSL; }
static inline void
usb_vreg_flag_high_set(void)             { USB->IFS = USB_IFS_VREGOSL; }

/* USB_IFC */
static inline void
usb_vreg_flags_clear(uint32_t v)         { USB->IFC = v; }
static inline void
usb_vreg_flag_low_clear(void)            { USB->IFC = USB_IFC_VREGOSL; }
static inline void
usb_vreg_flag_high_clear(void)           { USB->IFC = USB_IFC_VREGOSL; }

/* USB_IEN */
static inline void
usb_vreg_flags_disable(void)             { USB->IEN = 0; }
static inline void
usb_vreg_flags_enable(void)              { USB->IEN = USB_IEN_VREGOSL | USB_IEN_VREGOSH; }
static inline void
usb_vreg_flag_low_disable(void)          { USB->IEN &= ~USB_IEN_VREGOSL; }
static inline void
usb_vreg_flag_low_enable(void)           { USB->IEN |= USB_IEN_VREGOSL; }
static inline void
usb_vreg_flag_high_disable(void)         { USB->IEN &= ~USB_IEN_VREGOSL; }
static inline void
usb_vreg_flag_high_enable(void)          { USB->IEN |= USB_IEN_VREGOSL; }

/* USB_ROUTE */
static inline void
usb_pins_disable(void)                   { USB->ROUTE = 0; }
static inline void
usb_pins_enable(void)                    { USB->ROUTE = USB_ROUTE_PHYPEN; }
static inline void
usb_pins_pullup_enable(void)             { USB->ROUTE = USB_ROUTE_DMPUPEN | USB_ROUTE_PHYPEN; }

/* USB_GAHBCFG */
enum usb_ahb_config {
	USB_AHB_CONFIG_SINGLE_BURST      = USB_GAHBCFG_AHBSINGLE,
	USB_AHB_CONFIG_DMA_NOTIFY        = USB_GAHBCFG_NOTIALLDMAWRIT,
	USB_AHB_CONFIG_REMOTE_MEMORY     = USB_GAHBCFG_REMMEMSUPP,
	USB_AHB_CONFIG_TRIGGER_EMPTY     = USB_GAHBCFG_NPTXFEMPLVL,
	USB_AHB_CONFIG_DMA_ENABLE        = USB_GAHBCFG_DMAEN,
	USB_AHB_CONFIG_BURST_SINGLE      = USB_GAHBCFG_HBSTLEN_SINGLE,
	USB_AHB_CONFIG_BURST_INCR        = USB_GAHBCFG_HBSTLEN_INCR,
	USB_AHB_CONFIG_BURST_INCR4       = USB_GAHBCFG_HBSTLEN_INCR4,
	USB_AHB_CONFIG_BURST_INCR8       = USB_GAHBCFG_HBSTLEN_INCR8,
	USB_AHB_CONFIG_BURST_INCR16      = USB_GAHBCFG_HBSTLEN_INCR16,
	USB_AHB_CONFIG_INTERRUPTS_ENABLE = USB_GAHBCFG_GLBLINTRMSK,
};
static inline void
usb_ahb_config(uint32_t v)               { USB->GAHBCFG = v; }

/* USB_GUSBCFG */
static inline void
usb_config(void)                         { USB->GUSBCFG = USB_GUSBCFG_USBTRDTIM_DEFAULT; }

/* USB_GRSTCTL */
static inline uint32_t
usb_ahb_idle(void)                       { return USB->GRSTCTL & USB_GRSTCTL_AHBIDLE; }
static inline void
usb_fifo_flush(void)
{
	USB->GRSTCTL = USB_GRSTCTL_TXFNUM_FALL
	             | USB_GRSTCTL_TXFFLSH
	             | USB_GRSTCTL_RXFFLSH;
}
static inline uint32_t
usb_fifo_flushing(void)
{
	return USB->GRSTCTL & (USB_GRSTCTL_TXFFLSH | USB_GRSTCTL_RXFFLSH);
}
static inline void
usb_fifo_tx_flush(unsigned int v)
{
	USB->GRSTCTL = (v << _USB_GRSTCTL_TXFNUM_SHIFT) | USB_GRSTCTL_TXFFLSH;
}
static inline uint32_t
usb_fifo_tx_flushing(void)               { return USB->GRSTCTL & USB_GRSTCTL_TXFFLSH; }
static inline void
usb_fifo_rx_flush(void)                  { USB->GRSTCTL = USB_GRSTCTL_RXFFLSH; }
static inline uint32_t
usb_fifo_rx_flushing(void)               { return USB->GRSTCTL & USB_GRSTCTL_RXFFLSH; }
static inline void
usb_core_reset(void)                     { USB->GRSTCTL = USB_GRSTCTL_CSFTRST; }
static inline uint32_t
usb_core_resetting(void)                 { return USB->GRSTCTL & USB_GRSTCTL_CSFTRST; }

/* USB_GINTSTS */
static inline uint32_t
usb_flags(void)                          { return USB->GINTSTS; }
static inline void
usb_flags_clear(uint32_t v)
{
	USB->GINTSTS = v &
		( USB_GINTSTS_WKUPINT
		| USB_GINTSTS_RESETDET
		| USB_GINTSTS_FETSUSP
		| USB_GINTSTS_INCOMPLP
		| USB_GINTSTS_INCOMPISOIN
		| USB_GINTSTS_ISOOUTDROP
		| USB_GINTSTS_ENUMDONE
		| USB_GINTSTS_USBRST
		| USB_GINTSTS_USBSUSP
		| USB_GINTSTS_ERLYSUSP
		| USB_GINTSTS_SOF);
}
static inline uint32_t
usb_flag_wakeup(uint32_t v)              { return v & USB_GINTSTS_WKUPINT; }
static inline void
usb_flag_wakeup_clear(void)              { USB->GINTSTS = USB_GINTSTS_WKUPINT; }
static inline uint32_t
usb_flag_reset_detected(uint32_t v)      { return v & USB_GINTSTS_RESETDET; }
static inline void
usb_flag_reset_detected_clear(void)      { USB->GINTSTS = USB_GINTSTS_RESETDET; }
static inline uint32_t
usb_flag_fetch_suspended(uint32_t v)     { return v & USB_GINTSTS_FETSUSP; }
static inline void
usb_flag_fetch_suspended_clear(void)     { USB->GINTSTS = USB_GINTSTS_FETSUSP; }
static inline uint32_t
usb_flag_per_incomplete(uint32_t v)      { return v & USB_GINTSTS_INCOMPLP; }
static inline void
usb_flag_per_incomplete_clear(void)      { USB->GINTSTS = USB_GINTSTS_INCOMPLP; }
static inline uint32_t
usb_flag_iso_incomplete(uint32_t v)      { return v & USB_GINTSTS_INCOMPISOIN; }
static inline void
usb_flag_iso_incomplete_clear(void)      { USB->GINTSTS = USB_GINTSTS_INCOMPISOIN; }
static inline uint32_t
usb_flag_ep_out(uint32_t v)              { return v & USB_GINTSTS_OEPINT; }
static inline uint32_t
usb_flag_ep_in(uint32_t v)               { return v & USB_GINTSTS_IEPINT; }
static inline uint32_t
usb_flag_ep(uint32_t v)                  { return v & (USB_GINTSTS_OEPINT | USB_GINTSTS_IEPINT); }
static inline uint32_t
usb_flag_iso_dropped(uint32_t v)         { return v & USB_GINTSTS_ISOOUTDROP; }
static inline void
usb_flag_iso_dropped_clear(void)         { USB->GINTSTS = USB_GINTSTS_ISOOUTDROP; }
static inline uint32_t
usb_flag_enumdone(uint32_t v)            { return v & USB_GINTSTS_ENUMDONE; }
static inline void
usb_flag_enumdone_clear(void)            { USB->GINTSTS = USB_GINTSTS_ENUMDONE; }
static inline uint32_t
usb_flag_reset(uint32_t v)               { return v & USB_GINTSTS_USBRST; }
static inline void
usb_flag_reset_clear(void)               { USB->GINTSTS = USB_GINTSTS_USBRST; }
static inline uint32_t
usb_flag_suspend_usb(uint32_t v)         { return v & USB_GINTSTS_USBSUSP; }
static inline void
usb_flag_suspend_usb_clear(void)         { USB->GINTSTS = USB_GINTSTS_USBSUSP; }
static inline uint32_t
usb_flag_suspend_early(uint32_t v)       { return v & USB_GINTSTS_ERLYSUSP; }
static inline void
usb_flag_suspend_early_clear(void)       { USB->GINTSTS = USB_GINTSTS_ERLYSUSP; }
static inline uint32_t
usb_flag_suspend(uint32_t v)             { return v & (USB_GINTSTS_USBSUSP | USB_GINTSTS_ERLYSUSP); }
static inline void
usb_flag_suspend_clear(void)             { USB->GINTSTS = (USB_GINTSTS_USBSUSP | USB_GINTSTS_ERLYSUSP); }
static inline uint32_t
usb_flag_nak_out_effective(uint32_t v)   { return v & USB_GINTSTS_GOUTNAKEFF; }
static inline uint32_t
usb_flag_nak_in_effective(uint32_t v)    { return v & USB_GINTSTS_GINNAKEFF; }
static inline uint32_t
usb_flag_rxdata(uint32_t v)              { return v & USB_GINTSTS_RXFLVL; }
static inline uint32_t
usb_flag_sof(uint32_t v)                 { return v & USB_GINTSTS_SOF; }
static inline void
usb_flag_sof_clear(void)                 { USB->GINTSTS = USB_GINTSTS_SOF; }

/* USB_GINTMSK */
static inline void
usb_flags_enable(uint32_t v)             { USB->GINTMSK = v; }

/* USB_GRXSTSR */

/* USB_GRXSTSP */

/* USB_GRXFSIZ */

/* USB_GNPTXFSIZ */

/* USB_GDFIFOCFG */

/* USB_DIEPTXF1 */

/* USB_DIEPTXF2 */

/* USB_DIEPTXF3 */

/* USB_DCFG */
static inline void
usb_set_address(uint32_t addr)
{
	USB->DCFG = (USB->DCFG & ~_USB_DCFG_DEVADDR_MASK)
	          | (addr << _USB_DCFG_DEVADDR_SHIFT);
}

/* USB_DCTL */
static inline void
usb_global_out_nak_clear(void)           { USB->DCTL |= USB_DCTL_CGOUTNAK; }
static inline void
usb_global_out_nak_set(void)             { USB->DCTL |= USB_DCTL_SGOUTNAK; }
static inline void
usb_global_in_nak_clear(void)            { USB->DCTL |= USB_DCTL_CGNPINNAK; }
static inline void
usb_global_in_nak_set(void)              { USB->DCTL |= USB_DCTL_SGNPINNAK; }

#define USB_DCTL_WO_BITMASK (USB_DCTL_CGOUTNAK | USB_DCTL_SGOUTNAK | USB_DCTL_CGNPINNAK | USB_DCTL_SGNPINNAK)
static inline void
usb_connect(void)
{
	USB->DCTL &= ~(USB_DCTL_WO_BITMASK | USB_DCTL_SFTDISCON);
}
static inline void
usb_disconnect(void)
{
	USB->DCTL = (USB->DCTL & ~USB_DCTL_WO_BITMASK) | USB_DCTL_SFTDISCON;
}
static inline void
usb_remote_wakeup_clear(void)
{
	USB->DCTL &= ~(USB_DCTL_WO_BITMASK | USB_DCTL_RMTWKUPSIG);
}
static inline void
usb_remote_wakeup_set(void)
{
	USB->DCTL = (USB->DCTL & ~(USB_DCTL_WO_BITMASK | USB_DCTL_SFTDISCON))
	          | USB_DCTL_RMTWKUPSIG;
}

/* USB_DSTS */
static inline uint32_t
usb_enumerated_speed(void)
{
	return (USB->DSTS & _USB_DSTS_ENUMSPD_MASK) >> _USB_DSTS_ENUMSPD_SHIFT;
}

/* USB_DIEPMSK */

/* USB_DOEPMSK */

/* USB_DAINT */
static inline uint32_t
usb_ep_flags(void)                       { return USB->DAINT; }

/* USB_DAINTMSX */

/* USB_DIEPEMPMSK */

/* USB_DIEP0CTL */
static inline void
usb_ep0in_enable(void)
{
	USB->DIEP0CTL |= USB_DIEP0CTL_EPENA | USB_DIEP0CTL_CNAK;
}
static inline void
usb_ep0in_stall(void)
{
	USB->DIEP0CTL |= USB_DIEP0CTL_SNAK | USB_DIEP0CTL_STALL;
}

/* USB_DIEP0INT */
static inline uint32_t
usb_ep0in_flags(void)                    { return USB->DIEP0INT; }
static inline void
usb_ep0in_flags_clear(uint32_t v)
{
	USB->DIEP0INT = v &
		( USB_DIEP0INT_NAKINTRPT
		| USB_DIEP0INT_BBLEERR
		| USB_DIEP0INT_PKTDRPSTS
		| USB_DIEP0INT_INEPNAKEFF
		| USB_DIEP0INT_INTKNTXFEMP
		| USB_DIEP0INT_TIMEOUT
		| USB_DIEP0INT_AHBERR
		| USB_DIEP0INT_EPDISBLD
		| USB_DIEP0INT_XFERCOMPL);
}
static inline uint32_t
usb_ep0in_flag_nak(uint32_t v)           { return v & USB_DIEP0INT_NAKINTRPT; }
static inline uint32_t
usb_ep0in_flag_babble(uint32_t v)        { return v & USB_DIEP0INT_BBLEERR; }
static inline uint32_t
usb_ep0in_flag_packet_drop(uint32_t v)   { return v & USB_DIEP0INT_PKTDRPSTS; }
static inline uint32_t
usb_ep0in_flag_nak_effective(uint32_t v) { return v & USB_DIEP0INT_INEPNAKEFF; }
static inline uint32_t
usb_ep0in_flag_in_empty(uint32_t v)      { return v & USB_DIEP0INT_INTKNTXFEMP; }
static inline uint32_t
usb_ep0in_flag_timeout(uint32_t v)       { return v & USB_DIEP0INT_TIMEOUT; }
static inline uint32_t
usb_ep0in_flag_ahb_error(uint32_t v)     { return v & USB_DIEP0INT_AHBERR; }
static inline uint32_t
usb_ep0in_flag_disabled(uint32_t v)      { return v & USB_DIEP0INT_EPDISBLD; }
static inline uint32_t
usb_ep0in_flag_complete(uint32_t v)      { return v & USB_DIEP0INT_XFERCOMPL; }

/* USB_DIEP0TSIZ */
static inline void
usb_ep0in_transfer_size(uint32_t packets, uint32_t size)
{
	USB->DIEP0TSIZ = (packets << _USB_DIEP0TSIZ_PKTCNT_SHIFT) | size;
}
static inline uint32_t
usb_ep0in_bytes_left(void)
{
	return USB->DIEP0TSIZ & _USB_DIEP0TSIZ_XFERSIZE_MASK;
}

/* USB_DIEP0DMAADDR */
static inline const void *
usb_ep0in_dma_address(void)
{
	return (const void *)USB->DIEP0DMAADDR;
}
static inline void
usb_ep0in_dma_address_set(const void *addr)
{
	USB->DIEP0DMAADDR = (uint32_t)addr;
}

/* USB_DIEP0TXFSTS */

/* USB_DIEPx_CTL */
static inline void
usb_ep_in_enable(unsigned int i)
{
	USB->DIEP[i].CTL |= USB_DIEP_CTL_EPENA | USB_DIEP_CTL_CNAK;
}
static inline void
usb_ep_in_stall(unsigned int i)
{
	USB->DIEP[i].CTL |= USB_DIEP_CTL_SNAK | USB_DIEP_CTL_STALL;
}

/* USB_DIEPx_INT */
static inline uint32_t
usb_ep_in_flags(unsigned int i)
{
	return USB->DIEP[i].INT;
}
static inline void
usb_ep_in_flags_clear(unsigned int i, uint32_t v)
{
	USB->DIEP[i].INT = v &
		( USB_DIEP_INT_NAKINTRPT
		| USB_DIEP_INT_BBLEERR
		| USB_DIEP_INT_PKTDRPSTS
		| USB_DIEP_INT_INEPNAKEFF
		| USB_DIEP_INT_INTKNTXFEMP
		| USB_DIEP_INT_TIMEOUT
		| USB_DIEP_INT_AHBERR
		| USB_DIEP_INT_EPDISBLD
		| USB_DIEP_INT_XFERCOMPL);
}
static inline uint32_t
usb_ep_in_flag_nak(uint32_t v)           { return v & USB_DIEP_INT_NAKINTRPT; }
static inline uint32_t
usb_ep_in_flag_babble(uint32_t v)        { return v & USB_DIEP_INT_BBLEERR; }
static inline uint32_t
usb_ep_in_flag_packet_drop(uint32_t v)   { return v & USB_DIEP_INT_PKTDRPSTS; }
static inline uint32_t
usb_ep_in_flag_nak_effective(uint32_t v) { return v & USB_DIEP_INT_INEPNAKEFF; }
static inline uint32_t
usb_ep_in_flag_in_empty(uint32_t v)      { return v & USB_DIEP_INT_INTKNTXFEMP; }
static inline uint32_t
usb_ep_in_flag_timeout(uint32_t v)       { return v & USB_DIEP_INT_TIMEOUT; }
static inline uint32_t
usb_ep_in_flag_ahb_error(uint32_t v)     { return v & USB_DIEP_INT_AHBERR; }
static inline uint32_t
usb_ep_in_flag_disabled(uint32_t v)      { return v & USB_DIEP_INT_EPDISBLD; }
static inline uint32_t
usb_ep_in_flag_complete(uint32_t v)      { return v & USB_DIEP_INT_XFERCOMPL; }

/* USB_DIEPx_TSIZ */
static inline void
usb_ep_in_transfer_size(unsigned int i, uint32_t packets, uint32_t size)
{
	USB->DIEP[i].TSIZ = (packets << _USB_DIEP_TSIZ_PKTCNT_SHIFT)
	                  | size;
}
static inline uint32_t
usb_ep_in_bytes_left(unsigned int i)
{
	return USB->DIEP[i].TSIZ & _USB_DIEP_TSIZ_XFERSIZE_MASK;
}


/* USB_DIEPx_DMAADDR */
static inline uint32_t
usb_ep_in_dma_address(unsigned int i)
{
	return USB->DIEP[i].DMAADDR;
}
static inline void
usb_ep_in_dma_address_set(unsigned int i, const void *addr)
{
	USB->DIEP[i].DMAADDR = (uint32_t)addr;
}

/* USB_DIEPx_TXFSTS */

/* USB_DOEP0CTL */
static inline void
usb_ep0out_enable(void)
{
	USB->DOEP0CTL |= USB_DOEP0CTL_EPENA | USB_DOEP0CTL_CNAK;
}
static inline void
usb_ep0out_enable_setup(void)
{
	USB->DOEP0CTL |= USB_DOEP0CTL_EPENA | USB_DOEP0CTL_SNAK | USB_DOEP0CTL_STALL;
}

/* USB_DOEP0INT */
static inline uint32_t
usb_ep0out_flags(void)                   { return USB->DOEP0INT; }
static inline void
usb_ep0out_flags_clear(uint32_t v)
{
	USB->DOEP0INT = v &
		( USB_DOEP0INT_NAKINTRPT
		| USB_DOEP0INT_BBLEERR
		| USB_DOEP0INT_PKTDRPSTS
		| USB_DOEP0INT_BACK2BACKSETUP
		| USB_DOEP0INT_STSPHSERCVD
		| USB_DOEP0INT_OUTTKNEPDIS
		| USB_DOEP0INT_SETUP
		| USB_DOEP0INT_AHBERR
		| USB_DOEP0INT_EPDISBLD
		| USB_DOEP0INT_XFERCOMPL);
}
static inline uint32_t
usb_ep0out_flag_nak(uint32_t v)          { return v & USB_DOEP0INT_NAKINTRPT; }
static inline void
usb_ep0out_flag_nak_clear(void)          { USB->DOEP0INT = USB_DOEP0INT_NAKINTRPT; }
static inline uint32_t
usb_ep0out_flag_babble(uint32_t v)       { return v & USB_DOEP0INT_BBLEERR; }
static inline void
usb_ep0out_flag_babble_clear(void)       { USB->DOEP0INT = USB_DOEP0INT_BBLEERR; }
static inline uint32_t
usb_ep0out_flag_packet_drop(uint32_t v)  { return v & USB_DOEP0INT_PKTDRPSTS; }
static inline void
usb_ep0out_flag_packet_drop_clear(void)  { USB->DOEP0INT = USB_DOEP0INT_PKTDRPSTS; }
static inline uint32_t
usb_ep0out_flag_setup_b2b(uint32_t v)    { return v & USB_DOEP0INT_BACK2BACKSETUP; }
static inline void
usb_ep0out_flag_setup_b2b_clear(void)    { USB->DOEP0INT = USB_DOEP0INT_BACK2BACKSETUP; }
static inline uint32_t
usb_ep0out_flag_out_disabled(uint32_t v) { return v & USB_DOEP0INT_OUTTKNEPDIS; }
static inline void
usb_ep0out_flag_out_disabled_clear(void) { USB->DOEP0INT = USB_DOEP0INT_OUTTKNEPDIS; }
static inline uint32_t
usb_ep0out_flag_setup(uint32_t v)        { return v & USB_DOEP0INT_SETUP; }
static inline void
usb_ep0out_flag_setup_clear(void)        { USB->DOEP0INT = USB_DOEP0INT_SETUP; }
static inline uint32_t
usb_ep0out_flag_ahb_error(uint32_t v)    { return v & USB_DOEP0INT_AHBERR; }
static inline void
usb_ep0out_flag_ahb_error_clear(void)    { USB->DOEP0INT = USB_DOEP0INT_AHBERR; }
static inline uint32_t
usb_ep0out_flag_disabled(uint32_t v)     { return v & USB_DOEP0INT_EPDISBLD; }
static inline void
usb_ep0out_flag_disabled_clear(void)     { USB->DOEP0INT = USB_DOEP0INT_EPDISBLD; }
static inline uint32_t
usb_ep0out_flag_complete(uint32_t v)     { return v & USB_DOEP0INT_XFERCOMPL; }
static inline void
usb_ep0out_flag_complete_clear(void)     { USB->DOEP0INT = USB_DOEP0INT_XFERCOMPL; }

/* USB_DOEP0TSIZ */
static inline void
usb_ep0out_transfer_size(uint32_t packets, uint32_t size)
{
	USB->DOEP0TSIZ = (3 << _USB_DOEP0TSIZ_SUPCNT_SHIFT)
	               | (packets << _USB_DOEP0TSIZ_PKTCNT_SHIFT)
	               | size;
}
static inline uint32_t
usb_ep0out_bytes_left(void)
{
	return USB->DOEP0TSIZ & _USB_DOEP0TSIZ_XFERSIZE_MASK;
}

/* USB_DOEP0DMAADDR */
static inline void *
usb_ep0out_dma_address(void)
{
	return (void *)USB->DOEP0DMAADDR;
}
static inline void
usb_ep0out_dma_address_set(void *addr)
{
	USB->DOEP0DMAADDR = (uint32_t)addr;
}

/* USB_DOEPx_CTL */
static inline void
usb_ep_out_enable(unsigned int i)
{
	USB->DOEP[i].CTL |= USB_DOEP_CTL_EPENA | USB_DOEP_CTL_CNAK;
}

/* USB_DOEPx_INT */
static inline uint32_t
usb_ep_out_flags(unsigned int i)
{
	return USB->DOEP[i].INT;
}
static inline void
usb_ep_out_flags_clear(unsigned int i, uint32_t v)
{
	USB->DOEP[i].INT = v &
		( USB_DOEP_INT_NAKINTRPT
		| USB_DOEP_INT_BBLEERR
		| USB_DOEP_INT_PKTDRPSTS
		| USB_DOEP_INT_BACK2BACKSETUP
		| USB_DOEP_INT_STSPHSERCVD
		| USB_DOEP_INT_OUTTKNEPDIS
		| USB_DOEP_INT_SETUP
		| USB_DOEP_INT_AHBERR
		| USB_DOEP_INT_EPDISBLD
		| USB_DOEP_INT_XFERCOMPL);
}
static inline uint32_t
usb_ep_out_flag_nak(uint32_t v)                    { return v & USB_DOEP_INT_NAKINTRPT; }
static inline void
usb_ep_out_flag_nak_clear(unsigned int i)          { USB->DOEP[i].INT = USB_DOEP_INT_NAKINTRPT; }
static inline uint32_t
usb_ep_out_flag_babble(uint32_t v)                 { return v & USB_DOEP_INT_BBLEERR; }
static inline void
usb_ep_out_flag_babble_clear(unsigned int i)       { USB->DOEP[i].INT = USB_DOEP_INT_BBLEERR; }
static inline uint32_t
usb_ep_out_flag_packet_drop(uint32_t v)            { return v & USB_DOEP_INT_PKTDRPSTS; }
static inline void
usb_ep_out_flag_packet_drop_clear(unsigned int i)  { USB->DOEP[i].INT = USB_DOEP_INT_PKTDRPSTS; }
static inline uint32_t
usb_ep_out_flag_setup_b2b(uint32_t v)              { return v & USB_DOEP_INT_BACK2BACKSETUP; }
static inline void
usb_ep_out_flag_setup_b2b_clear(unsigned int i)    { USB->DOEP[i].INT = USB_DOEP_INT_BACK2BACKSETUP; }
static inline uint32_t
usb_ep_out_flag_out_disabled(uint32_t v)           { return v & USB_DOEP_INT_OUTTKNEPDIS; }
static inline void
usb_ep_out_flag_out_disabled_clear(unsigned int i) { USB->DOEP[i].INT = USB_DOEP_INT_OUTTKNEPDIS; }
static inline uint32_t
usb_ep_out_flag_setup(uint32_t v)                  { return v & USB_DOEP_INT_SETUP; }
static inline void
usb_ep_out_flag_setup_clear(unsigned int i)        { USB->DOEP[i].INT = USB_DOEP_INT_SETUP; }
static inline uint32_t
usb_ep_out_flag_ahb_error(uint32_t v)              { return v & USB_DOEP_INT_AHBERR; }
static inline void
usb_ep_out_flag_ahb_error_clear(unsigned int i)    { USB->DOEP[i].INT = USB_DOEP_INT_AHBERR; }
static inline uint32_t
usb_ep_out_flag_disabled(uint32_t v)               { return v & USB_DOEP_INT_EPDISBLD; }
static inline void
usb_ep_out_flag_disabled_clear(unsigned int i)     { USB->DOEP[i].INT = USB_DOEP_INT_EPDISBLD; }
static inline uint32_t
usb_ep_out_flag_complete(uint32_t v)               { return v & USB_DOEP_INT_XFERCOMPL; }
static inline void
usb_ep_out_flag_complete_clear(unsigned int i)     { USB->DOEP[i].INT = USB_DOEP_INT_XFERCOMPL; }

/* USB_DOEPx_TSIZ */
static inline void
usb_ep_out_transfer_size(unsigned int i, uint32_t packets, uint32_t size)
{
	USB->DOEP[i].TSIZ = (packets << _USB_DOEP_TSIZ_PKTCNT_SHIFT)
	                  | size;
}
static inline uint32_t
usb_ep_out_bytes_left(unsigned int i)
{
	return USB->DOEP[i].TSIZ & _USB_DOEP_TSIZ_XFERSIZE_MASK;
}

/* USB_DOEPx_DMAADDR */
static inline uint32_t
usb_ep_out_dma_address(unsigned int i)
{
	return USB->DOEP[i].DMAADDR;
}
static inline void
usb_ep_out_dma_address_set(unsigned int i, void *addr)
{
	USB->DOEP[i].DMAADDR = (uint32_t)addr;
}

/* USB_PCGCCTL */
static inline uint32_t
usb_phy_sleeping(void)                   { return USB->PCGCCTL & USB_PCGCCTL_PHYSLEEP; }
static inline void
usb_phy_stop(void)                       { USB->PCGCCTL = USB_PCGCCTL_STOPPCLK; }
static inline void
usb_phy_start(void)                      { USB->PCGCCTL = 0; }

/* USB_FIFO0Dx */

/* USB_FIFO1Dx */

/* USB_FIFO2Dx */

/* USB_FIFO3Dx */

/* USB_FIFORAMx */
