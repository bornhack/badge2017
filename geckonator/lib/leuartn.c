/* LEUARTn_CTRL */
static inline void
leuartn_(config, uint32_t v)               { LEUARTn->CTRL = v; }

/* LEUARTn_CMD */
static inline void
leuartn_(clear_rx, void)                   { LEUARTn->CMD = LEUART_CMD_CLEARRX; }
static inline void
leuartn_(clear_tx, void)                   { LEUARTn->CMD = LEUART_CMD_CLEARTX; }
static inline void
leuartn_(receiver_block_disable, void)     { LEUARTn->CMD = LEUART_CMD_RXBLOCKDIS; }
static inline void
leuartn_(receiver_block_enable, void)      { LEUARTn->CMD = LEUART_CMD_RXBLOCKEN; }
static inline void
leuartn_(tx_disable, void)                 { LEUARTn->CMD = LEUART_CMD_TXDIS; }
static inline void
leuartn_(tx_enable, void)                  { LEUARTn->CMD = LEUART_CMD_TXEN; }
static inline void
leuartn_(rx_disable, void)                 { LEUARTn->CMD = LEUART_CMD_RXDIS; }
static inline void
leuartn_(rx_enable, void)                  { LEUARTn->CMD = LEUART_CMD_RXEN; }
static inline void
leuartn_(rxtx_disable, void)               { LEUARTn->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS; }
static inline void
leuartn_(rxtx_enable, void)                { LEUARTn->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN; }

/* LEUARTn_STATUS */
static inline uint32_t
leuartn_(rxdata_valid, void)               { return LEUARTn->STATUS & LEUART_STATUS_RXDATAV; }
static inline uint32_t
leuartn_(tx_buffer_level, void)            { return LEUARTn->STATUS & LEUART_STATUS_TXBL; }
static inline uint32_t
leuartn_(tx_complete, void)                { return LEUARTn->STATUS & LEUART_STATUS_TXC; }
static inline uint32_t
leuartn_(block_incoming_data, void)        { return LEUARTn->STATUS & LEUART_STATUS_RXBLOCK; }
static inline uint32_t
leuartn_(transmitter_enabled, void)        { return LEUARTn->STATUS & LEUART_STATUS_TXENS; }
static inline uint32_t
leuartn_(receiver_enabled, void)           { return LEUARTn->STATUS & LEUART_STATUS_RXENS; }

/* LEUARTn_CLKDIV */
static inline void
leuartn_(clock_div, uint32_t v)            { LEUARTn->CLKDIV = v; }

/* LEUARTn_STARTFRAME */
static inline void
leuartn_(start_frame, uint32_t v)          { LEUARTn->STARTFRAME = v; }

/* LEUARTn_SIGFRAME */
static inline void
leuartn_(signal_frame, uint32_t v)         { LEUARTn->SIGFRAME = v; }

/* LEUARTn_RXDATA */
static inline uint8_t
leuartn_(rxdata, void)                     { return LEUARTn->RXDATA; }

/* LEUARTn_TXDATA */
static inline void
leuartn_(txdata, uint8_t v)                { LEUARTn->TXDATA = v; }

/* LEUARTn_IF */
static inline uint32_t
leuartn_(flags, void)                      { return LEUARTn->IF; }
static inline uint32_t
leuartn_(flag_signal_frame, uint32_t v)    { return v & LEUART_IF_SIGF; }
static inline uint32_t
leuartn_(flag_start_frame, uint32_t v)     { return v & LEUART_IF_STARTF; }
static inline uint32_t
leuartn_(flag_mp_address, uint32_t v)      { return v & LEUART_IF_MPAF; }
static inline uint32_t
leuartn_(flag_framing_error, uint32_t v)   { return v & LEUART_IF_FERR; }
static inline uint32_t
leuartn_(flag_parity_error, uint32_t v)    { return v & LEUART_IF_PERR; }
static inline uint32_t
leuartn_(flag_tx_overflow, uint32_t v)     { return v & LEUART_IF_TXOF; }
static inline uint32_t
leuartn_(flag_rx_underflow, uint32_t v)    { return v & LEUART_IF_RXUF; }
static inline uint32_t
leuartn_(flag_rx_overflow, uint32_t v)     { return v & LEUART_IF_RXOF; }
static inline uint32_t
leuartn_(flag_rx_data_valid, uint32_t v)   { return v & LEUART_IF_RXDATAV; }
static inline uint32_t
leuartn_(flag_tx_buffer_level, uint32_t v) { return v & LEUART_IF_TXBL; }
static inline uint32_t
leuartn_(flag_tx_complete, uint32_t v)     { return v & LEUART_IF_TXC; }

/* LEUARTn_IFS */
static inline void
leuartn_(flags_set, uint32_t v)            { LEUARTn->IFS = v; }
static inline void
leuartn_(flag_signal_frame_set, void)      { LEUARTn->IFS = LEUART_IFS_SIGF; }
static inline void
leuartn_(flag_start_frame_set, void)       { LEUARTn->IFS = LEUART_IFS_STARTF; }
static inline void
leuartn_(flag_mp_address_set, void)        { LEUARTn->IFS = LEUART_IFS_MPAF; }
static inline void
leuartn_(flag_framing_error_set, void)     { LEUARTn->IFS = LEUART_IFS_FERR; }
static inline void
leuartn_(flag_parity_error_set, void)      { LEUARTn->IFS = LEUART_IFS_PERR; }
static inline void
leuartn_(flag_tx_overflow_set, void)       { LEUARTn->IFS = LEUART_IFS_TXOF; }
static inline void
leuartn_(flag_rx_underflow_set, void)      { LEUARTn->IFS = LEUART_IFS_RXUF; }
static inline void
leuartn_(flag_rx_overflow_set, void)       { LEUARTn->IFS = LEUART_IFS_RXOF; }
static inline void
leuartn_(flag_tx_complete_set, void)       { LEUARTn->IFS = LEUART_IFS_TXC; }

/* LEUARTn_IFC */
static inline void
leuartn_(flags_clear, uint32_t v)
{
	LEUARTn->IFC = v &
		( LEUART_IFC_SIGF
		| LEUART_IFC_STARTF
		| LEUART_IFC_MPAF
		| LEUART_IFC_FERR
		| LEUART_IFC_PERR
		| LEUART_IFC_TXOF
		| LEUART_IFC_RXUF
		| LEUART_IFC_RXOF
		| LEUART_IFC_TXC);
}
static inline void
leuartn_(flags_clear_all, void)
{
	LEUARTn->IFC =
		( LEUART_IFC_SIGF
		| LEUART_IFC_STARTF
		| LEUART_IFC_MPAF
		| LEUART_IFC_FERR
		| LEUART_IFC_PERR
		| LEUART_IFC_TXOF
		| LEUART_IFC_RXUF
		| LEUART_IFC_RXOF
		| LEUART_IFC_TXC);
}
static inline void
leuartn_(flag_signal_frame_clear, void)    { LEUARTn->IFC = LEUART_IFC_SIGF; }
static inline void
leuartn_(flag_start_frame_clear, void)     { LEUARTn->IFC = LEUART_IFC_STARTF; }
static inline void
leuartn_(flag_mp_address_clear, void)      { LEUARTn->IFC = LEUART_IFC_MPAF; }
static inline void
leuartn_(flag_framing_error_clear, void)   { LEUARTn->IFC = LEUART_IFC_FERR; }
static inline void
leuartn_(flag_parity_error_clear, void)    { LEUARTn->IFC = LEUART_IFC_PERR; }
static inline void
leuartn_(flag_tx_overflow_clear, void)     { LEUARTn->IFC = LEUART_IFC_TXOF; }
static inline void
leuartn_(flag_rx_underflow_clear, void)    { LEUARTn->IFC = LEUART_IFC_RXUF; }
static inline void
leuartn_(flag_rx_overflow_clear, void)     { LEUARTn->IFC = LEUART_IFC_RXOF; }
static inline void
leuartn_(flag_tx_complete_clear, void)     { LEUARTn->IFC = LEUART_IFC_TXC; }

/* LEUARTn_IEN */
static inline void
leuartn_(flag_signal_frame_disable, void)    { LEUARTn->IEN &= ~LEUART_IEN_SIGF; }
static inline void
leuartn_(flag_signal_frame_enable, void)     { LEUARTn->IEN |= LEUART_IEN_SIGF; }
static inline void
leuartn_(flag_start_frame_disable, void)     { LEUARTn->IEN &= ~LEUART_IEN_STARTF; }
static inline void
leuartn_(flag_start_frame_enable, void)      { LEUARTn->IEN |= LEUART_IEN_STARTF; }
static inline void
leuartn_(flag_mp_address_disable, void)      { LEUARTn->IEN &= ~LEUART_IEN_MPAF; }
static inline void
leuartn_(flag_mp_address_enable, void)       { LEUARTn->IEN |= LEUART_IEN_MPAF; }
static inline void
leuartn_(flag_framing_error_disable, void)   { LEUARTn->IEN &= ~LEUART_IEN_FERR; }
static inline void
leuartn_(flag_framing_error_enable, void)    { LEUARTn->IEN |= LEUART_IEN_FERR; }
static inline void
leuartn_(flag_parity_error_disable, void)    { LEUARTn->IEN &= ~LEUART_IEN_PERR; }
static inline void
leuartn_(flag_parity_error_enable, void)     { LEUARTn->IEN |= LEUART_IEN_PERR; }
static inline void
leuartn_(flag_tx_overflow_disable, void)     { LEUARTn->IEN &= ~LEUART_IEN_TXOF; }
static inline void
leuartn_(flag_tx_overflow_enable, void)      { LEUARTn->IEN |= LEUART_IEN_TXOF; }
static inline void
leuartn_(flag_rx_underflow_disable, void)    { LEUARTn->IEN &= ~LEUART_IEN_RXUF; }
static inline void
leuartn_(flag_rx_underflow_enable, void)     { LEUARTn->IEN |= LEUART_IEN_RXUF; }
static inline void
leuartn_(flag_rx_overflow_disable, void)     { LEUARTn->IEN &= ~LEUART_IEN_RXOF; }
static inline void
leuartn_(flag_rx_overflow_enable, void)      { LEUARTn->IEN |= LEUART_IEN_RXOF; }
static inline void
leuartn_(flag_rx_data_valid_disable, void)   { LEUARTn->IEN &= ~LEUART_IEN_RXDATAV; }
static inline void
leuartn_(flag_rx_data_valid_enable, void)    { LEUARTn->IEN |= LEUART_IEN_RXDATAV; }
static inline void
leuartn_(flag_tx_buffer_level_disable, void) { LEUARTn->IEN &= ~LEUART_IEN_TXBL; }
static inline void
leuartn_(flag_tx_buffer_level_enable, void)  { LEUARTn->IEN |= LEUART_IEN_TXBL; }
static inline void
leuartn_(flag_tx_complete_disable, void)     { LEUARTn->IEN &= ~LEUART_IEN_TXC; }
static inline void
leuartn_(flag_tx_complete_enable, void)      { LEUARTn->IEN |= LEUART_IEN_TXC; }

/* LEUARTn_FREEZE */
static inline void
leuartn_(freeze, void)                     { LEUARTn->FREEZE = LEUART_FREEZE_REGFREEZE_FREEZE; }
static inline void
leuartn_(update, void)                     { LEUARTn->FREEZE = LEUART_FREEZE_REGFREEZE_UPDATE; }

/* LEUARTn_SYNCBUSY */
static inline uint32_t
leuartn_(syncbusy, void)                   { return LEUARTn->SYNCBUSY; }

/* LEUARTn_ROUTE */
static inline void
leuartn_(pins, uint32_t v)                 { LEUARTn->ROUTE = v; }
