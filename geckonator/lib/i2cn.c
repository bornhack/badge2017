/* I2Cn_CTRL */
static inline void
i2cn_(config, uint32_t v)                   { I2Cn->CTRL = v; }
static inline void
i2cn_(disable, void)                        { I2Cn->CTRL &= ~I2C_CTRL_EN; }
static inline void
i2cn_(enable, void)                         { I2Cn->CTRL |= I2C_CTRL_EN; }

/* I2Cn_CMD */
static inline void
i2cn_(clear_pending, void)                  { I2Cn->CMD = I2C_CMD_CLEARPC; }
static inline void
i2cn_(clear_tx, void)                       { I2Cn->CMD = I2C_CMD_CLEARTX; }
static inline void
i2cn_(abort, void)                          { I2Cn->CMD = I2C_CMD_ABORT; }
static inline void
i2cn_(continue, void)                       { I2Cn->CMD = I2C_CMD_CONT; }
static inline void
i2cn_(nack, void)                           { I2Cn->CMD = I2C_CMD_NACK; }
static inline void
i2cn_(ack, void)                            { I2Cn->CMD = I2C_CMD_ACK; }
static inline void
i2cn_(stop, void)                           { I2Cn->CMD = I2C_CMD_STOP; }
static inline void
i2cn_(start, void)                          { I2Cn->CMD = I2C_CMD_START; }

/* I2Cn_STATE */
static inline uint32_t
i2cn_(state, void)                          { return I2Cn->STATE >> 5; }
static inline uint32_t
i2cn_(bus_held, void)                       { return I2Cn->STATE & I2C_STATE_BUSHOLD; }
static inline uint32_t
i2cn_(nacked, void)                         { return I2Cn->STATE & I2C_STATE_NACKED; }
static inline uint32_t
i2cn_(transmitter, void)                    { return I2Cn->STATE & I2C_STATE_TRANSMITTER; }
static inline uint32_t
i2cn_(master, void)                         { return I2Cn->STATE & I2C_STATE_MASTER; }
static inline uint32_t
i2cn_(busy, void)                           { return I2Cn->STATE & I2C_STATE_BUSY; }

/* I2Cn_STATUS */
static inline uint32_t
i2cn_(status, void)                         { return I2Cn->STATUS; }
static inline uint32_t
i2cn_(rxdata_valid, void)                   { return I2Cn->STATUS & I2C_STATUS_RXDATAV; }
static inline uint32_t
i2cn_(tx_buffer_level, void)                { return I2Cn->STATUS & I2C_STATUS_TXBL; }
static inline uint32_t
i2cn_(tx_complete, void)                    { return I2Cn->STATUS & I2C_STATUS_TXC; }
static inline uint32_t
i2cn_(pending_abort, void)                  { return I2Cn->STATUS & I2C_STATUS_PABORT; }
static inline uint32_t
i2cn_(pending_continue, void)               { return I2Cn->STATUS & I2C_STATUS_PCONT; }
static inline uint32_t
i2cn_(pending_nack, void)                   { return I2Cn->STATUS & I2C_STATUS_PNACK; }
static inline uint32_t
i2cn_(pending_ack, void)                    { return I2Cn->STATUS & I2C_STATUS_PACK; }
static inline uint32_t
i2cn_(pending_stop, void)                   { return I2Cn->STATUS & I2C_STATUS_PSTOP; }
static inline uint32_t
i2cn_(pending_start, void)                  { return I2Cn->STATUS & I2C_STATUS_PSTART; }

/* I2Cn_CLKDIV */
static inline void
i2cn_(clock_div, uint32_t v)                { I2Cn->CLKDIV = v; }

/* I2Cn_SADDR */
static inline void
i2cn_(slave_address_set, uint32_t v)        { I2Cn->SADDR = v; }
static inline uint32_t
i2cn_(slave_address, void)                  { return I2Cn->SADDR; }

/* I2Cn_SADDRMASK */
static inline void
i2cn_(slave_address_mask_set, uint32_t v)   { I2Cn->SADDRMASK = v; }
static inline uint32_t
i2cn_(slave_address_mask, void)             { return I2Cn->SADDRMASK; }

/* I2Cn_RXDATA */
static inline uint8_t
i2cn_(rxdata, void)                         { return I2Cn->RXDATA; }

/* I2Cn_RXDATAP */
static inline uint8_t
i2cn_(rxdata_peek, void)                    { return I2Cn->RXDATAP; }

/* I2Cn_TXDATA */
static inline void
i2cn_(txdata, uint8_t v)                    { I2Cn->TXDATA = v; }

/* I2Cn_IF */
static inline uint32_t
i2cn_(flags, void)                          { return I2Cn->IF; }
static inline uint32_t
i2cn_(flag_slave_stop, uint32_t v)          { return v & I2C_IF_SSTOP; }
static inline uint32_t
i2cn_(flag_clock_low_timeout, uint32_t v)   { return v & I2C_IF_CLTO; }
static inline uint32_t
i2cn_(flag_bus_idle_timeout, uint32_t v)    { return v & I2C_IF_BITO; }
static inline uint32_t
i2cn_(flag_rx_underflow, uint32_t v)        { return v & I2C_IF_RXUF; }
static inline uint32_t
i2cn_(flag_tx_overflow, uint32_t v)         { return v & I2C_IF_TXOF; }
static inline uint32_t
i2cn_(flag_bus_held, uint32_t v)            { return v & I2C_IF_BUSHOLD; }
static inline uint32_t
i2cn_(flag_bus_error, uint32_t v)           { return v & I2C_IF_BUSERR; }
static inline uint32_t
i2cn_(flag_arbitration_lost, uint32_t v)    { return v & I2C_IF_ARBLOST; }
static inline uint32_t
i2cn_(flag_master_stop, uint32_t v)         { return v & I2C_IF_MSTOP; }
static inline uint32_t
i2cn_(flag_nack, uint32_t v)                { return v & I2C_IF_NACK; }
static inline uint32_t
i2cn_(flag_ack, uint32_t v)                 { return v & I2C_IF_ACK; }
static inline uint32_t
i2cn_(flag_rx_data_valid, uint32_t v)       { return v & I2C_IF_RXDATAV; }
static inline uint32_t
i2cn_(flag_tx_buffer_level, uint32_t v)     { return v & I2C_IF_TXBL; }
static inline uint32_t
i2cn_(flag_tx_complete, uint32_t v)         { return v & I2C_IF_TXC; }
static inline uint32_t
i2cn_(flag_address, uint32_t v)             { return v & I2C_IF_ADDR; }
static inline uint32_t
i2cn_(flag_repeated_start, uint32_t v)      { return v & I2C_IF_RSTART; }
static inline uint32_t
i2cn_(flag_start, uint32_t v)               { return v & I2C_IF_START; }

/* I2Cn_IFS */
static inline void
i2cn_(flags_set, uint32_t v)                { I2Cn->IFS = v; }
static inline void
i2cn_(flag_slave_stop_set, void)            { I2Cn->IFS = I2C_IFS_SSTOP; }
static inline void
i2cn_(flag_clock_low_timeout_set, void)     { I2Cn->IFS = I2C_IFS_CLTO; }
static inline void
i2cn_(flag_bus_idle_timeout_set, void)      { I2Cn->IFS = I2C_IFS_BITO; }
static inline void
i2cn_(flag_rx_underflow_set, void)          { I2Cn->IFS = I2C_IFS_RXUF; }
static inline void
i2cn_(flag_tx_overflow_set, void)           { I2Cn->IFS = I2C_IFS_TXOF; }
static inline void
i2cn_(flag_bus_held_set, void)              { I2Cn->IFS = I2C_IFS_BUSHOLD; }
static inline void
i2cn_(flag_bus_error_set, void)             { I2Cn->IFS = I2C_IFS_BUSERR; }
static inline void
i2cn_(flag_arbitration_lost_set, void)      { I2Cn->IFS = I2C_IFS_ARBLOST; }
static inline void
i2cn_(flag_master_stop_set, void)           { I2Cn->IFS = I2C_IFS_MSTOP; }
static inline void
i2cn_(flag_nack_set, void)                  { I2Cn->IFS = I2C_IFS_NACK; }
static inline void
i2cn_(flag_ack_set, void)                   { I2Cn->IFS = I2C_IFS_ACK; }
static inline void
i2cn_(flag_tx_complete_set, void)           { I2Cn->IFS = I2C_IFS_TXC; }
static inline void
i2cn_(flag_address_set, void)               { I2Cn->IFS = I2C_IFS_ADDR; }
static inline void
i2cn_(flag_repeated_start_set, void)        { I2Cn->IFS = I2C_IFS_RSTART; }
static inline void
i2cn_(flag_start_set, void)                 { I2Cn->IFS = I2C_IFS_START; }

/* I2Cn_IFC */
static inline void
i2cn_(flags_clear, uint32_t v)
{
	I2Cn->IFC = v &
		( I2C_IFC_SSTOP
		| I2C_IFC_CLTO
		| I2C_IFC_BITO
		| I2C_IFC_RXUF
		| I2C_IFC_TXOF
		| I2C_IFC_BUSHOLD
		| I2C_IFC_BUSERR
		| I2C_IFC_ARBLOST
		| I2C_IFC_MSTOP
		| I2C_IFC_NACK
		| I2C_IFC_ACK
		| I2C_IFC_TXC
		| I2C_IFC_ADDR
		| I2C_IFC_RSTART
		| I2C_IFC_START);
}
static inline void
i2cn_(flags_clear_all, void)
{
	I2Cn->IFC =
		( I2C_IFC_SSTOP
		| I2C_IFC_CLTO
		| I2C_IFC_BITO
		| I2C_IFC_RXUF
		| I2C_IFC_TXOF
		| I2C_IFC_BUSHOLD
		| I2C_IFC_BUSERR
		| I2C_IFC_ARBLOST
		| I2C_IFC_MSTOP
		| I2C_IFC_NACK
		| I2C_IFC_ACK
		| I2C_IFC_TXC
		| I2C_IFC_ADDR
		| I2C_IFC_RSTART
		| I2C_IFC_START);
}
static inline void
i2cn_(flag_slave_stop_clear, void)          { I2Cn->IFC = I2C_IFC_SSTOP; }
static inline void
i2cn_(flag_clock_low_timeout_clear, void)   { I2Cn->IFC = I2C_IFC_CLTO; }
static inline void
i2cn_(flag_bus_idle_timeout_clear, void)    { I2Cn->IFC = I2C_IFC_BITO; }
static inline void
i2cn_(flag_rx_underflow_clear, void)        { I2Cn->IFC = I2C_IFC_RXUF; }
static inline void
i2cn_(flag_tx_overflow_clear, void)         { I2Cn->IFC = I2C_IFC_TXOF; }
static inline void
i2cn_(flag_bus_held_clear, void)            { I2Cn->IFC = I2C_IFC_BUSHOLD; }
static inline void
i2cn_(flag_bus_error_clear, void)           { I2Cn->IFC = I2C_IFC_BUSERR; }
static inline void
i2cn_(flag_arbitration_lost_clear, void)    { I2Cn->IFC = I2C_IFC_ARBLOST; }
static inline void
i2cn_(flag_master_stop_clear, void)         { I2Cn->IFC = I2C_IFC_MSTOP; }
static inline void
i2cn_(flag_nack_clear, void)                { I2Cn->IFC = I2C_IFC_NACK; }
static inline void
i2cn_(flag_ack_clear, void)                 { I2Cn->IFC = I2C_IFC_ACK; }
static inline void
i2cn_(flag_tx_complete_clear, void)         { I2Cn->IFC = I2C_IFC_TXC; }
static inline void
i2cn_(flag_address_clear, void)             { I2Cn->IFC = I2C_IFC_ADDR; }
static inline void
i2cn_(flag_repeated_start_clear, void)      { I2Cn->IFC = I2C_IFC_RSTART; }
static inline void
i2cn_(flag_start_clear, void)               { I2Cn->IFC = I2C_IFC_START; }

/* I2Cn_IEN */
static inline void
i2cn_(flags_enable, uint32_t v)             { I2Cn->IEN = v; }
static inline void
i2cn_(flag_slave_stop_disable, void)        { I2Cn->IEN &= ~I2C_IEN_SSTOP; }
static inline void
i2cn_(flag_slave_stop_enable, void)         { I2Cn->IEN |= I2C_IEN_SSTOP; }
static inline void
i2cn_(flag_clock_low_timeout_disable, void) { I2Cn->IEN &= ~I2C_IEN_CLTO; }
static inline void
i2cn_(flag_clock_low_timeout_enable, void)  { I2Cn->IEN |= I2C_IEN_CLTO; }
static inline void
i2cn_(flag_bus_idle_timeout_disable, void)  { I2Cn->IEN &= ~I2C_IEN_BITO; }
static inline void
i2cn_(flag_bus_idle_timeout_enable, void)   { I2Cn->IEN |= I2C_IEN_BITO; }
static inline void
i2cn_(flag_rx_underflow_disable, void)      { I2Cn->IEN &= ~I2C_IEN_RXUF; }
static inline void
i2cn_(flag_rx_underflow_enable, void)       { I2Cn->IEN |= I2C_IEN_RXUF; }
static inline void
i2cn_(flag_tx_overflow_disable, void)       { I2Cn->IEN &= ~I2C_IEN_TXOF; }
static inline void
i2cn_(flag_tx_overflow_enable, void)        { I2Cn->IEN |= I2C_IEN_TXOF; }
static inline void
i2cn_(flag_bus_held_disable, void)          { I2Cn->IEN &= ~I2C_IEN_BUSHOLD; }
static inline void
i2cn_(flag_bus_held_enable, void)           { I2Cn->IEN |= I2C_IEN_BUSHOLD; }
static inline void
i2cn_(flag_bus_error_disable, void)         { I2Cn->IEN &= ~I2C_IEN_BUSERR; }
static inline void
i2cn_(flag_bus_error_enable, void)          { I2Cn->IEN |= I2C_IEN_BUSERR; }
static inline void
i2cn_(flag_arbitration_lost_disable, void)  { I2Cn->IEN &= ~I2C_IEN_ARBLOST; }
static inline void
i2cn_(flag_arbitration_lost_enable, void)   { I2Cn->IEN |= I2C_IEN_ARBLOST; }
static inline void
i2cn_(flag_master_stop_disable, void)       { I2Cn->IEN &= ~I2C_IEN_MSTOP; }
static inline void
i2cn_(flag_master_stop_enable, void)        { I2Cn->IEN |= I2C_IEN_MSTOP; }
static inline void
i2cn_(flag_nack_disable, void)              { I2Cn->IEN &= ~I2C_IEN_NACK; }
static inline void
i2cn_(flag_nack_enable, void)               { I2Cn->IEN |= I2C_IEN_NACK; }
static inline void
i2cn_(flag_ack_disable, void)               { I2Cn->IEN &= ~I2C_IEN_ACK; }
static inline void
i2cn_(flag_ack_enable, void)                { I2Cn->IEN |= I2C_IEN_ACK; }
static inline void
i2cn_(flag_rx_data_valid_disable, void)     { I2Cn->IEN &= ~I2C_IEN_RXDATAV; }
static inline void
i2cn_(flag_rx_data_valid_enable, void)      { I2Cn->IEN |= I2C_IEN_RXDATAV; }
static inline void
i2cn_(flag_tx_buffer_level_disable, void)   { I2Cn->IEN &= ~I2C_IEN_TXBL; }
static inline void
i2cn_(flag_tx_buffer_level_enable, void)    { I2Cn->IEN |= I2C_IEN_TXBL; }
static inline void
i2cn_(flag_tx_complete_disable, void)       { I2Cn->IEN &= ~I2C_IEN_TXC; }
static inline void
i2cn_(flag_tx_complete_enable, void)        { I2Cn->IEN |= I2C_IEN_TXC; }
static inline void
i2cn_(flag_address_disable, void)           { I2Cn->IEN &= ~I2C_IEN_ADDR; }
static inline void
i2cn_(flag_address_enable, void)            { I2Cn->IEN |= I2C_IEN_ADDR; }
static inline void
i2cn_(flag_repeated_start_disable, void)    { I2Cn->IEN &= ~I2C_IEN_RSTART; }
static inline void
i2cn_(flag_repeated_start_enable, void)     { I2Cn->IEN |= I2C_IEN_RSTART; }
static inline void
i2cn_(flag_start_disable, void)             { I2Cn->IEN &= ~I2C_IEN_START; }
static inline void
i2cn_(flag_start_enable, void)              { I2Cn->IEN |= I2C_IEN_START; }

/* I2Cn_ROUTE */
static inline void
i2cn_(pins, uint32_t v)                     { I2Cn->ROUTE = v; }
static inline void
i2cn_(pins_disable, void)                   { I2Cn->ROUTE &= ~(I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN); }
static inline void
i2cn_(pins_enable, void)                    { I2Cn->ROUTE |= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN; }
