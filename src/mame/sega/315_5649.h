// license: BSD-3-Clause
// copyright-holders: Dirk Best
/***************************************************************************

    Sega 315-5649

    I/O Controller

***************************************************************************/

#ifndef MAME_SEGA_315_5649_H
#define MAME_SEGA_315_5649_H

#pragma once


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class sega_315_5649_device : public device_t
{
public:
	// construction/destruction
	sega_315_5649_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// configuration
	auto in_pa_callback() { return m_in_port_cb[0].bind(); }
	auto in_pb_callback() { return m_in_port_cb[1].bind(); }
	auto in_pc_callback() { return m_in_port_cb[2].bind(); }
	auto in_pd_callback() { return m_in_port_cb[3].bind(); }
	auto in_pe_callback() { return m_in_port_cb[4].bind(); }
	auto in_pf_callback() { return m_in_port_cb[5].bind(); }
	auto in_pg_callback() { return m_in_port_cb[6].bind(); }

	auto out_pa_callback() { return m_out_port_cb[0].bind(); }
	auto out_pb_callback() { return m_out_port_cb[1].bind(); }
	auto out_pc_callback() { return m_out_port_cb[2].bind(); }
	auto out_pd_callback() { return m_out_port_cb[3].bind(); }
	auto out_pe_callback() { return m_out_port_cb[4].bind(); }
	auto out_pf_callback() { return m_out_port_cb[5].bind(); }
	auto out_pg_callback() { return m_out_port_cb[6].bind(); }

	template <unsigned N> auto an_port_callback() { return m_an_port_cb[N].bind(); }

	auto serial_ch1_rd_callback() { return m_serial_rd_cb[0].bind(); }
	auto serial_ch2_rd_callback() { return m_serial_rd_cb[1].bind(); }

	auto serial_ch1_wr_callback() { return m_serial_wr_cb[0].bind(); }
	auto serial_ch2_wr_callback() { return m_serial_wr_cb[1].bind(); }

	template <unsigned N> auto in_counter_callback() { return m_cnt_cb[N].bind(); }

	uint8_t read(offs_t offset);
	void write(offs_t offset, uint8_t data);

	// Emulator host-injection: push a byte into a channel's RX FIFO as if a
	// remote RS-422 peer had sent it. (Also invoked internally by LOOP echo.)
	void serial_rx(unsigned ch, uint8_t data);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	// callbacks
	devcb_read8::array<7> m_in_port_cb;
	devcb_write8::array<7> m_out_port_cb;
	devcb_read8::array<8> m_an_port_cb;
	devcb_read8::array<2> m_serial_rd_cb;
	devcb_write8::array<2> m_serial_wr_cb;
	devcb_read16::array<4> m_cnt_cb;

	uint8_t m_port_value[7];
	uint8_t m_port_config;
	uint8_t m_mode;
	int m_analog_channel;

	// RS-422 hardware loopback (MODE bit4): a TX byte is echoed straight into
	// the same channel's receive FIFO entirely on-chip. Used by the m2-x11 X11
	// server's transport self-test; gated on the LOOP bit so non-loopback games
	// keep the legacy callback/flag behaviour.
	static constexpr unsigned LOOP_FIFO_SIZE = 16;
	uint8_t m_loop_fifo[2][LOOP_FIFO_SIZE];   // board-visible RX FIFOs (read via RXD1/2)
	uint8_t m_loop_head[2];
	uint8_t m_loop_count[2];
	bool m_serial_live;   // true once LOOP or host injection has driven the link

	// Faithful two-channel transaction model (matches the real serial_stuff link):
	// TXD2 is latched, the TXD1 write is the strobe that runs one transfer and
	// synthesises the peer reply from the host->board queue (filled by injection).
	static constexpr unsigned HOST_TX_SIZE = 4096;
	uint8_t  m_tx_latch;                       // last TXD2 (data) byte written
	uint8_t  m_hosttx_fifo[HOST_TX_SIZE];      // host -> board data queue
	uint16_t m_hosttx_head;
	uint16_t m_hosttx_count;

	// board -> host capture: data bytes the board sends (DATA-command strobes)
	// are queued here for the emulator host to read back (RXD-pop @ offset 0x07,
	// count @ offset 0x08). Lets the MCP bridge act as the remote X11 client.
	uint8_t  m_hostrx_fifo[HOST_TX_SIZE];
	uint16_t m_hostrx_head;
	uint16_t m_hostrx_count;
};

// device type definition
DECLARE_DEVICE_TYPE(SEGA_315_5649, sega_315_5649_device)

#endif // MAME_SEGA_315_5649_H
