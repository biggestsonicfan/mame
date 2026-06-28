// license: BSD-3-Clause
// copyright-holders: Dirk Best
/***************************************************************************

    Sega 315-5649

    I/O Controller

***************************************************************************/

#include "emu.h"
#include "315_5649.h"

#define VERBOSE 0
#include "logmacro.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(SEGA_315_5649, sega_315_5649_device, "315_5649", "Sega 315-5649 I/O Controller")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  sega_315_5649_device - constructor
//-------------------------------------------------

sega_315_5649_device::sega_315_5649_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SEGA_315_5649, tag, owner, clock),
	m_in_port_cb(*this, 0xff),
	m_out_port_cb(*this),
	m_an_port_cb(*this, 0xff),
	m_serial_rd_cb(*this, 0),
	m_serial_wr_cb(*this),
	m_cnt_cb(*this, 0),
	m_port_config(0),
	m_mode(0),
	m_analog_channel(0)
{
	std::fill(std::begin(m_port_value), std::end(m_port_value), 0xff);
	std::fill(&m_loop_fifo[0][0], &m_loop_fifo[0][0] + sizeof(m_loop_fifo), 0);
	m_loop_head[0] = m_loop_head[1] = 0;
	m_loop_count[0] = m_loop_count[1] = 0;
	m_serial_live = false;
	std::fill(std::begin(m_hosttx_fifo), std::end(m_hosttx_fifo), 0);
	std::fill(std::begin(m_hostrx_fifo), std::end(m_hostrx_fifo), 0);
	m_tx_latch = 0;
	m_hosttx_head = 0;
	m_hosttx_count = 0;
	m_hostrx_head = 0;
	m_hostrx_count = 0;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sega_315_5649_device::device_start()
{
	// register for save states
	save_item(NAME(m_port_value));
	save_item(NAME(m_port_config));
	save_item(NAME(m_analog_channel));
	save_item(NAME(m_mode));
	save_item(NAME(m_loop_fifo));
	save_item(NAME(m_loop_head));
	save_item(NAME(m_loop_count));
	save_item(NAME(m_serial_live));
	save_item(NAME(m_tx_latch));
	save_item(NAME(m_hosttx_fifo));
	save_item(NAME(m_hosttx_head));
	save_item(NAME(m_hosttx_count));
	save_item(NAME(m_hostrx_fifo));
	save_item(NAME(m_hostrx_head));
	save_item(NAME(m_hostrx_count));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sega_315_5649_device::device_reset()
{
	// set all ports to input on reset
	m_port_config = 0xff;
	m_mode = 0;
	m_loop_head[0] = m_loop_head[1] = 0;
	m_loop_count[0] = m_loop_count[1] = 0;
	m_tx_latch = 0;
	m_hosttx_head = 0;
	m_hosttx_count = 0;
	m_hostrx_head = 0;
	m_hostrx_count = 0;
}


//**************************************************************************
//  INTERFACE
//**************************************************************************

// Push one received byte into a channel's RX FIFO. Used both by the LOOP echo
// path and by the emulator host-injection hook. Marks the link live so the FLAG
// register reports real buffer status instead of the legacy always-ready hack.
void sega_315_5649_device::serial_rx(unsigned ch, uint8_t data)
{
	if (ch >= 2)
		return;
	m_serial_live = true;
	if (m_loop_count[ch] < LOOP_FIFO_SIZE)
	{
		unsigned tail = (m_loop_head[ch] + m_loop_count[ch]) % LOOP_FIFO_SIZE;
		m_loop_fifo[ch][tail] = data;
		m_loop_count[ch]++;
	}
}

uint8_t sega_315_5649_device::read(offs_t offset)
{
	uint8_t data = 0xff;

	switch (offset)
	{
	// port a to g
	case 0x06:
		if (m_mode & 0x80) // port G counter mode - 4x 16bit counters, auto-increments
		{
			data = m_cnt_cb[(m_port_value[6] >> 1) & 3](0) >> (((m_port_value[6] & 1) ^ 1) * 8);
			if (!machine().side_effects_disabled())
				m_port_value[6] = (m_port_value[6] & 0xf8) | ((m_port_value[6] + 1) & 7);
			break;
		}
		[[fallthrough]];
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
		if (BIT(m_port_config, offset))
			data = m_in_port_cb[offset](0);
		else
			data = m_port_value[offset];
		break;

	// board->host capture pop (emulator host-read hook; offset 0x07 = addr 0x0e)
	case 0x07:
		if (m_serial_live)
		{
			data = m_hostrx_count ? m_hostrx_fifo[m_hostrx_head] : 0;
			if (m_hostrx_count)
			{
				m_hostrx_head = (m_hostrx_head + 1) % HOST_TX_SIZE;
				m_hostrx_count--;
			}
		}
		break;

	// board->host capture count (offset 0x08 = addr 0x10)
	case 0x08:
		if (m_serial_live)
			data = (m_hostrx_count > 255) ? 255 : (uint8_t)m_hostrx_count;
		break;

	// RS-422 channel 1/2 input
	case 0x0b:
	case 0x0c:
	{
		unsigned ch = offset - 0x0b;
		if (m_loop_count[ch]) // pop from the RX FIFO (LOOP echo or host injection)
		{
			data = m_loop_fifo[ch][m_loop_head[ch]];
			if (!machine().side_effects_disabled())
			{
				m_loop_head[ch] = (m_loop_head[ch] + 1) % LOOP_FIFO_SIZE;
				m_loop_count[ch]--;
			}
		}
		else
		{
			data = m_serial_rd_cb[ch](0);
		}
		break;
	}

	// RS-422 status
	// 7--- ----  RX2IE
	// -6-- ----  RX1IE some I-error ?
	// --5- ----  RX2FE
	// ---4 ----  RX1FE framing error ?
	// ---- 3---  RX2BF
	// ---- -2--  RX1BF 1 = receive buffer full
	// ---- --1-  TX2BF
	// ---- ---0  TX1BF 1 = transmit buffer full
	case 0x0d:
		if ((m_mode & 0x10) || m_serial_live)
		{
			// Live RS-422 (LOOP echo or host injection): report real FIFO state.
			// TX always ready/empty (TXnBF clear, matching real silicon at idle =
			// 0x0c); RXnBF set only when that channel has data.
			data = 0x00;
			if (m_loop_count[0]) data |= 0x04; // RX1BF
			if (m_loop_count[1]) data |= 0x08; // RX2BF
		}
		else
		{
			data = 0x0c; // HACK, recv buffers always full, transmit buffers always empty
		}
		break;

	// analog input, auto-increments
	case 0x0f:
		data = m_an_port_cb[m_analog_channel](0);
		if (!machine().side_effects_disabled())
			m_analog_channel = (m_analog_channel + 1) & 0x07;
		break;
	}

	LOG("RD %02x = %02x\n", offset, data);

	return data;
}

void sega_315_5649_device::write(offs_t offset, uint8_t data)
{
	LOG("WR %02x = %02x\n", offset, data);

	switch (offset)
	{
	// port a-g
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
	case 0x06:  // when in counter mode, bit 7 - 0 reset counters (not implemented)
		m_port_value[offset] = data;
		m_out_port_cb[offset](data);
		break;

	// port direction register (0 = output, 1 = input)
	case 0x08: m_port_config = data; break;

	// RS-422 channel 2 (TXD2) = data register: latch the outbound byte. The
	// transfer is not triggered until the command/strobe (TXD1) is written.
	case 0x0a:
		m_tx_latch = data;
		if (!(m_mode & 0x10))
			m_serial_wr_cb[1](data);   // capture board->host data byte
		break;

	// RS-422 channel 1 (TXD1) = command/strobe register: run one transaction.
	case 0x09:
		if (m_mode & 0x10)
		{
			// LOOP: echo both channels as-is (cmd -> RXD1, latched data -> RXD2)
			serial_rx(0, data);
			serial_rx(1, m_tx_latch);
		}
		else
		{
			// Real two-channel transfer: emit the command to the host and
			// synthesise the peer reply from the host->board queue. Channel 0
			// (RXD1) is the status (bit0 = inbound data valid); channel 1 (RXD2)
			// carries the data byte. Pushing both keeps the firmware's
			// "wait until RX1BF & RX2BF" handshake satisfied every transaction.
			uint8_t st = 0, dt = 0;
			m_serial_wr_cb[0](data);   // capture board->host command
			// board->host data capture: a DATA-command (0x07) carries an
			// outbound byte in TXD2 — queue it for the host to read back.
			if (data == 0x07 && m_hostrx_count < HOST_TX_SIZE)
			{
				unsigned t = (m_hostrx_head + m_hostrx_count) % HOST_TX_SIZE;
				m_hostrx_fifo[t] = m_tx_latch;
				m_hostrx_count++;
			}
			if (m_hosttx_count)
			{
				dt = m_hosttx_fifo[m_hosttx_head];
				m_hosttx_head = (m_hosttx_head + 1) % HOST_TX_SIZE;
				m_hosttx_count--;
				st = 0x01;
			}
			serial_rx(0, st);
			serial_rx(1, dt);
		}
		break;

	// RS-422 input registers are read-only on real hardware; a WRITE here is the
	// emulator host-injection hook: enqueue a byte from the host (remote RS-422
	// peer / X11 client) onto the board's data channel. The board receives it on
	// the next transaction (RXD2 + status-valid).
	case 0x0b:
	case 0x0c:
		if (m_hosttx_count < HOST_TX_SIZE)
		{
			unsigned tail = (m_hosttx_head + m_hosttx_count) % HOST_TX_SIZE;
			m_hosttx_fifo[tail] = data;
			m_hosttx_count++;
			m_serial_live = true;
		}
		break;

	// mode register
	// 7--- ----  port G counter mode
	// -6-- ----  ?
	// --5- ----  RS-422 satellite mode
	// ---4 ----  RS-422 loopback
	// ---- 3210  RS-422 satellite N#
	case 0x0e:
		m_mode = data;
		break;

	// analog mux select
	case 0x0f:
		m_analog_channel = data & 0x07;
		break;
	}
}
