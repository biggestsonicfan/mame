// license:BSD-3-Clause
// copyright-holders:
/***************************************************************************************************

  TN160 / SL00055 / JBC017-11 12-bay AA/AAA NiMH smart battery charger.
  Sold as SunLabz "Fast Battery Charger", Kweller X-1200, Tenergy TN160, etc.
  Board silk: "DHX-2C  UL E342984  94V-0".

  Main MCU:  Sinowealth SH79F166AF (enhanced 1T-8051, 16KB flash, 10-bit ADC, QFP44).
  Display:   CMS1621E (HT1621-compatible LCD driver), bit-banged on P2.0(WR)/P1.7(DATA)/P2.1(CS).
  Sensing:   two CD4051 8:1 analog muxes route one cell's voltage to the on-chip ADC.
             IC4 addr = {P3.0,P1.0,P1.1}, selected when ADCH bit7 set.
             IC5 addr = {P0.5,P0.6,P0.7}, selected when ADCH bit3 set.
  Charge:    continuous, per-bay 0.75ohm ballast resistors (no MCU current switch, no PWM).
  Backlight: P2.4 (on whenever any bay is actively charging).

  This driver models enough of the SH79F166A (ADC + SFR-bank switch + Timer2 tick) plus the
  CD4051 mux + a settable 12-cell battery model to boot the firmware and observe behaviour.

  MILESTONE 1: boot -> main, Timer2 scheduler tick, ADC/mux + scriptable cells, observe P2.4
  backlight and slot LEDs via the debugger/MCP bridge. LCD decode + artwork come later.

***************************************************************************************************/

#include "emu.h"
#include "cpu/mcs51/i8052.h"

#include "tn160.lh"


//**************************************************************************
//  SH79F166A CPU DEVICE (derived from i8052 for native Timer2 = system tick)
//**************************************************************************

DECLARE_DEVICE_TYPE(SH79F166A, sh79f166a_device)

class sh79f166a_device : public i8052_device
{
public:
	sh79f166a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// driver supplies the 10-bit ADC reading for the current channel; offset = ADCH latch value
	auto adc_in_cb() { return m_adc_in_cb.bind(); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void sfr_map(address_map &map) override ATTR_COLD;

private:
	// ADC
	u8   adcon_r() { return m_adcon; }
	void adcon_w(u8 data);
	u8   adt_r()  { return m_adt; }
	void adt_w(u8 data)  { m_adt = data; }
	u8   adch_r() { return m_adch; }
	void adch_w(u8 data) { m_adch = data; }
	u8   addl_r() { return m_addl; }
	u8   addh_r() { return m_addh; }
	void start_conversion();

	// INSCON (0x86): bit6 (BKS0) selects SFR bank 1
	u8   inscon_r() { return m_inscon; }
	void inscon_w(u8 data) { m_inscon = data; }
	bool bank1() const { return BIT(m_inscon, 6); }

	// bank-aware Timer2 registers (protect from bank-1 T4/T5 accesses)
	u8   tc8_r();
	void tc8_w(u8 data);
	u8   tcc_r(offs_t o);
	void tcc_w(offs_t o, u8 data);
	u8   tce_r(offs_t o) { return m_bank1[0xce + o]; }
	void tce_w(offs_t o, u8 data) { m_bank1[0xce + o] = data; }

	// generic absorb latch for unmodeled SFRs (config/display/pwm/etc.)
	u8   latch_r(offs_t o) { return m_sfr[o & 0xff]; }
	void latch_w(offs_t o, u8 data) { m_sfr[o & 0xff] = data; }

	devcb_read16 m_adc_in_cb;

	u8  m_inscon = 0;
	u8  m_adcon = 0, m_adt = 0, m_adch = 0, m_addl = 0, m_addh = 0;
	u8  m_bank1[0x100];
	u8  m_sfr[0x100];
};

DEFINE_DEVICE_TYPE(SH79F166A, sh79f166a_device, "sh79f166a", "Sinowealth SH79F166A")


sh79f166a_device::sh79f166a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: i8052_device(mconfig, SH79F166A, tag, owner, clock, 14 /* 16KB program */)
	, m_adc_in_cb(*this, 0)
{
}

void sh79f166a_device::device_start()
{
	i8052_device::device_start();
	// devcb read callbacks resolve automatically during device setup; no manual resolve needed
	std::fill(std::begin(m_bank1), std::end(m_bank1), 0);
	std::fill(std::begin(m_sfr),   std::end(m_sfr),   0);
	save_item(NAME(m_inscon));
	save_item(NAME(m_adcon)); save_item(NAME(m_adt)); save_item(NAME(m_adch));
	save_item(NAME(m_addl)); save_item(NAME(m_addh));
	save_item(NAME(m_bank1)); save_item(NAME(m_sfr));
}

void sh79f166a_device::device_reset()
{
	i8052_device::device_reset();
	m_inscon = m_adcon = m_adt = m_adch = m_addl = m_addh = 0;
}

// SH79F166A on-chip ADC: writing ADCON.0 starts a conversion; result appears in ADDH:ADDL
// (ADDH = bits 9:2, ADDL[1:0] = bits 1:0) and ADCON.0 self-clears when done.
void sh79f166a_device::start_conversion()
{
	u16 v = m_adc_in_cb(m_adch) & 0x3ff;
	m_addh = (v >> 2) & 0xff;
	m_addl = v & 0x03;
	m_adcon &= ~0x01;
}

void sh79f166a_device::adcon_w(u8 data)
{
	m_adcon = data;
	if (BIT(data, 0))
		start_conversion();
}

// 0xC8: T2CON (bank0, drives our tick) / T4CON (bank1, latch only)
u8 sh79f166a_device::tc8_r() { return bank1() ? m_bank1[0xc8] : i8052_device::t2con_r(); }
void sh79f166a_device::tc8_w(u8 data) { if (bank1()) m_bank1[0xc8] = data; else i8052_device::t2con_w(data); }

// 0xCC-0xCD: TL2/TH2 (bank0) / TL4/TH4 (bank1)
u8 sh79f166a_device::tcc_r(offs_t o) { return bank1() ? m_bank1[0xcc + o] : i8052_device::t2_r(o); }
void sh79f166a_device::tcc_w(offs_t o, u8 data) { if (bank1()) m_bank1[0xcc + o] = data; else i8052_device::t2_w(o, data); }

void sh79f166a_device::sfr_map(address_map &map)
{
	i8052_device::sfr_map(map);   // P0-P3, timers, T2CON@0xC8, ACC/B/PSW, etc.

	map(0x86, 0x86).rw(FUNC(sh79f166a_device::inscon_r), FUNC(sh79f166a_device::inscon_w));

	// on-chip 10-bit ADC
	map(0x93, 0x93).rw(FUNC(sh79f166a_device::adcon_r), FUNC(sh79f166a_device::adcon_w));
	map(0x94, 0x94).rw(FUNC(sh79f166a_device::adt_r), FUNC(sh79f166a_device::adt_w));
	map(0x95, 0x95).rw(FUNC(sh79f166a_device::adch_r), FUNC(sh79f166a_device::adch_w));
	map(0x96, 0x96).r(FUNC(sh79f166a_device::addl_r));
	map(0x97, 0x97).r(FUNC(sh79f166a_device::addh_r));

	// bank-aware shared registers (protect Timer2 from bank-1 T4/T5 accesses).
	// NOTE: 0x80 (P0/P5) is intentionally NOT banked: bit-addressed port SFRs like P0.1
	// (the REFRESH button, read while the ISR has bank1 selected for P5 config) do not bank
	// on this part, and we don't model P5 outputs (XTAL/RST/buzzer). Folding 0x80 to P0
	// keeps the button reading P0.1.
	map(0xc8, 0xc8).rw(FUNC(sh79f166a_device::tc8_r), FUNC(sh79f166a_device::tc8_w));
	map(0xcc, 0xcd).rw(FUNC(sh79f166a_device::tcc_r), FUNC(sh79f166a_device::tcc_w));
	map(0xce, 0xcf).rw(FUNC(sh79f166a_device::tce_r), FUNC(sh79f166a_device::tce_w));

	// everything else the firmware touches: absorb into a latch (read returns last write)
	map(0xa7, 0xa7).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xa9, 0xa9).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xaa, 0xad).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0x9a, 0x9f).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xb1, 0xbb).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xbd, 0xbd).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xc0, 0xc0).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xd1, 0xd6).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xe1, 0xef).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xf1, 0xf7).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
	map(0xfc, 0xfd).rw(FUNC(sh79f166a_device::latch_r), FUNC(sh79f166a_device::latch_w));
}


//**************************************************************************
//  CHARGER MACHINE DRIVER
//**************************************************************************

namespace {

class tn160_state : public driver_device
{
public:
	tn160_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_cell(*this, "CELL%u", 0U)
		, m_refresh(*this, "REFRESH")
		, m_lcdseg(*this, "lcdseg%u", 0U)
		, m_backlight(*this, "backlight")
	{
	}

	void tn160(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<sh79f166a_device> m_maincpu;
	optional_ioport_array<12> m_cell;
	required_ioport m_refresh;      // REFRESH button (P0.1, active low)
	output_finder<128> m_lcdseg;   // one per HT1621 RAM bit (addr*4 + bit)
	output_finder<> m_backlight;   // P2.4

	u8 m_p0 = 0xff, m_p1 = 0xff, m_p2 = 0xff, m_p3 = 0xff;
	u16 m_cell_mv[12];

	// CMS1621 / HT1621 LCD controller serial interface (P2.1=CS, P2.0=WR, P1.7=DATA)
	u8  m_lcd_ram[32];      // 32 addresses x 4 bits (128 segment bits)
	u32 m_lcd_acc;          // shift accumulator
	int m_lcd_nbits;        // bits collected in current field
	int m_lcd_phase;        // 0=mode, 1=command, 2=address, 3=data
	u8  m_lcd_addr;         // current RAM address (auto-increments in data phase)
	void lcd_shift_bit(int bit);
	void lcd_frame_end();
	void lcd_update_outputs();

	u8 p0_r();
	void p0_w(u8 data) { m_p0 = data; }
	void p1_w(u8 data) { m_p1 = data; }   // DATA on bit 7
	void p2_w(u8 data);                    // CS/WR serial clock -> HT1621 decode
	void p3_w(u8 data) { m_p3 = data; }

	u16 adc_r(offs_t adch);
	int decode_bay(u8 adch) const;
};

// map (mux, 3-bit address) -> logical bay 0..11, matching the firmware detection scan order.
int tn160_state::decode_bay(u8 adch) const
{
	if (BIT(adch, 7)) // IC4: {P3.0,P1.0,P1.1}
	{
		u8 a = (BIT(m_p3,0) << 2) | (BIT(m_p1,0) << 1) | BIT(m_p1,1);
		switch (a) { case 0b001:return 0; case 0b011:return 1; case 0b111:return 2;
		             case 0b101:return 3; case 0b110:return 4; case 0b000:return 5; }
	}
	else if (BIT(adch, 3)) // IC5: address {P0.7(MSB),P0.6,P0.5(LSB)}
	{
		u8 a = (BIT(m_p0,7) << 2) | (BIT(m_p0,6) << 1) | BIT(m_p0,5);
		switch (a) { case 0b001:return 6; case 0b011:return 7; case 0b111:return 8;
		             case 0b101:return 9; case 0b000:return 10; case 0b110:return 11; }
	}
	return -1;
}

// bay voltage (mV) -> 10-bit ADC code (assume ~2.5V AVREF, no divider).
u16 tn160_state::adc_r(offs_t adch)
{
	int bay = decode_bay(adch);
	if (bay < 0)
	{
		// A mux is selected but the 3-bit code isn't one of the 6 per-cell channels
		// (e.g. code 100 = chsel_100/33C8, the reference/second-connect channel the charge
		// engine reads before the cell). Return a neutral reference above the low-reading
		// fault threshold (0x0051) and below over-voltage so it doesn't false-trigger.
		// TODO: reverse-engineer the exact reference so the full charge cycle is faithful.
		if (BIT(adch, 7) || BIT(adch, 3))
		{
			static int n = 0;
			if (n++ < 200)
				logerror("REF adch=%02x p0=%02x p1=%02x p3=%02x  (returns 0x100)\n",
						 adch, m_p0, m_p1, m_p3);
			return 0x0100;
		}
		return 0;
	}
	u16 mv = m_cell_mv[bay];
	bool present = m_cell[bay].found() && m_cell[bay]->read();
	if (present)
		mv = mv ? mv : 1300;
	u32 code = (u32)mv * 1023 / 2500;
	if (code > 1023) code = 1023;
	// DEBUG: any read >= detection threshold (0x23) from an EMPTY bay is a phantom source
	if (code >= 0x23 && mv == 0 && !present)
	{
		static int n = 0;
		if (n++ < 150)
			logerror("PH adch=%02x p0=%02x p1=%02x p3=%02x bay=%d code=%u\n",
					 adch, m_p0, m_p1, m_p3, bay, code);
	}
	return code;
}

// ---- CMS1621 / HT1621 LCD controller serial decode ----
// The firmware bit-bangs: CS=P2.1 frames a transfer, each bit is DATA=P1.7 sampled on the
// rising edge of WR=P2.0. A frame is: 3 mode bits ("100"=command, "101"=write, "110"=read),
// then either 9-bit command(s) or a 6-bit address followed by 4-bit data nibbles (addr
// auto-increments). We keep the 32x4 segment RAM and mirror it to outputs + the log.
void tn160_state::p2_w(u8 data)
{
	u8 old = m_p2;
	m_p2 = data;
	m_backlight = BIT(data, 4);   // P2.4 = display backlight
	bool cs_old = BIT(old, 1), cs_new = BIT(data, 1);
	bool wr_old = BIT(old, 0), wr_new = BIT(data, 0);

	if (cs_old && !cs_new) { m_lcd_acc = 0; m_lcd_nbits = 0; m_lcd_phase = 0; }   // CS falling: start
	if (!cs_new && !wr_old && wr_new) lcd_shift_bit(BIT(m_p1, 7));                // WR rising: clock bit
	if (!cs_old && cs_new) lcd_frame_end();                                       // CS rising: end
}

void tn160_state::lcd_shift_bit(int bit)
{
	m_lcd_acc = (m_lcd_acc << 1) | (bit & 1);
	m_lcd_nbits++;
	switch (m_lcd_phase)
	{
	case 0: // mode: 3 bits
		if (m_lcd_nbits == 3)
		{
			u8 mode = m_lcd_acc & 7;
			m_lcd_acc = 0; m_lcd_nbits = 0;
			m_lcd_phase = (mode == 0b101) ? 2 : (mode == 0b100) ? 1 : 4; // write / command / (read)
		}
		break;
	case 1: // command: 9-bit fields (consumed, not acted on)
		if (m_lcd_nbits == 9) { m_lcd_acc = 0; m_lcd_nbits = 0; }
		break;
	case 2: // address: 6 bits (MSB first)
		if (m_lcd_nbits == 6)
		{
			m_lcd_addr = m_lcd_acc & 0x3f;
			m_lcd_acc = 0; m_lcd_nbits = 0; m_lcd_phase = 3;
		}
		break;
	case 3: // data: 4-bit nibbles, address auto-increments
		if (m_lcd_nbits == 4)
		{
			if (m_lcd_addr < 32) m_lcd_ram[m_lcd_addr] = m_lcd_acc & 0x0f;
			m_lcd_addr = (m_lcd_addr + 1) & 0x3f;
			m_lcd_acc = 0; m_lcd_nbits = 0;
		}
		break;
	default: break; // read mode: ignore
	}
}

void tn160_state::lcd_update_outputs()
{
	for (int a = 0; a < 32; a++)
		for (int b = 0; b < 4; b++)
			m_lcdseg[a * 4 + b] = BIT(m_lcd_ram[a], b);
}

void tn160_state::lcd_frame_end()
{
	lcd_update_outputs();
	// dump RAM for offline segment mapping (goes to mame/error.log via -log)
	std::string s;
	for (int a = 0; a < 32; a++) s += util::string_format("%X", m_lcd_ram[a]);
	logerror("LCD RAM: %s\n", s);
}

u8 tn160_state::p0_r()
{
	// External pin levels seen by the CPU. MAME computes the port read as
	// (latch | forced_inputs) & input_cb(); set_port_forced_input() pulls P0.1 high so the
	// button (active-low) is released by default. Pressing REFRESH clears P0.1 here.
	u8 v = 0xff;
	if (m_refresh->read() & 1) v &= ~0x02;   // REFRESH held -> P0.1 low
	return v;
}

void tn160_state::machine_start()
{
	m_lcdseg.resolve();
	m_backlight.resolve();
	std::fill(std::begin(m_cell_mv), std::end(m_cell_mv), 0);
	std::fill(std::begin(m_lcd_ram), std::end(m_lcd_ram), 0);
	m_lcd_acc = 0; m_lcd_nbits = 0; m_lcd_phase = 0; m_lcd_addr = 0;
	save_item(NAME(m_p0)); save_item(NAME(m_p1)); save_item(NAME(m_p2)); save_item(NAME(m_p3));
	save_item(NAME(m_cell_mv));
	save_item(NAME(m_lcd_ram)); save_item(NAME(m_lcd_acc)); save_item(NAME(m_lcd_nbits));
	save_item(NAME(m_lcd_phase)); save_item(NAME(m_lcd_addr));
}

void tn160_state::machine_reset()
{
	// Start with an empty charger: no cells -> nothing charging -> backlight turns off.
	// Insert batteries at runtime with the CELL toggles (keys 1-9,0,-,= for bays 1-12).
	std::fill(std::begin(m_cell_mv), std::end(m_cell_mv), 0);
}

static INPUT_PORTS_START(tn160)
	PORT_START("REFRESH")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("REFRESH") PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT(0xfe, IP_ACTIVE_HIGH, IPT_UNUSED)

	// One toggle per bay: press to insert / remove a ~1.3V cell (bay d -> driver index d-1).
	PORT_START("CELL0")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_1) PORT_NAME("Bay 1 battery")
	PORT_START("CELL1")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_2) PORT_NAME("Bay 2 battery")
	PORT_START("CELL2")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_3) PORT_NAME("Bay 3 battery")
	PORT_START("CELL3")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_4) PORT_NAME("Bay 4 battery")
	PORT_START("CELL4")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_5) PORT_NAME("Bay 5 battery")
	PORT_START("CELL5")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_6) PORT_NAME("Bay 6 battery")
	PORT_START("CELL6")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_7) PORT_NAME("Bay 7 battery")
	PORT_START("CELL7")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_8) PORT_NAME("Bay 8 battery")
	PORT_START("CELL8")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_9) PORT_NAME("Bay 9 battery")
	PORT_START("CELL9")  PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_0) PORT_NAME("Bay 10 battery")
	PORT_START("CELL10") PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_MINUS) PORT_NAME("Bay 11 battery")
	PORT_START("CELL11") PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_CODE(KEYCODE_EQUALS) PORT_NAME("Bay 12 battery")
INPUT_PORTS_END

void tn160_state::tn160(machine_config &config)
{
	SH79F166A(config, m_maincpu, 12'000'000);   // 12 MHz internal RC (code option)
	m_maincpu->set_port_forced_input(0, 0x02);   // P0.1 REFRESH button pulled high (released)
	m_maincpu->port_out_cb<0>().set(FUNC(tn160_state::p0_w));
	m_maincpu->port_in_cb<0>().set(FUNC(tn160_state::p0_r));
	m_maincpu->port_out_cb<1>().set(FUNC(tn160_state::p1_w));
	m_maincpu->port_out_cb<2>().set(FUNC(tn160_state::p2_w));
	m_maincpu->port_out_cb<3>().set(FUNC(tn160_state::p3_w));
	m_maincpu->adc_in_cb().set(FUNC(tn160_state::adc_r));

	config.set_default_layout(layout_tn160);
}

ROM_START(tn160)
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("sunlabz-mod.bin", 0x0000, 0x4000, CRC(8b6493bd) SHA1(94cb4f63a84cb2b5c5bdef5877ab6fd5af003044))
ROM_END

} // anonymous namespace


//    YEAR  NAME   PARENT MACHINE INPUT  CLASS        INIT        MONITOR COMPANY    FULLNAME                             FLAGS
GAME( 2016, tn160, 0,     tn160,  tn160, tn160_state, empty_init, ROT0,   "SunLabz", "12-bay NiMH charger (SUNLABZ-MOD)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
