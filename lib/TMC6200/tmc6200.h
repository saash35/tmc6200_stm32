// =============================================================================
//  TMC6200 SPI driver (register level)
//
//  Protocol (datasheet rev 1.08, section "SPI interface"):
//   - 40-bit datagram, MSB first, SPI mode 3
//   - byte 0: bit7 = 1 for write, bits 6..0 = register address
//   - bytes 1..4: 32-bit data
//   - on a read, the data comes back in the SAME datagram (unlike TMC stepper ICs)
//   - the first byte clocked out is an SPI status byte
//   - SCK max 3.5 MHz with the internal oscillator
//
//  The chip only answers when VS (motor supply) is present, because VCC_IO logic
//  is fed but the core runs from the internal 5 V regulator.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>

namespace tmc6200 {

// ---- Register addresses ------------------------------------------------------
enum Reg : uint8_t {
  REG_GCONF        = 0x00,  // RW   global configuration
  REG_GSTAT        = 0x01,  // RW   global status, write 1 to clear
  REG_IOIN         = 0x04,  // R    pin states + version
  REG_OTP_PROG     = 0x06,  // W    OTP programming (never written by this driver)
  REG_OTP_READ     = 0x07,  // R    OTP memory
  REG_FACTORY_CONF = 0x08,  // RW   clock trim
  REG_SHORT_CONF   = 0x09,  // RW   short-circuit protection
  REG_DRV_CONF     = 0x0A,  // RW   gate driver configuration
};

// ---- GCONF (0x00) ------------------------------------------------------------
constexpr uint32_t GCONF_DISABLE       = 1u << 0;  // 1 = driver disabled (like DRV_EN low)
constexpr uint32_t GCONF_SINGLELINE    = 1u << 1;  // 1 = xH is polarity, xL is enable (3-PWM)
                                                   // 0 = individual H/L gate inputs (6-PWM)
                                                   // RESET DEFAULT IS 1 !
constexpr uint32_t GCONF_FAULTDIRECT   = 1u << 2;  // 1 = FAULT pin not latched
constexpr uint32_t GCONF_AMPL_SHIFT    = 4;        // bits 5:4: 0 = x5, 1 = x10, 2/3 = x20
constexpr uint32_t GCONF_AMPL_MASK     = 3u << 4;
constexpr uint32_t GCONF_AMPLIFIER_OFF = 1u << 6;  // 1 = sense amplifiers off
constexpr uint32_t GCONF_TEST_MODE     = 1u << 7;  // keep 0
constexpr uint32_t GCONF_RW_MASK       = 0x000000F7;

// ---- GSTAT (0x01) ------------------------------------------------------------
constexpr uint32_t GSTAT_RESET      = 1u << 0;   // chip has been reset since last clear
constexpr uint32_t GSTAT_DRV_OTPW   = 1u << 1;   // over-temperature pre-warning
constexpr uint32_t GSTAT_DRV_OT     = 1u << 2;   // over-temperature shutdown
constexpr uint32_t GSTAT_UV_CP      = 1u << 3;   // charge pump under-voltage
constexpr uint32_t GSTAT_SHORTDET_U = 1u << 4;
constexpr uint32_t GSTAT_S2GU       = 1u << 5;   // phase U short to GND
constexpr uint32_t GSTAT_S2VSU      = 1u << 6;   // phase U short to VS
constexpr uint32_t GSTAT_SHORTDET_V = 1u << 8;
constexpr uint32_t GSTAT_S2GV       = 1u << 9;
constexpr uint32_t GSTAT_S2VSV      = 1u << 10;
constexpr uint32_t GSTAT_SHORTDET_W = 1u << 12;
constexpr uint32_t GSTAT_S2GW       = 1u << 13;
constexpr uint32_t GSTAT_S2VSW      = 1u << 14;
constexpr uint32_t GSTAT_ALL        = 0x0000777F;
// Bits that mean "the power stage was shut down / must not run"
constexpr uint32_t GSTAT_FAULT_MASK = GSTAT_DRV_OT | GSTAT_UV_CP |
                                      GSTAT_SHORTDET_U | GSTAT_S2GU | GSTAT_S2VSU |
                                      GSTAT_SHORTDET_V | GSTAT_S2GV | GSTAT_S2VSV |
                                      GSTAT_SHORTDET_W | GSTAT_S2GW | GSTAT_S2VSW;

// ---- IOIN (0x04) -------------------------------------------------------------
constexpr uint32_t IOIN_UL      = 1u << 0;
constexpr uint32_t IOIN_UH      = 1u << 1;
constexpr uint32_t IOIN_VL      = 1u << 2;
constexpr uint32_t IOIN_VH      = 1u << 3;
constexpr uint32_t IOIN_WL      = 1u << 4;
constexpr uint32_t IOIN_WH      = 1u << 5;
constexpr uint32_t IOIN_DRV_EN  = 1u << 6;
constexpr uint32_t IOIN_OTPW    = 1u << 8;
constexpr uint32_t IOIN_OT136   = 1u << 9;
constexpr uint32_t IOIN_OT143   = 1u << 10;
constexpr uint32_t IOIN_OT150   = 1u << 11;
constexpr uint8_t  IOIN_VERSION = 0x10;          // bits 31:24

// ---- SHORT_CONF (0x09) -------------------------------------------------------
constexpr uint32_t SHORT_S2VS_LEVEL_SHIFT  = 0;   // 4 bits, 1 (sensitive) .. 15 (insensitive)
constexpr uint32_t SHORT_S2G_LEVEL_SHIFT   = 8;   // 4 bits, 2 .. 15
constexpr uint32_t SHORT_FILTER_SHIFT      = 16;  // 2 bits, 0 = 100 ns .. 3 = 3 us
constexpr uint32_t SHORT_DELAY             = 1u << 20;  // 0 = 750 ns, 1 = 1500 ns
constexpr uint32_t SHORT_RETRY_SHIFT       = 24;  // 2 bits
constexpr uint32_t SHORT_PROTECT_PARALLEL  = 1u << 28;  // 1 = one short disables all bridges
constexpr uint32_t SHORT_DISABLE_S2G       = 1u << 29;
constexpr uint32_t SHORT_DISABLE_S2VS      = 1u << 30;
constexpr uint32_t SHORT_RW_MASK           = 0x73130F0F;

// ---- DRV_CONF (0x0A) ---------------------------------------------------------
constexpr uint32_t DRV_BBMCLKS_SHIFT     = 0;    // 5 bits, only used when singleline = 1
constexpr uint32_t DRV_OTSELECT_SHIFT    = 16;   // 2 bits: 0 = 150 C, 1 = 143, 2 = 136, 3 = 120
constexpr uint32_t DRV_STRENGTH_SHIFT    = 18;   // 2 bits: 0 weak, 1 weak+TC, 2 medium, 3 strong
constexpr uint32_t DRV_RW_MASK           = 0x000F001F;

// ---- User configuration ------------------------------------------------------
struct Config {
  bool    singleLine      = false; // false = 6-PWM (individual gate inputs)
  bool    faultDirect     = false; // false = FAULT latched until DRV_EN is cycled
  uint8_t csGain          = 10;    // 5, 10 or 20
  bool    amplifierOff    = false;
  uint8_t s2vsLevel       = 6;     // reset default 6
  uint8_t s2gLevel        = 6;     // reset default 6
  uint8_t shortFilter     = 1;     // reset default 1 (1 us)
  bool    shortDelay      = false; // reset default 0 (750 ns)
  uint8_t retry           = 3;     // reset default 3
  bool    protectParallel = true;  // reset default 1
  bool    disableS2G      = false;
  bool    disableS2VS     = false;
  uint8_t bbmClks         = 4;     // ignored in 6-PWM mode
  uint8_t otSelect        = 0;     // 150 C
  uint8_t drvStrength     = 2;     // medium (reset default)
};

class Driver {
public:
  Driver(SPIClass& spi, int csnPin, uint32_t spiHz = 1000000)
      : _spi(spi), _csn(csnPin), _settings(spiHz, MSBFIRST, SPI_MODE3) {}

  // Configure CSN. Call SPI.begin() yourself before this.
  void begin();

  // Raw register access
  uint32_t readReg(uint8_t addr);
  void     writeReg(uint8_t addr, uint32_t value);
  uint8_t  lastSpiStatus() const { return _lastStatus; }

  // true when IOIN reports the TMC6200 version byte (0x10)
  bool isConnected();

  // Write GCONF / SHORT_CONF / DRV_CONF and read them back. false = mismatch.
  bool configure(const Config& cfg);

  uint32_t readStatus()                 { return readReg(REG_GSTAT) & GSTAT_ALL; }
  void     clearStatus(uint32_t bits = GSTAT_ALL) { writeReg(REG_GSTAT, bits); }
  uint32_t readInputs()                 { return readReg(REG_IOIN); }

  // Human readable dumps
  void printStatus(Print& out);
  static void printGstat(Print& out, uint32_t gstat);

private:
  uint32_t transfer40(uint8_t first, uint32_t data);

  SPIClass&   _spi;
  int         _csn;
  SPISettings _settings;
  uint8_t     _lastStatus = 0;
};

}  // namespace tmc6200
