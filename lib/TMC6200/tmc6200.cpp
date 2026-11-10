#include "tmc6200.h"

namespace tmc6200 {

void Driver::begin() {
  digitalWrite(_csn, HIGH);   // set level before making it an output: no glitch
  pinMode(_csn, OUTPUT);
  digitalWrite(_csn, HIGH);
}

// One 40-bit datagram. CSN must stay low for all 5 bytes; the rising edge of
// CSN latches a write.
uint32_t Driver::transfer40(uint8_t first, uint32_t data) {
  _spi.beginTransaction(_settings);
  digitalWrite(_csn, LOW);
  _lastStatus = _spi.transfer(first);
  uint32_t rx = 0;
  rx = (rx << 8) | _spi.transfer((uint8_t)(data >> 24));
  rx = (rx << 8) | _spi.transfer((uint8_t)(data >> 16));
  rx = (rx << 8) | _spi.transfer((uint8_t)(data >> 8));
  rx = (rx << 8) | _spi.transfer((uint8_t)(data));
  digitalWrite(_csn, HIGH);
  _spi.endTransaction();
  delayMicroseconds(2);       // CSN high time between datagrams
  return rx;
}

uint32_t Driver::readReg(uint8_t addr) {
  return transfer40(addr & 0x7F, 0);
}

void Driver::writeReg(uint8_t addr, uint32_t value) {
  transfer40(addr | 0x80, value);
}

bool Driver::isConnected() {
  uint32_t ioin = readInputs();
  return (uint8_t)(ioin >> 24) == IOIN_VERSION;
}

bool Driver::configure(const Config& c) {
  // ---- GCONF
  uint32_t ampl = (c.csGain >= 20) ? 2u : (c.csGain >= 10) ? 1u : 0u;
  uint32_t gconf = 0;
  if (c.singleLine)   gconf |= GCONF_SINGLELINE;
  if (c.faultDirect)  gconf |= GCONF_FAULTDIRECT;
  if (c.amplifierOff) gconf |= GCONF_AMPLIFIER_OFF;
  gconf |= (ampl << GCONF_AMPL_SHIFT);

  // ---- SHORT_CONF
  uint32_t sc = 0;
  sc |= (uint32_t)(c.s2vsLevel & 0x0F)   << SHORT_S2VS_LEVEL_SHIFT;
  sc |= (uint32_t)(c.s2gLevel & 0x0F)    << SHORT_S2G_LEVEL_SHIFT;
  sc |= (uint32_t)(c.shortFilter & 0x03) << SHORT_FILTER_SHIFT;
  sc |= (uint32_t)(c.retry & 0x03)       << SHORT_RETRY_SHIFT;
  if (c.shortDelay)      sc |= SHORT_DELAY;
  if (c.protectParallel) sc |= SHORT_PROTECT_PARALLEL;
  if (c.disableS2G)      sc |= SHORT_DISABLE_S2G;
  if (c.disableS2VS)     sc |= SHORT_DISABLE_S2VS;

  // ---- DRV_CONF
  uint32_t dc = 0;
  dc |= (uint32_t)(c.bbmClks & 0x1F)     << DRV_BBMCLKS_SHIFT;
  dc |= (uint32_t)(c.otSelect & 0x03)    << DRV_OTSELECT_SHIFT;
  dc |= (uint32_t)(c.drvStrength & 0x03) << DRV_STRENGTH_SHIFT;

  writeReg(REG_GCONF, gconf);
  writeReg(REG_SHORT_CONF, sc);
  writeReg(REG_DRV_CONF, dc);

  bool ok = true;
  ok &= ((readReg(REG_GCONF)      & GCONF_RW_MASK) == (gconf & GCONF_RW_MASK));
  ok &= ((readReg(REG_SHORT_CONF) & SHORT_RW_MASK) == (sc    & SHORT_RW_MASK));
  ok &= ((readReg(REG_DRV_CONF)   & DRV_RW_MASK)   == (dc    & DRV_RW_MASK));
  return ok;
}

static void printHex32(Print& out, uint32_t v) {
  out.print("0x");
  for (int i = 28; i >= 0; i -= 4) out.print((v >> i) & 0xF, HEX);
}

void Driver::printGstat(Print& out, uint32_t g) {
  if ((g & GSTAT_ALL) == 0) { out.println("  (no flags set)"); return; }
  if (g & GSTAT_RESET)      out.println("  RESET      chip was reset (registers are back at defaults!)");
  if (g & GSTAT_DRV_OTPW)   out.println("  DRV_OTPW   over-temperature pre-warning");
  if (g & GSTAT_DRV_OT)     out.println("  DRV_OT     over-temperature shutdown");
  if (g & GSTAT_UV_CP)      out.println("  UV_CP      charge pump under-voltage (check VS, CPI/CPO cap)");
  if (g & GSTAT_SHORTDET_U) out.println("  SHORTDET_U short detected on phase U");
  if (g & GSTAT_S2GU)       out.println("  S2GU       phase U short to GND");
  if (g & GSTAT_S2VSU)      out.println("  S2VSU      phase U short to VS");
  if (g & GSTAT_SHORTDET_V) out.println("  SHORTDET_V short detected on phase V");
  if (g & GSTAT_S2GV)       out.println("  S2GV       phase V short to GND");
  if (g & GSTAT_S2VSV)      out.println("  S2VSV      phase V short to VS");
  if (g & GSTAT_SHORTDET_W) out.println("  SHORTDET_W short detected on phase W");
  if (g & GSTAT_S2GW)       out.println("  S2GW       phase W short to GND");
  if (g & GSTAT_S2VSW)      out.println("  S2VSW      phase W short to VS");
}

void Driver::printStatus(Print& out) {
  uint32_t gconf = readReg(REG_GCONF);
  uint32_t gstat = readReg(REG_GSTAT);
  uint32_t ioin  = readReg(REG_IOIN);
  uint32_t sc    = readReg(REG_SHORT_CONF);
  uint32_t dc    = readReg(REG_DRV_CONF);

  out.println("--- TMC6200 ---");
  out.print("GCONF      "); printHex32(out, gconf);
  out.print((gconf & GCONF_SINGLELINE) ? "  3-PWM(singleline)" : "  6-PWM");
  out.print("  gain x");
  { uint32_t a = (gconf & GCONF_AMPL_MASK) >> GCONF_AMPL_SHIFT; out.print(a == 0 ? 5 : a == 1 ? 10 : 20); }
  out.println((gconf & GCONF_DISABLE) ? "  DISABLED" : "");

  out.print("GSTAT      "); printHex32(out, gstat); out.println();
  printGstat(out, gstat);

  out.print("IOIN       "); printHex32(out, ioin);
  out.print("  version 0x"); out.print((ioin >> 24) & 0xFF, HEX);
  out.print("  DRV_EN="); out.print((ioin & IOIN_DRV_EN) ? 1 : 0);
  out.print("  UH/UL/VH/VL/WH/WL=");
  out.print((ioin & IOIN_UH) ? 1 : 0); out.print((ioin & IOIN_UL) ? 1 : 0);
  out.print((ioin & IOIN_VH) ? 1 : 0); out.print((ioin & IOIN_VL) ? 1 : 0);
  out.print((ioin & IOIN_WH) ? 1 : 0); out.print((ioin & IOIN_WL) ? 1 : 0);
  if (ioin & IOIN_OTPW) out.print("  OTPW");
  out.println();

  out.print("SHORT_CONF "); printHex32(out, sc); out.println();
  out.print("DRV_CONF   "); printHex32(out, dc);
  out.print("  strength="); out.println((dc >> DRV_STRENGTH_SHIFT) & 0x3);
}

}  // namespace tmc6200
