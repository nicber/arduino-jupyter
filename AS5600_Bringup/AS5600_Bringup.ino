// AS5600 bring-up sketch
//
// Hardware:
// Arduino UNO
// Hall Position Sensor: AS5600 (I2C)
//
// Wiring: SDA -> A4, SCL -> A5, VDD -> 5V, GND -> GND
// (The AS5600 breakout boards carry their own 3V3 regulator and bus pull-ups.
//  A bare AS5600 in 3.3V mode needs level shifting and 4.7k pull-ups to 3V3.)
//
// Register map, CONF fields and STATUS bits are per the AS5600 datasheet
// [v1-06] 2018-Jun-20, Figures 21/22/23 (Docs/infineon-as5600-datasheet-en.pdf).
//
// Output: 115200 baud. Configuration is dumped at startup (and again whenever a
// character is sent over the serial monitor); telemetry streams at 5 Hz.

#include <Wire.h>

// Set to 0 if the AS5600 is supplied from 3.3V: the AGC range halves.
#define AS5600_VDD_5V 1

static const uint8_t AS5600_ADDR = 0x36;  // 7-bit, 0110110b

// Register addresses (high byte first for the 12-bit registers).
static const uint8_t REG_ZMCO      = 0x00;
static const uint8_t REG_ZPOS_H    = 0x01;
static const uint8_t REG_MPOS_H    = 0x03;
static const uint8_t REG_MANG_H    = 0x05;
static const uint8_t REG_CONF_H    = 0x07;
static const uint8_t REG_RAWANG_H  = 0x0C;
static const uint8_t REG_ANGLE_H   = 0x0E;
static const uint8_t REG_STATUS    = 0x0B;
static const uint8_t REG_AGC       = 0x1A;
static const uint8_t REG_MAG_H     = 0x1B;

// STATUS register bits (Figure 23).
static const uint8_t STATUS_MH = _BV(3);  // AGC minimum gain overflow, magnet too strong
static const uint8_t STATUS_ML = _BV(4);  // AGC maximum gain overflow, magnet too weak
static const uint8_t STATUS_MD = _BV(5);  // magnet was detected

// AGC should sit near the middle of its range; the airgap is adjusted to get there.
#if AS5600_VDD_5V
static const uint8_t AGC_FULL_SCALE = 255;
#else
static const uint8_t AGC_FULL_SCALE = 128;
#endif

// Last I2C error, so every read can report why it failed instead of returning
// a silent zero. 0 = OK, 1..4 = Wire.endTransmission() codes, 5 = short read.
static uint8_t g_i2cError = 0;

static const __FlashStringHelper *i2cErrorText(uint8_t err) {
  switch (err) {
    case 0: return F("ok");
    case 1: return F("data too long for buffer");
    case 2: return F("NACK on address (device not responding)");
    case 3: return F("NACK on data");
    case 4: return F("bus error");
    case 5: return F("short read (fewer bytes than requested)");
    default: return F("unknown");
  }
}

// Reads `count` bytes starting at `reg`. Returns false and sets g_i2cError on failure.
static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t count) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();  // STOP, then a fresh START for the read
  if (err != 0) {
    g_i2cError = err;
    return false;
  }
  if (Wire.requestFrom(AS5600_ADDR, count) != count) {
    g_i2cError = 5;
    return false;
  }
  for (uint8_t i = 0; i < count; i++) {
    buf[i] = Wire.read();
  }
  g_i2cError = 0;
  return true;
}

static bool read8(uint8_t reg, uint8_t *value) {
  return readRegs(reg, value, 1);
}

// 12-bit registers: high byte first, upper nibble of the high byte is unused.
static bool read12(uint8_t regHigh, uint16_t *value) {
  uint8_t buf[2];
  if (!readRegs(regHigh, buf, 2)) {
    return false;
  }
  *value = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
  return true;
}

// 0..4095 counts -> full int16_t range, i.e. Q15 turns: -32768..32767 maps to
// -180..+180 degrees and wraps at the same point the sensor does.
static int16_t angleToInt16(uint16_t counts) {
  return (int16_t)((uint16_t)counts << 4);
}

static float countsToDegrees(uint16_t counts) {
  return counts * (360.0f / 4096.0f);
}

static void printPaddedHex(uint16_t value, uint8_t digits) {
  for (uint8_t i = digits; i > 1; i--) {
    if (value < ((uint32_t)1 << (4 * (i - 1)))) {
      Serial.print('0');
    }
  }
  Serial.print(value, HEX);
}

// ---------------------------------------------------------------- CONF decode

static void printConf(uint16_t conf) {
  Serial.print(F("CONF     = 0x"));
  printPaddedHex(conf, 4);
  Serial.println();

  Serial.print(F("  PM   (power mode)      = "));
  switch (conf & 0x03) {
    case 0: Serial.println(F("NOM")); break;
    case 1: Serial.println(F("LPM1")); break;
    case 2: Serial.println(F("LPM2")); break;
    default: Serial.println(F("LPM3")); break;
  }

  Serial.print(F("  HYST (hysteresis)      = "));
  switch ((conf >> 2) & 0x03) {
    case 0: Serial.println(F("OFF")); break;
    case 1: Serial.println(F("1 LSB")); break;
    case 2: Serial.println(F("2 LSBs")); break;
    default: Serial.println(F("3 LSBs")); break;
  }

  Serial.print(F("  OUTS (output stage)    = "));
  switch ((conf >> 4) & 0x03) {
    case 0: Serial.println(F("analog, full range 0..100% VDD")); break;
    case 1: Serial.println(F("analog, reduced range 10..90% VDD")); break;
    case 2: Serial.println(F("digital PWM")); break;
    default: Serial.println(F("reserved")); break;
  }

  Serial.print(F("  PWMF (PWM frequency)   = "));
  switch ((conf >> 6) & 0x03) {
    case 0: Serial.println(F("115 Hz")); break;
    case 1: Serial.println(F("230 Hz")); break;
    case 2: Serial.println(F("460 Hz")); break;
    default: Serial.println(F("920 Hz")); break;
  }

  Serial.print(F("  SF   (slow filter)     = "));
  switch ((conf >> 8) & 0x03) {
    case 0: Serial.println(F("16x")); break;
    case 1: Serial.println(F("8x")); break;
    case 2: Serial.println(F("4x")); break;
    default: Serial.println(F("2x")); break;
  }

  Serial.print(F("  FTH  (fast filter thr) = "));
  switch ((conf >> 10) & 0x07) {
    case 0: Serial.println(F("slow filter only")); break;
    case 1: Serial.println(F("6 LSBs")); break;
    case 2: Serial.println(F("7 LSBs")); break;
    case 3: Serial.println(F("9 LSBs")); break;
    case 4: Serial.println(F("18 LSBs")); break;
    case 5: Serial.println(F("21 LSBs")); break;
    case 6: Serial.println(F("24 LSBs")); break;
    default: Serial.println(F("10 LSBs")); break;
  }

  Serial.print(F("  WD   (watchdog)        = "));
  Serial.println((conf & _BV(13)) ? F("ON") : F("OFF"));
}

// ------------------------------------------------------- one-shot config dump

static void printAngleRegister(const __FlashStringHelper *name, uint8_t reg) {
  uint16_t counts;
  Serial.print(name);
  if (!read12(reg, &counts)) {
    Serial.print(F("<read failed: "));
    Serial.print(i2cErrorText(g_i2cError));
    Serial.println('>');
    return;
  }
  Serial.print(counts);
  Serial.print(F(" counts ("));
  Serial.print(countsToDegrees(counts), 2);
  Serial.println(F(" deg)"));
}

static void dumpConfiguration() {
  Serial.println(F("--- AS5600 configuration -------------------------------"));

  uint8_t zmco;
  Serial.print(F("ZMCO     = "));
  if (read8(REG_ZMCO, &zmco)) {
    Serial.print(zmco & 0x03);
    Serial.println(F(" (times ZPOS/MPOS have been burned; max 3)"));
  } else {
    Serial.println(i2cErrorText(g_i2cError));
  }

  printAngleRegister(F("ZPOS     = "), REG_ZPOS_H);
  printAngleRegister(F("MPOS     = "), REG_MPOS_H);
  printAngleRegister(F("MANG     = "), REG_MANG_H);

  // CONF is 14 bits wide, so read the two bytes directly rather than via read12().
  uint8_t confBuf[2];
  if (readRegs(REG_CONF_H, confBuf, 2)) {
    printConf(((((uint16_t)confBuf[0] << 8) | confBuf[1]) & 0x3FFF));
  } else {
    Serial.print(F("CONF     = <read failed: "));
    Serial.print(i2cErrorText(g_i2cError));
    Serial.println('>');
  }
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("Send any character to re-dump the configuration."));
  Serial.println();
}

// ------------------------------------------------------------------- telemetry

static void printTelemetry() {
  uint16_t rawAngle, angle, magnitude;
  uint8_t status, agc;

  bool ok = read12(REG_RAWANG_H, &rawAngle);
  uint8_t err = g_i2cError;
  ok &= read12(REG_ANGLE_H, &angle);
  if (g_i2cError) err = g_i2cError;
  ok &= read8(REG_STATUS, &status);
  if (g_i2cError) err = g_i2cError;
  ok &= read8(REG_AGC, &agc);
  if (g_i2cError) err = g_i2cError;
  ok &= read12(REG_MAG_H, &magnitude);
  if (g_i2cError) err = g_i2cError;

  if (!ok) {
    Serial.print(F("I2C ERROR: "));
    Serial.println(i2cErrorText(err));
    return;
  }

  Serial.print(F("RAW="));
  Serial.print(rawAngle);
  Serial.print(F("  ANGLE="));
  Serial.print(angle);
  Serial.print(F("  int16="));
  Serial.print(angleToInt16(angle));       // Q15 turns, +/-180 deg full scale
  Serial.print(F("  deg="));
  Serial.print(countsToDegrees(angle), 2);

  Serial.print(F("  AGC="));
  Serial.print(agc);
  Serial.print('/');
  Serial.print(AGC_FULL_SCALE);
  Serial.print(F("  MAG="));
  Serial.print(magnitude);

  Serial.print(F("  STATUS=0x"));
  printPaddedHex(status, 2);
  Serial.print(F(" ["));
  Serial.print((status & STATUS_MD) ? F("MD") : F("--"));
  Serial.print((status & STATUS_ML) ? F(" ML") : F(" --"));
  Serial.print((status & STATUS_MH) ? F(" MH") : F(" --"));
  Serial.print(']');

  // Warnings and errors, spelled out.
  if (!(status & STATUS_MD)) {
    Serial.print(F("  ERROR: no magnet detected"));
  }
  if (status & STATUS_ML) {
    Serial.print(F("  WARN: AGC max gain overflow, magnet too weak / airgap too large"));
  }
  if (status & STATUS_MH) {
    Serial.print(F("  WARN: AGC min gain overflow, magnet too strong / airgap too small"));
  }
  if ((status & STATUS_MD) && !(status & (STATUS_ML | STATUS_MH))) {
    // In range, but flag a gain that is far off centre: the airgap wants tuning.
    uint8_t quarter = AGC_FULL_SCALE / 4;
    if (agc < quarter) {
      Serial.print(F("  NOTE: AGC low, consider increasing airgap"));
    } else if (agc > (uint8_t)(AGC_FULL_SCALE - quarter)) {
      Serial.print(F("  NOTE: AGC high, consider reducing airgap"));
    }
  }
  Serial.println();
}

// ------------------------------------------------------------------- Arduino

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // harmless on the UNO, needed on native-USB boards
  }

  Wire.begin();
  Wire.setClock(400000);  // AS5600 supports fast mode

  Serial.println();
  Serial.println(F("AS5600 bring-up"));
  Serial.print(F("I2C address 0x"));
  Serial.println(AS5600_ADDR, HEX);
  Serial.print(F("Supply assumed: "));
  Serial.println(AS5600_VDD_5V ? F("5V (AGC 0..255)") : F("3.3V (AGC 0..128)"));

  // Probe before doing anything else: a NACK here means wiring or pull-ups.
  Wire.beginTransmission(AS5600_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(F("Device did not ACK: "));
    Serial.println(i2cErrorText(err));
    Serial.println(F("Check SDA/SCL wiring, pull-ups and supply; retrying in loop()."));
  } else {
    Serial.println(F("Device ACKed."));
  }

  dumpConfiguration();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    dumpConfiguration();
  }

  printTelemetry();
  delay(200);  // 5 Hz
}
