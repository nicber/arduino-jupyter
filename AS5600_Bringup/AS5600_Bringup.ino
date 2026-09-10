// Sketch de puesta en marcha del AS5600
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)
//
// Conexionado: SDA -> A4, SCL -> A5, VDD -> 5V, GND -> GND
// (Las plaquetas de AS5600 traen su propio regulador de 3V3 y las resistencias
//  de pull-up del bus. Un AS5600 pelado en modo 3,3 V necesita adaptación de
//  niveles y pull-ups de 4,7k a 3V3.)
//
// El mapa de registros, los campos de CONF y los bits de STATUS siguen la hoja
// de datos del AS5600 [v1-06] 2018-Jun-20, Figuras 21/22/23
// (Docs/infineon-as5600-datasheet-en.pdf).
//
// Salida: 115200 baudios. La configuración se vuelca al arrancar (y de nuevo
// cada vez que se envía un carácter por el monitor serie); la telemetría sale a
// 5 Hz.

#include <Wire.h>
#include <BoardStart.h>

// Poner en 0 si el AS5600 se alimenta con 3,3 V: el rango del AGC se reduce a la
// mitad.
#define AS5600_VDD_5V 1

static const uint8_t AS5600_ADDR = 0x36;  // 7 bits, 0110110b

// Direcciones de registro (byte alto primero en los registros de 12 bits).
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

// Bits del registro STATUS (Figura 23).
static const uint8_t STATUS_MH = _BV(3);  // desborde de ganancia mínima del AGC, imán muy fuerte
static const uint8_t STATUS_ML = _BV(4);  // desborde de ganancia máxima del AGC, imán muy débil
static const uint8_t STATUS_MD = _BV(5);  // se detectó el imán

// El AGC tiene que quedar cerca del medio de su rango; el entrehierro se ajusta
// hasta llegar ahí.
#if AS5600_VDD_5V
static const uint8_t AGC_FULL_SCALE = 255;
#else
static const uint8_t AGC_FULL_SCALE = 128;
#endif

// Último error de I2C, para que cada lectura pueda informar por qué falló en
// lugar de devolver un cero mudo. 0 = OK, 1..4 = códigos de
// Wire.endTransmission(), 5 = lectura corta.
static uint8_t g_i2cError = 0;

static const __FlashStringHelper *i2cErrorText(uint8_t err) {
  switch (err) {
    case 0: return F("ok");
    case 1: return F("datos demasiado largos para el buffer");
    case 2: return F("NACK en la direccion (el dispositivo no responde)");
    case 3: return F("NACK en los datos");
    case 4: return F("error de bus");
    case 5: return F("lectura corta (menos bytes de los pedidos)");
    default: return F("desconocido");
  }
}

// Lee `count` bytes a partir de `reg`. Devuelve false y carga g_i2cError si falla.
static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t count) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();  // STOP, y despues un START nuevo para la lectura
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

// Registros de 12 bits: byte alto primero, el nibble superior del byte alto no
// se usa.
static bool read12(uint8_t regHigh, uint16_t *value) {
  uint8_t buf[2];
  if (!readRegs(regHigh, buf, 2)) {
    return false;
  }
  *value = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
  return true;
}

// 0..4095 cuentas -> rango completo de int16_t, es decir vueltas en Q15:
// -32768..32767 mapea a -180..+180 grados y da la vuelta en el mismo punto que
// el sensor.
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

// ------------------------------------------------------- decodificación de CONF

static void printConf(uint16_t conf) {
  Serial.print(F("CONF     = 0x"));
  printPaddedHex(conf, 4);
  Serial.println();

  Serial.print(F("  PM   (modo de consumo)   = "));
  switch (conf & 0x03) {
    case 0: Serial.println(F("NOM")); break;
    case 1: Serial.println(F("LPM1")); break;
    case 2: Serial.println(F("LPM2")); break;
    default: Serial.println(F("LPM3")); break;
  }

  Serial.print(F("  HYST (histeresis)        = "));
  switch ((conf >> 2) & 0x03) {
    case 0: Serial.println(F("APAGADA")); break;
    case 1: Serial.println(F("1 LSB")); break;
    case 2: Serial.println(F("2 LSBs")); break;
    default: Serial.println(F("3 LSBs")); break;
  }

  Serial.print(F("  OUTS (etapa de salida)   = "));
  switch ((conf >> 4) & 0x03) {
    case 0: Serial.println(F("analogica, rango completo 0..100% VDD")); break;
    case 1: Serial.println(F("analogica, rango reducido 10..90% VDD")); break;
    case 2: Serial.println(F("PWM digital")); break;
    default: Serial.println(F("reservado")); break;
  }

  Serial.print(F("  PWMF (frecuencia de PWM) = "));
  switch ((conf >> 6) & 0x03) {
    case 0: Serial.println(F("115 Hz")); break;
    case 1: Serial.println(F("230 Hz")); break;
    case 2: Serial.println(F("460 Hz")); break;
    default: Serial.println(F("920 Hz")); break;
  }

  Serial.print(F("  SF   (filtro lento)      = "));
  switch ((conf >> 8) & 0x03) {
    case 0: Serial.println(F("16x")); break;
    case 1: Serial.println(F("8x")); break;
    case 2: Serial.println(F("4x")); break;
    default: Serial.println(F("2x")); break;
  }

  Serial.print(F("  FTH  (umbral f. rapido)  = "));
  switch ((conf >> 10) & 0x07) {
    case 0: Serial.println(F("solo filtro lento")); break;
    case 1: Serial.println(F("6 LSBs")); break;
    case 2: Serial.println(F("7 LSBs")); break;
    case 3: Serial.println(F("9 LSBs")); break;
    case 4: Serial.println(F("18 LSBs")); break;
    case 5: Serial.println(F("21 LSBs")); break;
    case 6: Serial.println(F("24 LSBs")); break;
    default: Serial.println(F("10 LSBs")); break;
  }

  Serial.print(F("  WD   (perro guardian)    = "));
  Serial.println((conf & _BV(13)) ? F("ENCENDIDO") : F("APAGADO"));
}

// ------------------------------------- volcado de configuración, una sola vez

static void printAngleRegister(const __FlashStringHelper *name, uint8_t reg) {
  uint16_t counts;
  Serial.print(name);
  if (!read12(reg, &counts)) {
    Serial.print(F("<fallo la lectura: "));
    Serial.print(i2cErrorText(g_i2cError));
    Serial.println('>');
    return;
  }
  Serial.print(counts);
  Serial.print(F(" cuentas ("));
  Serial.print(countsToDegrees(counts), 2);
  Serial.println(F(" grados)"));
}

static void dumpConfiguration() {
  Serial.println(F("--- configuracion del AS5600 ---------------------------"));

  uint8_t zmco;
  Serial.print(F("ZMCO     = "));
  if (read8(REG_ZMCO, &zmco)) {
    Serial.print(zmco & 0x03);
    Serial.println(F(" (veces que se grabaron ZPOS/MPOS; maximo 3)"));
  } else {
    Serial.println(i2cErrorText(g_i2cError));
  }

  printAngleRegister(F("ZPOS     = "), REG_ZPOS_H);
  printAngleRegister(F("MPOS     = "), REG_MPOS_H);
  printAngleRegister(F("MANG     = "), REG_MANG_H);

  // CONF tiene 14 bits de ancho, asi que se leen los dos bytes directamente en
  // lugar de usar read12().
  uint8_t confBuf[2];
  if (readRegs(REG_CONF_H, confBuf, 2)) {
    printConf(((((uint16_t)confBuf[0] << 8) | confBuf[1]) & 0x3FFF));
  } else {
    Serial.print(F("CONF     = <fallo la lectura: "));
    Serial.print(i2cErrorText(g_i2cError));
    Serial.println('>');
  }
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("Enviar cualquier caracter para volcar de nuevo la configuracion."));
  Serial.println();
}

// ------------------------------------------------------------------ telemetría

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
    Serial.print(F("ERROR DE I2C: "));
    Serial.println(i2cErrorText(err));
    return;
  }

  Serial.print(F("RAW="));
  Serial.print(rawAngle);
  Serial.print(F("  ANGLE="));
  Serial.print(angle);
  Serial.print(F("  int16="));
  Serial.print(angleToInt16(angle));       // vueltas en Q15, +/-180 grados a fondo de escala
  Serial.print(F("  grados="));
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

  // Advertencias y errores, dichos con todas las letras.
  if (!(status & STATUS_MD)) {
    Serial.print(F("  ERROR: no se detecta el iman"));
  }
  if (status & STATUS_ML) {
    Serial.print(F("  AVISO: desborde de ganancia maxima del AGC, iman muy debil / entrehierro muy grande"));
  }
  if (status & STATUS_MH) {
    Serial.print(F("  AVISO: desborde de ganancia minima del AGC, iman muy fuerte / entrehierro muy chico"));
  }
  if ((status & STATUS_MD) && !(status & (STATUS_ML | STATUS_MH))) {
    // Está en rango, pero se marca una ganancia muy corrida del centro: el
    // entrehierro pide ajuste.
    uint8_t quarter = AGC_FULL_SCALE / 4;
    if (agc < quarter) {
      Serial.print(F("  NOTA: AGC bajo, conviene aumentar el entrehierro"));
    } else if (agc > (uint8_t)(AGC_FULL_SCALE - quarter)) {
      Serial.print(F("  NOTA: AGC alto, conviene reducir el entrehierro"));
    }
  }
  Serial.println();
}

// ------------------------------------------------------------------- Arduino

void setup() {
  // Antes del Serial: a 4 MHz este println saldria a 28800 y el monitor
  // mostraria basura en vez del diagnostico. Ver BoardStart.h.
  boardClockBegin();

  Serial.begin(115200);
  while (!Serial) {
    ;  // inofensivo en el UNO, necesario en placas con USB nativo
  }

  // Si el reset que trajo hasta aca cayo en medio de una lectura, el sensor
  // quedo esperando pulsos de reloj y sujetando SDA, y Wire se colgaria en la
  // primera transferencia sin decir nada. Ver BoardStart.h.
  uint8_t pulsos = i2cBusRecover();

  Wire.begin();
  Wire.setClock(400000);  // el AS5600 soporta modo rapido

  Serial.println();
  Serial.println(F("Puesta en marcha del AS5600"));
  if (pulsos) {
    Serial.print(F("El bus estaba tomado; se libero con "));
    Serial.print(pulsos);
    Serial.println(F(" pulso(s) de reloj."));
  }
  Serial.print(F("Direccion I2C 0x"));
  Serial.println(AS5600_ADDR, HEX);
  Serial.print(F("Alimentacion supuesta: "));
  Serial.println(AS5600_VDD_5V ? F("5V (AGC 0..255)") : F("3.3V (AGC 0..128)"));

  // Sondear antes que nada: un NACK acá es cableado o pull-ups.
  Wire.beginTransmission(AS5600_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(F("El dispositivo no dio ACK: "));
    Serial.println(i2cErrorText(err));
    Serial.println(F("Revisar el cableado de SDA/SCL, los pull-ups y la alimentacion; se reintenta en loop()."));
  } else {
    Serial.println(F("El dispositivo dio ACK."));
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
