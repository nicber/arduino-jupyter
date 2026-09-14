// Experimento: el ADC contra la fase del PWM.
//
// Existe para decidir cómo medir la corriente en Banco. Guarda ráfagas de
// conversiones seguidas, cada una con la cuenta del Timer1 --que es la fase del
// PWM-- tomada en la interrupción de fin de conversión. Con eso la computadora
// reconstruye la forma de onda de la corriente dentro del período y emula sobre
// los mismos datos cualquier manera de muestrear y filtrar.
//
// No usa CtrlLink ni el AS5600: el Timer2 queda quieto, y durante la ráfaga se
// apaga también la interrupción del Timer0, para que nada más le mueva el instante
// a la interrupción del ADC. Protocolo de líneas, a 1 Mbaud:
//
//   u <n>      comando sobre el actuador, -255..255 (un solo cuadrante)
//   p <n>      bits del preescalador del ADC, 1..7 (7 = /128)
//   m <n>      disparo: 0 libre, 1 fondo del Timer1, 2 comparación B, 3 tope
//   o <n>      OCR1B, para el disparo por comparación B
//   c <n>      canal del multiplexor, 0..31
//   q <0|1>    onda de prueba con el DAC interno: sube en el fondo del Timer1 y
//              baja en el tope
//   b <n>      una ráfaga de n conversiones (hasta MAX_N), cada una con su fase
//   B <n>      lo mismo sin la fase, de a 2 bytes y hasta 2*MAX_N: con el ADC libre
//              la fase sale del índice, y basta la de la primera
//   x <0|1|2>  1: calcular un filtro de dos polos en la interrupción y guardar su
//              duración en lugar de la fase; 2: sólo acumular la fila
//   w <n>      estados estáticos de los pines del actuador, sin PWM (ver command())
//   s <0|1|2>  interrupción periódica a 5 kHz: 1 vacía, 2 con el I2C del AS5600
//   r <0..4>   referencia del ADC: AVCC, interna de 4,096 V, de 2,048 V, de 1,024 V,
//              o 4: el pin AREF, con ADTM apagado
//   v <n>      fuente del divisor interno (VDS): 0 apagado, 6 la alimentación
//   a <0|1>    apagar o prender el ADC
//   l          vueltas de un bucle vacío en 500 ms, para medir la carga
//   i          lo que está puesto
//
// La ráfaga vuelve como una línea "# b ...", los bytes crudos y "# end". Cada
// muestra son dos bytes de la conversión y dos de la cuenta del Timer1, con el bit
// 15 en 1 si el contador bajaba: en modo phase-correct la cuenta sola no dice de
// qué mitad del período es.
//
// Lo medido en el clon con LGT8F328P: una conversión libre dura 22 relojes del ADC
// (176 us a /128), la entrada queda retenida unos 15 relojes antes del fin, y el
// disparo automático por el Timer1 (ADTS) no convierte. La hoja de datos pide un
// reloj del ADC de 300 kHz a 3 MHz, o sea /16 a /64 a 16 MHz.

#include <util/atomic.h>
#include <util/delay.h>

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <BoardStart.h>
#include <HBridge.h>
#include <SampleClock.h>

// Un muestreador a 5 kHz como el de Banco: `s 1` prende la interrupción periódica
// sola, `s 2` además las transferencias I2C al AS5600, como en Banco.
typedef AS5600<NI2CBus> Sensor;
static SampleClock g_clock(10);
static uint8_t     g_tick = 0;

ISR(TIMER2_COMPA_vect)
{
    if (g_tick == 2) Sensor::do_transfer();
    g_clock.on_isr();
}

static const uint32_t BAUD    = 1000000;
static const uint16_t PWM_TOP = 8000;       // 1 kHz, como Banco
static const uint16_t MAX_N   = 128;   // el I2C del AS5600 también quiere RAM

typedef HBridge<9, 6, 7> Motor;

static Motor g_motor(PWM_TOP);

static uint8_t           g_buf[4 * MAX_N];
static volatile uint16_t g_n     = 0;
static volatile uint16_t g_want  = 0;
static volatile uint16_t g_first = 0;       // la cuenta del Timer1 de la primera muestra
static uint8_t           g_mode  = 0;
static uint8_t           g_wide  = 4;       // bytes por muestra: 4, o 2 sin la fase
static uint16_t          g_full  = 1024;

// Lo que costaría filtrar en la interrupción, para medirlo: dos polos por
// corrimiento con 8 bits de guarda, y un acumulador de caja. Con `g_costo` en 1 la
// ráfaga guarda, en lugar de la fase, cuántas cuentas del Timer1 (62,5 ns) pasaron
// entre la entrada y la salida del cálculo.
static uint8_t  g_costo = 0;
static uint8_t  g_k     = 4;
static int32_t  g_y1 = 0, g_y2 = 0;
static uint32_t g_caja = 0;
static uint16_t g_cajan = 0;

ISR(ADC_vect)
{
    const uint16_t t1 = TCNT1;
    const uint16_t t2 = TCNT1;
    const uint16_t a  = ADC;

    uint16_t dur = 0;
    if (g_costo == 2)
    {
        // Sólo el acumulador de una fila: lo mínimo para promediar.
        g_caja += a;
        g_cajan++;
    }
    else if (g_costo)
    {
        const int32_t x = (int32_t)a << 8;
        g_y1 += (x - g_y1) >> g_k;
        g_y2 += (g_y1 - g_y2) >> g_k;
        g_caja += a;
        if (++g_cajan == 909) { g_cajan = 0; g_caja = 0; }
        const uint16_t t3 = TCNT1;
        dur = (uint16_t)((t3 > t2) ? (t3 - t2) : (t2 - t3));
    }

    const uint16_t n = g_n;
    if (n < g_want)
    {
        const uint16_t t = g_costo ? dur : (uint16_t)(t1 | ((t2 < t1) ? 0x8000u : 0u));
        uint8_t* p = &g_buf[g_wide * n];
        p[0] = (uint8_t)(a >> 8);
        p[1] = (uint8_t)a;
        if (g_wide == 4)
        {
            p[2] = (uint8_t)(t >> 8);
            p[3] = (uint8_t)t;
        }
        if (n == 0) g_first = t;
        g_n = n + 1;
    }
}

static uint8_t g_presc = 7;
static uint8_t g_chan  = 0;

// La referencia del ADC y la fuente del divisor interno, en el LGT8F328P (ADCSRD en
// 0xAD: BGEN bit 7, REFS2 bit 6, VDS bits 2..0). `g_ref`: 0 AVCC, 1 interna de
// 4,096 V, 2 interna de 2,048 V. `g_vds`: 0 divisor apagado, 6 la alimentación del
// micro, que se lee por los canales 8 (1/5) y 14 (4/5).
#define LGT_ADCSRD _SFR_MEM8(0xAD)
#define LGT_VCAL   _SFR_MEM8(0xC8)
#define LGT_VCAL2  _SFR_MEM8(0xCE)
#define LGT_VCAL3  _SFR_MEM8(0xCC)
static uint8_t g_ref = 0;
static uint8_t g_vds = 0;

// El disparo automático del ADC por las banderas del Timer1 (ADTS) no convierte
// nunca en el LGT8F328P, así que los disparos por temporizador son por software:
// la interrupción del Timer1 escribe ADSC. Cuesta unos µs de latencia fija, y la
// conversión arranca en el flanco siguiente del reloj del ADC.
static inline void start_conversion(void) { ADCSRA |= _BV(ADSC); }

// La onda de prueba, con el DAC interno del LGT8F328P (DACON en 0xA0, DALR en
// 0xA1): sube en el fondo del contador y baja en el tope. Leída con el ADC libre
// sobre el canal del DAC, dice cuánto antes del fin de la conversión se toma la
// muestra, que es lo que falta para ubicar cada muestra en el PWM. Adentro del
// chip: no maneja ningún pin.
#define LGT_DACON _SFR_MEM8(0xA0)
#define LGT_DALR  _SFR_MEM8(0xA1)

static uint8_t g_square = 0;

ISR(TIMER1_OVF_vect)
{
    if (g_square) LGT_DALR = 255;
    if (g_mode == 1) start_conversion();
}
ISR(TIMER1_COMPB_vect) { start_conversion(); }
ISR(TIMER1_CAPT_vect)
{
    if (g_square) LGT_DALR = 0;
    if (g_mode == 3) start_conversion();
}

static void adc_apply(void)
{
    ADCSRA = 0;
    TIMSK1 = 0;
    uint8_t refs = _BV(REFS0);                                  // AVCC
    if (g_full >= 4096)
    {
        uint8_t d = (uint8_t)(LGT_ADCSRD & ~(0xC0 | 0x07));
        d |= (uint8_t)(g_vds & 0x07);
        if (g_ref == 1) { d |= 0xC0; refs = 0;                         LGT_VCAL = LGT_VCAL3; }
        if (g_ref == 2) { d |= 0x80; refs = _BV(REFS1);                LGT_VCAL = LGT_VCAL2; }
        if (g_ref == 3) { d |= 0x80; refs = _BV(REFS1) | _BV(REFS0);   LGT_VCAL = _SFR_MEM8(0xCD); }
        // AREF externo: REFS2 y REFS en cero, y ADTM (bit 0 de ADCSRC, 0x7D) apagado,
        // que si no el chip saca su referencia interna por el mismo pin.
        if (g_ref == 4) { refs = 0; _SFR_MEM8(0x7D) &= (uint8_t)~0x01; }
        LGT_ADCSRD = d;
        // Lo que hace analogReference() del core lgt8fx para cualquier referencia
        // interna: sin esto REFS2 y VCAL no eligen nada.
        if (g_ref && g_ref != 4) LGT_DACON = (uint8_t)((LGT_DACON & 0x0C) | 0x02);
        else       LGT_DACON = (uint8_t)(LGT_DACON & 0x0C);
    }
    ADMUX  = (uint8_t)(refs | (g_chan & 0x1F));
    ADCSRB = (uint8_t)(ADCSRB & ~0x07);

    TIFR1  = _BV(TOV1) | _BV(OCF1B) | _BV(ICF1);
    const uint8_t square = g_square ? (uint8_t)(_BV(TOIE1) | _BV(ICIE1)) : 0;

    if (g_mode == 0)
    {
        ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | _BV(ADATE) | (g_presc & 0x07));
        ADCSRA |= _BV(ADSC);
        TIMSK1 = square;
        return;
    }

    ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | (g_presc & 0x07));
    TIMSK1 = (uint8_t)(square |
             ((g_mode == 1) ? _BV(TOIE1) : (g_mode == 2) ? _BV(OCIE1B) : _BV(ICIE1)));
}

static void info(void)
{
    Serial.print(F("# i full="));  Serial.print(g_full);
    Serial.print(F(" top="));      Serial.print(PWM_TOP);
    Serial.print(F(" u="));        Serial.print(g_motor.u);
    Serial.print(F(" ocr1a="));    Serial.print(OCR1A);
    Serial.print(F(" ocr1b="));    Serial.print(OCR1B);
    Serial.print(F(" mode="));     Serial.print(g_mode);
    Serial.print(F(" presc="));    Serial.print(g_presc);
    Serial.print(F(" chan="));     Serial.print(g_chan);
    Serial.print(F(" adcsra="));   Serial.print(ADCSRA, HEX);
    Serial.print(F(" adcsrb="));   Serial.println(ADCSRB, HEX);
}

static void burst(uint16_t n, uint8_t wide)
{
    g_wide = wide;
    const uint16_t max = (uint16_t)(4 * MAX_N / wide);
    if (n > max) n = max;

    // La primera conversión puede venir de antes de pedir: se descarta una.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_n = 0; g_want = 0; }
    Serial.flush();

    const uint8_t timsk0 = TIMSK0;
    TIMSK0 = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_n = 0; g_want = n; }

    // Con el Timer0 apagado no hay millis() ni delay(): se espera contando.
    for (uint16_t k = 0; k < 2000; k++)
    {
        uint16_t got;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { got = g_n; }
        if (got >= n) break;
        _delay_ms(1);
    }

    TIMSK0 = timsk0;

    uint16_t got;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { got = g_n; g_want = 0; }

    Serial.print(F("# b n="));   Serial.print(got);
    Serial.print(F(" u="));      Serial.print(g_motor.u);
    Serial.print(F(" ocr1a="));  Serial.print(OCR1A);
    Serial.print(F(" mode="));   Serial.print(g_mode);
    Serial.print(F(" presc="));  Serial.print(g_presc);
    Serial.print(F(" chan="));   Serial.print(g_chan);
    Serial.print(F(" wide="));   Serial.print(wide);
    Serial.print(F(" first="));  Serial.println(g_first);
    Serial.write(g_buf, wide * got);
    Serial.println();
    Serial.println(F("# end"));
}

static char    g_line[24];
static uint8_t g_len = 0;

static void command(char* s)
{
    const char     c = s[0];
    const long     v = atol(s + 1);

    switch (c)
    {
        case 'u': g_motor.write((Motor::Command)v); break;
        case 'p': g_presc = (uint8_t)v; adc_apply(); break;
        case 'm': g_mode  = (uint8_t)v; adc_apply(); break;
        case 'c': g_chan  = (uint8_t)v; adc_apply(); break;
        case 'o': OCR1B   = (uint16_t)v; break;
        case 'x': g_costo = (uint8_t)v; break;
        case 's': g_tick = (uint8_t)v; TIMSK2 = g_tick ? _BV(OCIE2A) : 0; break;
        case 'w':
            // Estados estáticos de los pines del actuador, sin PWM: para ver cuánto
            // corre A0 cada uno. 1 pin 9 en bajo, 2 en alto, 3 entrada con pull-up
            // (corriente de base unas 40 veces menor), 4 pin 6 en alto, 5 pin 7 en
            // alto. 0 vuelve al PWM con el último comando.
            TCCR1A &= (uint8_t)~_BV(COM1A1);
            pinMode(9, OUTPUT); digitalWrite(9, LOW);
            digitalWrite(6, LOW); digitalWrite(7, LOW);
            if (v == 2) digitalWrite(9, HIGH);
            if (v == 3) pinMode(9, INPUT_PULLUP);
            if (v == 4) digitalWrite(6, HIGH);
            if (v == 5) digitalWrite(7, HIGH);
            if (v == 0) { const Motor::Command u = g_motor.u; g_motor.write(0); g_motor.write(u); }
            break;
        case 'r': g_ref   = (uint8_t)v; adc_apply(); break;
        case 'v': g_vds   = (uint8_t)v; adc_apply(); break;
        case 'a': ADCSRA = (uint8_t)(v ? ADCSRA : 0); if (v) adc_apply(); break;
        case 'l':
        {
            // Vueltas de un bucle vacío en 500 ms: la CPU que dejan las interrupciones.
            uint32_t vueltas = 0;
            const uint32_t t0 = millis();
            while (millis() - t0 < 500) vueltas++;
            Serial.print(F("# i vueltas=")); Serial.println(vueltas);
            return;
        }
        case 'q':
            // Sólo en la placa de 12 bits: en un UNO 0xA0 no es un registro.
            if (g_full < 4096) { Serial.println(F("# err")); return; }
            g_square = (uint8_t)(v != 0);
            if (g_square) LGT_DACON = (uint8_t)((LGT_DACON & ~0x03) | 0x08);   // DACEN, contra VCC
            else          LGT_DACON = (uint8_t)(LGT_DACON & ~0x08);
            adc_apply();
            break;
        case 'b': burst((uint16_t)v, 4); return;
        case 'B': burst((uint16_t)v, 2); return;
        case 'i': info(); return;
        default:  Serial.println(F("# err")); return;
    }
    Serial.println(F("# ok"));
}

void setup()
{
    board::clock_begin();
    g_full = board::adc_full_scale();
    board::adc_select_vcc(g_full >= 4096);

    board::bus_recover();

    g_motor.begin();
    g_motor.write(0);

    Sensor::begin();
    g_clock.begin(5000);
    TIMSK2 = 0;

    Serial.begin(BAUD);
    adc_apply();

    Serial.println(F("# AdcFase listo"));
}

void loop()
{
    while (Serial.available())
    {
        const char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r')
        {
            if (g_len)
            {
                g_line[g_len] = 0;
                command(g_line);
                g_len = 0;
            }
        }
        else if (g_len < sizeof(g_line) - 1)
        {
            g_line[g_len++] = ch;
        }
    }
}
