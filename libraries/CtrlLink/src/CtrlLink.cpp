#include "CtrlLink.h"

#include <stdlib.h>
#include <string.h>

// ------------------------------------------------------------ almacenamiento

const CtrlParam*   CtrlLink::m_params        = 0;
const CtrlChannel* CtrlLink::m_channels      = 0;
uint8_t            CtrlLink::m_param_count   = 0;
uint8_t            CtrlLink::m_channel_count = 0;

const __FlashStringHelper* CtrlLink::m_id = 0;

char    CtrlLink::m_cmd[CTRL_CMD_LEN];
uint8_t CtrlLink::m_cmd_len      = 0;
bool    CtrlLink::m_cmd_overflow = false;

bool     CtrlLink::m_streaming = false;
bool     CtrlLink::m_usable    = false;
uint16_t CtrlLink::m_tick      = 0;
uint16_t CtrlLink::m_decimate  = 1;
uint16_t CtrlLink::m_dec_count = 0;
uint32_t CtrlLink::m_dt_us     = 0;
uint32_t CtrlLink::m_rows      = 0;
uint32_t CtrlLink::m_drops     = 0;
uint16_t CtrlLink::m_writes    = 0;

#ifdef CTRL_PROFILE
CtrlProfile g_ctrl_profile = { 0, 0, false, 0, 0 };
#endif

const CtrlParam CtrlLink::k_builtin[] PROGMEM =
{
    { "dec", CTRL_U16, (void*)&CtrlLink::m_decimate, 0 },
};

// ------------------------------------------------------------------ auxiliares

static const char HEXDIGITS[] PROGMEM = "0123456789ABCDEF";

// Escribe `digits` nibbles de `value`, el más significativo primero, y devuelve
// la posición justo después de ellos. Los nibbles por encima de `digits` se
// descartan, que es lo que hace que un valor angosto con signo extendido salga
// bien.
//
// `digits` tiene que ser par: el valor se consume de a un byte porque el AVR no
// tiene desplazador de barril, así que un `>>= 4` de 32 bits compila a un bucle
// de cuatro pasadas de lsr/ror/ror/ror —24 ciclos por nibble, más que todo el
// resto del trabajo junto—. Desplazar de a 8 son apenas movimientos entre
// registros, y los dos nibbles de un byte salen con un andi y un swap. Todos los
// anchos que devuelve hex_width() son pares, así que nada acá necesita una
// cantidad impar.
static char* put_hex(char* p, uint32_t value, uint8_t digits)
{
    char* end = p + digits;
    char* q   = end;

    while (digits >= 2)
    {
        uint8_t byte = (uint8_t)value;

        *--q = (char)pgm_read_byte(&HEXDIGITS[byte & 0x0F]);
        *--q = (char)pgm_read_byte(&HEXDIGITS[byte >> 4]);

        value >>= 8;
        digits -= 2;
    }

    return end;
}

// Lee una variable de tipo `type` en los bits bajos de un uint32_t. Un float se
// reinterpreta en lugar de convertirse: sus cuatro bytes salen tal cual, y la
// computadora los lee como IEEE 754 big-endian.
static uint32_t read_value(const void* addr, uint8_t type)
{
    switch (type)
    {
        case CTRL_I8:  return (uint32_t)(uint8_t) *(const int8_t*)addr;
        case CTRL_U8:  return (uint32_t)          *(const uint8_t*)addr;
        case CTRL_I16: return (uint32_t)(uint16_t)*(const int16_t*)addr;
        case CTRL_U16: return (uint32_t)          *(const uint16_t*)addr;
        case CTRL_F32: { uint32_t bits; memcpy(&bits, addr, 4); return bits; }
        default:       return                     *(const uint32_t*)addr;
    }
}

static void write_value(void* addr, uint8_t type, const char* text)
{
    switch (type)
    {
        case CTRL_I8:  *(int8_t*)addr   = (int8_t)  strtol(text, 0, 0);  break;
        case CTRL_U8:  *(uint8_t*)addr  = (uint8_t) strtoul(text, 0, 0); break;
        case CTRL_I16: *(int16_t*)addr  = (int16_t) strtol(text, 0, 0);  break;
        case CTRL_U16: *(uint16_t*)addr = (uint16_t)strtoul(text, 0, 0); break;
        case CTRL_I32: *(int32_t*)addr  = (int32_t) strtol(text, 0, 0);  break;
        case CTRL_U32: *(uint32_t*)addr = (uint32_t)strtoul(text, 0, 0); break;
        default:       *(float*)addr    = (float)   atof(text);          break;
    }
}

// Compara un nombre en PROGMEM rellenado con NUL contra una cadena en RAM
// terminada en NUL.
static bool name_equals(const char* pgm_name, const char* text)
{
    for (uint8_t i = 0; i < CTRL_NAME_LEN; i++)
    {
        char c = (char)pgm_read_byte(pgm_name + i);

        if (c == '\0')
        {
            return *text == '\0';
        }
        if (*text != c)
        {
            return false;
        }
        text++;
    }

    // El nombre llenó el campo justo, así que no hay NUL contra el cual
    // comparar.
    return *text == '\0';
}

uint8_t CtrlLink::hex_width(uint8_t type)
{
    switch (type)
    {
        case CTRL_I8:
        case CTRL_U8:  return 2;
        case CTRL_I16:
        case CTRL_U16: return 4;
        default:       return 8;
    }
}

uint8_t CtrlLink::row_width(void)
{
    uint8_t width = 4 + 1;  // tick, fin de linea

    for (uint8_t i = 0; i < m_channel_count; i++)
    {
        width += hex_width(pgm_read_byte(&m_channels[i].type));
    }

    return width;
}

void CtrlLink::print_name(const char* pgm_name)
{
    for (uint8_t i = 0; i < CTRL_NAME_LEN; i++)
    {
        char c = (char)pgm_read_byte(pgm_name + i);

        if (c == '\0')
        {
            break;
        }
        Serial.write(c);
    }
}

void CtrlLink::print_type(uint8_t type)
{
    switch (type)
    {
        case CTRL_I8:  Serial.print(F("i8"));  break;
        case CTRL_U8:  Serial.print(F("u8"));  break;
        case CTRL_I16: Serial.print(F("i16")); break;
        case CTRL_U16: Serial.print(F("u16")); break;
        case CTRL_I32: Serial.print(F("i32")); break;
        case CTRL_U32: Serial.print(F("u32")); break;
        default:       Serial.print(F("f32")); break;
    }
}

void CtrlLink::error(const __FlashStringHelper* reason)
{
    Serial.print(F("# err "));
    Serial.println(reason);
}

void CtrlLink::note(const __FlashStringHelper* text)
{
    Serial.print(F("# note "));
    Serial.println(text);
}

// ------------------------------------------------------------------ parámetros

// Los incorporados ocupan los índices [0, K_BUILTIN_COUNT); la tabla del sketch
// va a continuación.
static const CtrlParam* param_at(const CtrlParam* user,
                                 const CtrlParam* builtin, uint8_t builtin_count,
                                 int16_t index)
{
    return (index < (int16_t)builtin_count) ? &builtin[index]
                                            : &user[index - builtin_count];
}

int16_t CtrlLink::find_param(const char* name)
{
    for (uint8_t i = 0; i < K_BUILTIN_COUNT; i++)
    {
        if (name_equals(k_builtin[i].name, name))
        {
            return (int16_t)i;
        }
    }

    for (uint8_t i = 0; i < m_param_count; i++)
    {
        if (name_equals(m_params[i].name, name))
        {
            return (int16_t)(K_BUILTIN_COUNT + i);
        }
    }

    return -1;
}

void CtrlLink::print_param_value(int16_t index)
{
    const CtrlParam* entry = param_at(m_params, k_builtin, K_BUILTIN_COUNT, index);

    uint8_t type = pgm_read_byte(&entry->type);
    void*   addr = (void*)pgm_read_word(&entry->addr);

    switch (type)
    {
        case CTRL_I8:  Serial.print(*(const int8_t*)addr);   break;
        case CTRL_U8:  Serial.print(*(const uint8_t*)addr);  break;
        case CTRL_I16: Serial.print(*(const int16_t*)addr);  break;
        case CTRL_U16: Serial.print(*(const uint16_t*)addr); break;
        case CTRL_I32: Serial.print(*(const int32_t*)addr);  break;
        case CTRL_U32: Serial.print(*(const uint32_t*)addr); break;
        default:       Serial.print(*(const float*)addr, 6); break;
    }
}

// ------------------------------------------------------------------- comandos

void CtrlLink::cmd_id(void)
{
    Serial.print(F("# id CtrlLink 1 "));
    Serial.print(m_id ? m_id : F("sketch"));
    Serial.print(F(" chans="));
    Serial.print(m_channel_count);
    Serial.print(F(" row="));
    Serial.print(row_width());
    Serial.print(F(" dt_us="));
    Serial.println(m_dt_us);
    Serial.println(F("# ok"));
}

void CtrlLink::cmd_params(void)
{
    for (int16_t i = 0; i < (int16_t)(K_BUILTIN_COUNT + m_param_count); i++)
    {
        const CtrlParam* entry = param_at(m_params, k_builtin, K_BUILTIN_COUNT, i);

        Serial.print(F("# p "));
        print_name(entry->name);
        Serial.write(' ');
        print_type(pgm_read_byte(&entry->type));
        Serial.write(' ');
        Serial.print((int8_t)pgm_read_byte(&entry->frac));
        Serial.write(' ');
        print_param_value(i);
        Serial.println();
    }

    Serial.println(F("# ok"));
}

void CtrlLink::cmd_chans(void)
{
    for (uint8_t i = 0; i < m_channel_count; i++)
    {
        Serial.print(F("# c "));
        Serial.print(i);
        Serial.write(' ');
        print_name(m_channels[i].name);
        Serial.write(' ');
        print_type(pgm_read_byte(&m_channels[i].type));
        Serial.write(' ');
        Serial.print(pgm_read_float(&m_channels[i].scale), 7);
        Serial.write(' ');
        print_name(m_channels[i].unit);
        Serial.println();
    }

    Serial.println(F("# ok"));
}

void CtrlLink::cmd_get(const char* name)
{
    int16_t index = find_param(name);

    if (index < 0)
    {
        error(F("no existe ese parametro"));
        return;
    }

    Serial.print(F("# v "));
    Serial.print(name);
    Serial.write(' ');
    print_param_value(index);
    Serial.println();
    Serial.println(F("# ok"));
}

void CtrlLink::cmd_set(const char* name, const char* value)
{
    int16_t index = find_param(name);

    if (index < 0)
    {
        error(F("no existe ese parametro"));
        return;
    }

    const CtrlParam* entry = param_at(m_params, k_builtin, K_BUILTIN_COUNT, index);

    write_value((void*)pgm_read_word(&entry->addr),
                pgm_read_byte(&entry->type),
                value);

    m_writes++;

    // Un set que cae en medio de una captura es una entrada escalón, así que el
    // tick en el que entró en vigencia es parte de la medición. Se informa en el
    // flujo en lugar de dejar que la computadora lo deduzca del orden de las
    // líneas.
    if (m_streaming)
    {
        Serial.print(F("# mark "));
        Serial.print(m_tick);
        Serial.write(' ');
        Serial.print(name);
        Serial.write(' ');
        print_param_value(index);
        Serial.println();
    }

    Serial.print(F("# v "));
    Serial.print(name);
    Serial.write(' ');
    print_param_value(index);
    Serial.println();
    Serial.println(F("# ok"));
}

void CtrlLink::cmd_start(void)
{
    if (!m_usable)
    {
        error(F("fila demasiado larga"));
        return;
    }
    if (m_decimate == 0)
    {
        m_decimate = 1;
    }

    m_rows      = 0;
    m_drops     = 0;
    m_dec_count = 0;

    Serial.println(F("# begin"));

    Serial.print(F("# rate dt_us="));
    Serial.print(m_dt_us);
    Serial.print(F(" dec="));
    Serial.println(m_decimate);

    // El tick es siempre la columna cero y siempre u16. Da la vuelta cada 65536
    // períodos de control y la computadora lo desenrolla.
    Serial.println(F("# col tick u16 1 tick"));

    for (uint8_t i = 0; i < m_channel_count; i++)
    {
        Serial.print(F("# col "));
        print_name(m_channels[i].name);
        Serial.write(' ');
        print_type(pgm_read_byte(&m_channels[i].type));
        Serial.write(' ');
        Serial.print(pgm_read_float(&m_channels[i].scale), 7);
        Serial.write(' ');
        print_name(m_channels[i].unit);
        Serial.println();
    }

    Serial.println(F("# data"));

    // Se activa al final: ninguna fila puede salir antes de que el encabezado
    // esté completo.
    m_streaming = true;
}

void CtrlLink::cmd_stop(void)
{
    m_streaming = false;

    Serial.print(F("# end rows="));
    Serial.print(m_rows);
    Serial.print(F(" drops="));
    Serial.println(m_drops);
    Serial.println(F("# ok"));
}

// Parte `line` en la primera tanda de espacios y devuelve el resto, que queda
// vacío si no había ninguno. La línea se modifica en el lugar.
static char* split(char* line)
{
    while (*line && *line != ' ')
    {
        line++;
    }
    if (*line == '\0')
    {
        return line;
    }

    *line++ = '\0';
    while (*line == ' ')
    {
        line++;
    }

    return line;
}

void CtrlLink::handle_command(char* line)
{
#ifdef CTRL_PROFILE
    g_ctrl_profile.cmds++;
#endif

    if (m_cmd_overflow)
    {
        m_cmd_overflow = false;
        error(F("comando demasiado largo"));
        return;
    }

    while (*line == ' ')
    {
        line++;
    }
    if (*line == '\0')
    {
        return;  // linea vacia: un empujon para resincronizar, no un error
    }

    char* rest = split(line);

    if      (strcmp(line, "id")     == 0) { cmd_id(); }
    else if (strcmp(line, "params") == 0) { cmd_params(); }
    else if (strcmp(line, "chans")  == 0) { cmd_chans(); }
    else if (strcmp(line, "start")  == 0) { cmd_start(); }
    else if (strcmp(line, "stop")   == 0) { cmd_stop(); }
    else if (strcmp(line, "get")    == 0)
    {
        if (*rest == '\0') { error(F("get necesita un nombre")); }
        else               { cmd_get(rest); }
    }
    else if (strcmp(line, "set") == 0)
    {
        char* value = split(rest);

        if (*rest == '\0' || *value == '\0') { error(F("set necesita un nombre y un valor")); }
        else                                 { cmd_set(rest, value); }
    }
    else
    {
        error(F("comando desconocido"));
    }
}

// ----------------------------------------------------------- interfaz pública

bool CtrlLink::begin(uint32_t baud,
                     const CtrlParam* params, uint8_t param_count,
                     const CtrlChannel* channels, uint8_t channel_count,
                     uint32_t dt_us)
{
    m_params        = params;
    m_param_count   = param_count;
    m_channels      = channels;
    m_channel_count = channel_count;
    m_dt_us         = dt_us;

    Serial.begin(baud);
    while (!Serial)
    {
        ;  // inofensivo en el UNO, necesario en placas con USB nativo
    }

    m_usable = (row_width() <= CTRL_MAX_ROW);

    if (!m_usable)
    {
        error(F("la tabla de canales genera una fila mas larga que el buffer de transmision"));
    }

    return m_usable;
}

void CtrlLink::poll(void)
{
    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            m_cmd[m_cmd_len] = '\0';
            m_cmd_len        = 0;
            handle_command(m_cmd);

            // Un comando por llamada. Una ráfaga de comandos no puede
            // convertirse en una cantidad de trabajo sin cota dentro de un mismo
            // período de control.
            return;
        }

        if (m_cmd_len < (CTRL_CMD_LEN - 1))
        {
            m_cmd[m_cmd_len++] = c;
        }
        else
        {
            m_cmd_overflow = true;
        }
    }
}

bool CtrlLink::emit(void)
{
    uint16_t tick = m_tick++;

#ifdef CTRL_PROFILE
    g_ctrl_profile.row = false;
#endif

    if (!m_streaming)
    {
        return true;
    }

    if (m_dec_count != 0)
    {
        m_dec_count--;
        return true;
    }

    // Un cero haría desbordar la cuenta regresiva hacia abajo y frenaría el flujo
    // durante 65535 ticks, y la computadora puede escribir este parámetro
    // mientras hay flujo.
    m_dec_count = (m_decimate > 1) ? (m_decimate - 1) : 0;

#ifdef CTRL_PROFILE
    const uint32_t t_fmt = micros();
#endif

    char  buf[CTRL_MAX_ROW];
    char* p = put_hex(buf, tick, 4);

    for (uint8_t i = 0; i < m_channel_count; i++)
    {
        uint8_t type = pgm_read_byte(&m_channels[i].type);
        p = put_hex(p,
                    read_value((const void*)pgm_read_word(&m_channels[i].addr), type),
                    hex_width(type));
    }

    *p++ = '\n';

    uint8_t length = (uint8_t)(p - buf);

#ifdef CTRL_PROFILE
    const uint32_t t_wr = micros();
    g_ctrl_profile.fmt_us = (uint16_t)(t_wr - t_fmt);
    g_ctrl_profile.wr_us  = 0;
    g_ctrl_profile.row    = true;
#endif

    // Nunca bloquear el lazo de control esperando a la UART. Una fila descartada
    // deja un hueco en la secuencia de ticks, que la computadora puede ver y
    // tener en cuenta; una escritura bloqueante distorsionaría en silencio la
    // temporización del lazo.
#ifdef CTRL_PROFILE
    // Escritura por encuesta: esperar UDRE y escribir UDR, byte por byte, con las
    // interrupciones abiertas. Primero se deja salir lo que HardwareSerial tenga
    // encolado --una respuesta a un comando-- para no mezclar las dos colas.
    if (g_ctrl_profile.poll_tx)
    {
        if (Serial.availableForWrite() < (int)(SERIAL_TX_BUFFER_SIZE - 1))
        {
            Serial.flush();
        }
        for (uint8_t i = 0; i < length; i++)
        {
            while (!(UCSR0A & _BV(UDRE0)))
            {
            }
            UDR0 = (uint8_t)buf[i];
        }
        m_rows++;
        g_ctrl_profile.wr_us = (uint16_t)(micros() - t_wr);
        return true;
    }
#endif

    if (Serial.availableForWrite() < (int)length)
    {
        m_drops++;
        return false;
    }

    Serial.write((const uint8_t*)buf, length);
    m_rows++;

#ifdef CTRL_PROFILE
    g_ctrl_profile.wr_us = (uint16_t)(micros() - t_wr);
#endif

    return true;
}
