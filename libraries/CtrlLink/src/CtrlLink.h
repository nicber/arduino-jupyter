// Enlace de telemetría y parámetros entre un lazo de control en Arduino y una
// computadora.
//
// Un solo puerto serie lleva las dos direcciones a la vez:
//
//   PC -> dispositivo   comandos ASCII, uno por línea, cada uno respondido por
//                       una respuesta que termina en "# ok", "# err <motivo>" o
//                       "# data".
//   dispositivo -> PC   las líneas que empiezan con '#' son texto (respuestas,
//                       eventos, diagnósticos); cualquier otra línea es una fila
//                       de telemetría en hexadecimal de ancho fijo y mayúsculas,
//                       con el nibble más significativo primero.
//
// Una fila es el contador de ticks de 16 bits seguido de los canales declarados,
// sin separadores:
//
//   0412 CDB9 0C80 0076
//   tick ref   y    u          ->  "0412CDB90C800076\n"
//
// Los anchos salen de los tipos de los canales, que la computadora lee del
// encabezado, así que toda la captura se interpreta como un único dtype
// estructurado de numpy. Nada en la fila es de ancho variable y nada hay que
// separarlo en tokens.
//
// Ancho de banda: el puerto serie usa 10 bits por byte, así que el enlace mueve
// baud/10 bytes por segundo. Una fila cuesta 4 + suma(anchos de canal) + 1
// bytes. A 1 Mbaud una fila de 4 canales int16 son 21 bytes, así que un lazo de
// 1 kHz usa el 21 % del enlace. Conviene mantener la utilización bastante por
// debajo de la mitad: emit() nunca bloquea, descarta una fila en su lugar, y una
// fila descartada es un agujero en la serie temporal.
//
// Los parámetros y los canales se declaran como tablas en PROGMEM dentro del
// sketch. La computadora los descubre en tiempo de ejecución, así que agregar
// una ganancia no cuesta nada del lado de Python.

#ifndef CTRLLINK_H
#define CTRLLINK_H

#include <Arduino.h>
#include <stdint.h>
#include <avr/pgmspace.h>

// Tipos de almacenamiento, compartidos por parámetros y canales. El ancho
// hexadecimal de un canal se desprende de su tipo: 2 nibbles por byte.
enum : uint8_t
{
    CTRL_I8  = 0,
    CTRL_U8  = 1,
    CTRL_I16 = 2,
    CTRL_U16 = 3,
    CTRL_I32 = 4,
    CTRL_U32 = 5,
    CTRL_F32 = 6,
};

// Los nombres son de ancho fijo y se rellenan con NUL, así que las tablas son
// arreglos comunes en PROGMEM sin símbolos de cadena aparte que haya que
// declarar.
//
// Doce y no ocho para que quepa un prefijo de módulo. Una tabla de tres docenas de
// parámetros planos no dice quién es dueño de cuál, y `ctl_uff` contra `ang_cal`
// contra `loop_div` lo dice sin que haya que ir a leer el sketch. Los cuatro bytes de
// más por entrada viven en flash y no en RAM.
static const uint8_t CTRL_NAME_LEN = 12;

// Línea de comando más larga que se acepta de la computadora, argumentos
// incluidos.
static const uint8_t CTRL_CMD_LEN = 40;

// Techo de 4 + suma(anchos de canal) + 1. Una fila nunca puede ser más larga que
// el buffer de transmisión, porque availableForWrite() nunca informa más de
// SERIAL_TX_BUFFER_SIZE - 1 libres y emit() se niega a escribir sin lugar: una
// fila demasiado larga descartaría todas las muestras y no enviaría ninguna.
// begin() rechaza una tabla de canales así en lugar de fallar en silencio.
//
// Son 63 bytes en un UNO, o sea hasta 14 canales int16 o 7 float. Además de
// entrar, importa quedarse bastante por debajo del límite: una fila cercana al
// tamaño del buffer sólo sale cuando el buffer está casi vacío.
static const uint8_t CTRL_MAX_ROW = SERIAL_TX_BUFFER_SIZE - 1;

// Una variable que la computadora puede escribir. `addr` apunta a RAM que
// pertenece al sketch; el enlace convierte entre el texto del cable y `type` a
// la ida y a la vuelta.
//
// `frac` es cuántos bits fraccionarios lleva el entero almacenado: la
// computadora lee `raw / 2^frac` y escribe `round(valor * 2^frac)`. Es lo que le
// permite a un dispositivo guardar un parámetro en la forma de punto fijo que su
// aritmética prefiera —una ganancia en Q22, el polo de un filtro en Q16— mientras
// la computadora lo sigue fijando como 0.5 o 0.02. La conversión ocurre en la
// computadora, que tiene unidad de punto flotante y no tiene plazo que cumplir;
// el dispositivo sólo ve el entero que quería.
//
// Una potencia de dos en lugar de la escala flotante arbitraria de un canal,
// porque eso es lo que realmente es un formato de punto fijo, y porque sobrevive
// al cable de forma exacta: una escala Q22 es 2.38e-7, y ningún número fijo de
// decimales la imprime de manera útil junto con los 9.3e-10 de una Q30. Usar 0
// para un parámetro que ya está en unidades naturales, incluido cualquier f32.
// Los valores negativos escalan hacia arriba.
struct CtrlParam
{
    char    name[CTRL_NAME_LEN];
    uint8_t type;
    void*   addr;
    int8_t  frac;
};

// Una columna de telemetría. `scale` y `unit` se pasan tal cual a la
// computadora, que multiplica el entero crudo por `scale` para obtener unidades
// de ingeniería; el dispositivo nunca gasta ciclos en la conversión.
struct CtrlChannel
{
    char        name[CTRL_NAME_LEN];
    uint8_t     type;
    const void* addr;
    float       scale;
    char        unit[CTRL_NAME_LEN];
};

class CtrlLink
{
    public:

    // `dt_us` es el período de control nominal, que se le informa a la
    // computadora para que pueda convertir números de tick en segundos. Las
    // tablas siguen perteneciendo a quien llama y tienen que vivir en PROGMEM.
    //
    // Devuelve false si la tabla de canales produjera una fila más larga que
    // CTRL_MAX_ROW; el enlace igual funciona, pero el flujo queda deshabilitado.
    static bool begin(uint32_t baud,
                      const CtrlParam* params, uint8_t param_count,
                      const CtrlChannel* channels, uint8_t channel_count,
                      uint32_t dt_us);

    // Lee lo que haya mandado la computadora y ejecuta cualquier comando
    // completo. No bloquea. Llamar una vez por período de control, antes o
    // después de emit().
    static void poll(void);

    // Avanza el tick y, si hay flujo y la diezmación no descarta la fila,
    // escribe una fila. Llamar exactamente una vez por período de control, desde
    // el mismo contexto que calculó las variables de los canales: acá los
    // canales se leen a través de sus punteros, así que esto no puede competir
    // con el código que los escribe.
    //
    // Devuelve false si la fila se descartó por falta de lugar en el buffer de
    // transmisión. Nunca bloquea y nunca frena el lazo de control.
    static bool emit(void);

    static bool     streaming(void) { return m_streaming; }
    static uint32_t rows(void)      { return m_rows; }
    static uint32_t drops(void)     { return m_drops; }

    // Cuenta las escrituras de parámetros aceptadas de la computadora. Un sketch
    // con constantes derivadas de sus parámetros —una ganancia convertida a
    // punto fijo, un coeficiente de filtro calculado a partir de una constante
    // de tiempo— puede vigilar este único valor en lugar de comparar cada
    // parámetro del que depende, y hacer la derivación sólo cuando algo se movió
    // de verdad. Da la vuelta; comparar por diferencia, no por orden.
    static uint16_t writes(void)    { return m_writes; }

    // Período de control nominal. El sketch puede cambiarlo en tiempo de
    // ejecución siempre que se lo avise al enlace, para que el encabezado que lee
    // la computadora siga diciendo la verdad.
    static void     set_period_us(uint32_t dt_us) { m_dt_us = dt_us; }
    static uint32_t period_us(void)               { return m_dt_us; }

    // Texto adicional para la respuesta de "id", por ejemplo el nombre del
    // sketch. Tiene que ser una cadena en PROGMEM; se guarda como puntero, no se
    // copia.
    static void set_id(const __FlashStringHelper* id) { m_id = id; }

    // Emite "# note <texto>" como línea fuera de banda. Es seguro llamarlo
    // mientras hay flujo: la computadora lo registra contra el tick actual.
    static void note(const __FlashStringHelper* text);

    private:

    static void handle_command(char* line);
    static void cmd_id(void);
    static void cmd_params(void);
    static void cmd_chans(void);
    static void cmd_get(const char* name);
    static void cmd_set(const char* name, const char* value);
    static void cmd_start(void);
    static void cmd_stop(void);

    // Devuelve el índice de `name` en la tabla de parámetros, o -1. Los
    // parámetros incorporados viven por encima de la tabla del usuario y se
    // buscan primero.
    static int16_t find_param(const char* name);
    static void    print_param_value(int16_t index);

    static void print_name(const char* pgm_name);
    static void print_type(uint8_t type);
    static void error(const __FlashStringHelper* reason);

    static uint8_t hex_width(uint8_t type);
    static uint8_t row_width(void);

    // Parámetros que el propio enlace administra. Se buscan antes que la tabla
    // del sketch y se listan junto con ella, así que la computadora los descubre
    // de la misma manera.
    static const CtrlParam k_builtin[];
    static const uint8_t   K_BUILTIN_COUNT = 1;

    static const CtrlParam*   m_params;
    static const CtrlChannel* m_channels;
    static uint8_t            m_param_count;
    static uint8_t            m_channel_count;

    static const __FlashStringHelper* m_id;

    static char    m_cmd[CTRL_CMD_LEN];
    static uint8_t m_cmd_len;
    static bool    m_cmd_overflow;

    static bool     m_streaming;
    static bool     m_usable;      // la fila entra en CTRL_MAX_ROW
    static uint16_t m_tick;
    static uint16_t m_decimate;    // emitir una fila cada m_decimate ticks
    static uint16_t m_dec_count;   // ticks que faltan saltear antes de la próxima fila
    static uint32_t m_dt_us;
    static uint32_t m_rows;
    static uint32_t m_drops;
    static uint16_t m_writes;
};

#endif  // CTRLLINK_H
