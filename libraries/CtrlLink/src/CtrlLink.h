// Telemetry and parameter link between an Arduino control loop and a host.
//
// One serial port carries both directions at once:
//
//   host -> device   ASCII commands, one per line, each answered by a reply
//                    that ends in "# ok", "# err <reason>" or "# data".
//   device -> host   lines starting with '#' are text (replies, events,
//                    diagnostics); every other line is one telemetry row of
//                    fixed-width uppercase hex, most significant nibble first.
//
// A row is the 16-bit tick counter followed by the declared channels, with no
// separators:
//
//   0412 CDB9 0C80 0076
//   tick ref   y    u          ->  "0412CDB90C800076\n"
//
// Widths come from the channel types, which the host reads from the header, so
// the whole capture parses as one numpy structured dtype. Nothing in the row is
// variable-width and nothing needs to be tokenised.
//
// Bandwidth: serial is 10 bits per byte, so the link moves baud/10 bytes per
// second. A row costs 4 + sum(channel widths) + 1 bytes. At 1 Mbaud a 4-channel
// int16 row is 21 bytes, so a 1 kHz loop uses 21% of the link. Keep utilisation
// well under half: emit() never blocks, it drops a row instead, and a dropped
// row is a hole in the timeseries.
//
// Parameters and channels are declared as PROGMEM tables in the sketch. The
// host discovers them at runtime, so adding a gain costs nothing on the Python
// side.

#ifndef CTRLLINK_H
#define CTRLLINK_H

#include <Arduino.h>
#include <stdint.h>
#include <avr/pgmspace.h>

// Storage types, shared by parameters and channels. The hex width of a channel
// follows from its type: 2 nibbles per byte.
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

// Names are fixed-width and NUL-padded so the tables are plain PROGMEM arrays
// with no separate string symbols to declare.
static const uint8_t CTRL_NAME_LEN = 8;

// Longest command line accepted from the host, including arguments.
static const uint8_t CTRL_CMD_LEN = 40;

// Ceiling on 4 + sum(channel widths) + 1. A row can never be longer than the
// transmit buffer, because availableForWrite() never reports more than
// SERIAL_TX_BUFFER_SIZE - 1 free and emit() refuses to write without room: an
// over-long row would drop every sample and never send one. begin() rejects
// such a channel table instead of failing silently.
//
// 63 bytes on an UNO, so up to 14 int16 or 7 float channels. Staying well under
// the limit matters as well as fitting it -- a row close to the buffer size only
// goes out when the buffer happens to be nearly empty.
static const uint8_t CTRL_MAX_ROW = SERIAL_TX_BUFFER_SIZE - 1;

// A host-writable variable. `addr` points at RAM the sketch owns; the link
// converts between the wire text and `type` on the way in and out.
struct CtrlParam
{
    char    name[CTRL_NAME_LEN];
    uint8_t type;
    void*   addr;
};

// A telemetry column. `scale` and `unit` are passed through to the host, which
// multiplies the raw integer by `scale` to get engineering units -- the device
// never spends cycles on the conversion.
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

    // `dt_us` is the nominal control period, reported to the host so it can
    // turn tick numbers into seconds. The tables stay owned by the caller and
    // must live in PROGMEM.
    //
    // Returns false if the channel table would produce a row longer than
    // CTRL_MAX_ROW; the link still runs, but streaming stays disabled.
    static bool begin(uint32_t baud,
                      const CtrlParam* params, uint8_t param_count,
                      const CtrlChannel* channels, uint8_t channel_count,
                      uint32_t dt_us);

    // Reads whatever the host has sent and executes any complete command.
    // Non-blocking. Call once per control period, before or after emit().
    static void poll(void);

    // Advances the tick and, if streaming and the row is not decimated away,
    // writes one row. Call exactly once per control period, from the same
    // context that computed the channel variables -- the channels are read
    // through their pointers here, so this must not race the code writing them.
    //
    // Returns false if the row was dropped for lack of room in the transmit
    // buffer. Never blocks and never stalls the control loop.
    static bool emit(void);

    static bool     streaming(void) { return m_streaming; }
    static uint32_t rows(void)      { return m_rows; }
    static uint32_t drops(void)     { return m_drops; }

    // Nominal control period. The sketch may change it at runtime as long as it
    // tells the link, so the header the host reads stays truthful.
    static void     set_period_us(uint32_t dt_us) { m_dt_us = dt_us; }
    static uint32_t period_us(void)               { return m_dt_us; }

    // Extra text for the "id" reply, e.g. the sketch name. Must be a PROGMEM
    // string; kept as a pointer, not copied.
    static void set_id(const __FlashStringHelper* id) { m_id = id; }

    // Emits "# note <text>" as an out-of-band line. Safe to call while
    // streaming: the host records it against the current tick.
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

    // Returns the index of `name` in the parameter table, or -1. Built-in
    // parameters live above the user table and are searched first.
    static int16_t find_param(const char* name);
    static void    print_param_value(int16_t index);

    static void print_name(const char* pgm_name);
    static void print_type(uint8_t type);
    static void error(const __FlashStringHelper* reason);

    static uint8_t hex_width(uint8_t type);
    static uint8_t row_width(void);

    // Parameters the link owns itself. Searched before the sketch's table and
    // listed alongside it, so the host discovers them the same way.
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
    static bool     m_usable;      // row fits in CTRL_MAX_ROW
    static uint16_t m_tick;
    static uint16_t m_decimate;    // emit one row every m_decimate ticks
    static uint16_t m_dec_count;   // ticks still to skip before the next row
    static uint32_t m_dt_us;
    static uint32_t m_rows;
    static uint32_t m_drops;
};

#endif  // CTRLLINK_H
