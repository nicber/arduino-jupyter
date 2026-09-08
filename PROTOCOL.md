# CtrlLink

A serial protocol for driving an Arduino control loop from a Jupyter notebook:
set gains, step the reference, and pull back the resulting timeseries.

```python
from ctrllink import CtrlLink

dev = CtrlLink('/dev/tty.usbmodem1101')
dev.kp, dev.ki, dev.mode = 2.5, 0.1, 1
df = dev.step('ref', 1024, pre=0.1, post=0.9)   # a DataFrame, t = 0 at the step
df.plot(x='t', y=['ref', 'y'])
```

The device describes its own parameters and telemetry channels, so the host has
no knowledge of any particular sketch. Adding a gain to the firmware makes it
appear in the notebook with no change on the Python side.

## Design

**One port, two directions, no framing layer.** Lines beginning with `#` are
text: command replies, events, diagnostics. Every other line is one telemetry
row. That single convention makes the stream both machine-parseable and readable
in the Arduino Serial Monitor, and it lets `Serial.print` debugging coexist with
a live capture instead of corrupting it.

**Fixed-width hex rather than decimal or binary.** The reason is the *fixed
width*, not the hex. A row is the same length every time, so the bandwidth
budget is exact rather than worst-case, `emit()` can ask for precisely the room
it needs, and a capture decodes in one vectorized call instead of tokenizing
line by line:

```python
raw = bytes.fromhex(b''.join(rows).decode())
arr = np.frombuffer(raw, dtype=structured_dtype)
```

Hex is also somewhat cheaper to generate than decimal — a table lookup and a
`swap` per byte, against `itoa`'s repeated division — but avr-libc's `itoa` is
hand-written assembly, so the gap is maybe 20% of the encode cost, not an order
of magnitude. Encoding a 4 × int16 row costs on the order of 350 cycles, which
the Arduino serial write then roughly doubles; call it 70 µs, or 7% of a 1 ms
control period. Both are secondary to the framing benefits above.

Binary framing (COBS + CRC) would save a further ~40% of the bytes, but at
1 Mbaud the bandwidth is not the binding constraint, and it would cost the
ability to read the stream with your eyes. Raising the baud rate is the cheaper
lever: 115200 → 1 Mbaud is 8.7×, where hex → binary is 1.4×.

**1 Mbaud, not 115200.** On a 16 MHz AVR, 1000000 is an exact divisor (UBRR=1);
115200 lands on UBRR=16 for an actual 117647 baud, 2.1% off. The faster rate is
also the more accurate one. 250000 and 500000 are exact too, if a board's
USB-serial bridge will not go higher.

**The two directions are not equally robust.** Telemetry at 1 Mbaud is solid:
measured over five seconds, zero dropped rows and zero gaps. Commands are not.
An incoming byte lands every 10 µs, the AVR's USART holds two, and the 5 kHz
sampler and nI2C's TWI handler between them keep interrupts disabled for longer
than that — so roughly 4% of the bytes of a command sent back to back are lost.
Marking the sampler `ISR_NOBLOCK` does not fix it, because the TWI handler is
also a blocker and making an I2C state machine reentrant is not worth the risk.

The host therefore spaces command bytes half a millisecond apart, which removes
the loss entirely. Commands are rare and short — a `set` takes about 6 ms to
send — so it costs nothing, and the tick reported by `# mark` means even a step
sent mid-capture is still placed to the exact sample. On top of that the host
retries a command the device says it did not understand, and checks the value
echoed by a `set` rather than trusting it: a mangled *command* is rejected
loudly, but a mangled *value* would otherwise be accepted in silence.

**Streaming, not buffered capture.** An ATmega328P has 2 KB of SRAM, so a
buffered capture holds roughly 175 samples — 175 ms at 1 kHz, well short of a
settling transient. Streaming has no length limit; the link rate caps the sample
rate instead, and at 1 Mbaud that cap is far above any loop the UNO can run.

**Never block the control loop.** `Serial.write` blocks once the 64-byte
transmit buffer fills, which would stall the loop and distort the dynamics being
measured. `emit()` checks `availableForWrite()` first and drops the row if there
is no space, counting the drop. A dropped row leaves a visible gap in the tick
sequence; a blocking write would leave an invisible timing error.

## Bandwidth

Serial is 10 bits per byte, so the link carries `baud/10` bytes per second. A row
costs `4 + Σ(channel widths) + 1` bytes, where a channel is 4 hex characters for
an int16 and 8 for an int32 or float.

| | row | @115200 | @250k | @500k | @1M |
|---|---|---|---|---|---|
| 4 × int16 | 21 B | 548 Hz | 1.2 kHz | 2.4 kHz | 4.8 kHz |
| 4 × float | 37 B | 311 Hz | 676 Hz | 1.4 kHz | 2.7 kHz |

Those are 100% utilization. Stay under half: the `ControlDemo` sketch runs 4
int16 channels at 1 kHz, which is 21 kB/s, or 21% of a 1 Mbaud link.

## Wire protocol

Commands are ASCII, one per line, `\n` terminated. Every reply ends with `# ok`,
`# err <reason>` or `# data`, so the host can wait for a definite terminator
rather than guessing with a sleep.

| Command | Reply |
|---|---|
| `id` | `# id CtrlLink 1 <sketch> chans=<n> row=<bytes> dt_us=<n>` |
| `params` | one `# p <name> <type> <frac> <value>` per parameter |
| `chans` | one `# c <i> <name> <type> <scale> <unit>` per channel |
| `get <name>` | `# v <name> <value>` |
| `set <name> <value>` | `# v <name> <value>` |
| `start` | the stream header, ending `# data`, then rows |
| `stop` | `# end rows=<n> drops=<n>` |

Types are `i8 u8 i16 u16 i32 u32 f32`. A bare newline is ignored, so sending one
is a safe way to resynchronize.

`<value>` on the wire is always the device's raw storage. `<frac>` says how many
fractional bits that storage carries, so the host reads `raw / 2**frac` and
writes `round(value * 2**frac)`. A device can therefore keep a gain in Q22 and a
filter pole in Q16 — whatever its arithmetic wants — while the host goes on
setting them as `0.5` and `0.02`, and the conversion happens on the side with a
floating-point unit and no deadline. `frac = 0` is a plain integer.

Powers of two rather than a channel's arbitrary float `scale`, because that is
what a fixed-point format is, and because it crosses the wire exactly: no fixed
number of decimal places prints both a Q22 scale (2.4e-7) and a Q30 one
(9.3e-10) usefully. Anything needing a scale that is not a power of two —
degrees per count, milliamps per LSB — is a channel, which has one.

### Stream

```
# begin
# rate dt_us=1000 dec=1
# col tick u16 1 tick
# col ref i16 0.0878906 deg
# col y i16 0.0878906 deg
# col u i16 1.0000000 pwm
# data
0412CDB90C800076
0413CDB90C830074
```

Column zero is always a 16-bit tick counter, incremented once per control period
whether or not a row is emitted. It wraps every 65536 periods and the host
unwraps it. Multiply by `dt_us` for seconds; multiply a channel by its `scale`
for engineering units. Every value is unsigned hex of the raw storage, most
significant nibble first — a float is its four IEEE-754 bytes, not a decimal
conversion, which makes it free to emit on a device with no FPU.

The device only ever emits values; unit conversion happens on the host.

### Events during a capture

`set` works while streaming, and the device reports the tick the value took
effect on:

```
0412CDB90C800076
# mark 1043 ref 1024
0413CDB90C830074
```

This is what makes a step response exact. The host schedules the step with
millisecond-ish accuracy, but the *measurement* uses the tick the device
reports, so host scheduling jitter never enters the data.

## Host API

`CtrlLink(port, baud=1_000_000)` opens the port, waits out the DTR auto-reset,
and discovers the device.

- Parameters are attributes, always in real units: `dev.kp = 2.5`,
  `print(dev.kp)`. `dev.params` reads
  them all back.
- `dev.capture(duration, events=[(delay, name, value), ...])` → DataFrame.
- `dev.step(name, value, pre=0.1, post=0.9, back=None)` → DataFrame with `t = 0`
  at the step.

The DataFrame carries `t` in seconds plus one column per channel in engineering
units. `df.attrs` holds `dt_us`, `dec`, `units`, `marks`, `notes`, the device's
own `rows`/`drops` counts, and `gaps` — the number of breaks in the tick
sequence, which is 0 for a clean capture.

## Firmware

Declare the tunables and the telemetry as PROGMEM tables and the rest follows:

```cpp
static const CtrlParam PROGMEM g_params[] = {
    { "kp",  CTRL_I32, &g_kp,  22 },   // Q22: the host sets 0.5, the device stores 2097152
    { "ref", CTRL_I16, &g_ref,  0 },   // a plain integer
};

static const CtrlChannel PROGMEM g_channels[] = {
    { "y", CTRL_I16, &g_y, 360.0f / 4096, "deg" },
    { "u", CTRL_I16, &g_u, 1.0f,          "pwm" },
};

void setup() {
    CtrlLink::set_id(F("MySketch"));
    CtrlLink::begin(1000000, g_params, 2, g_channels, 2, /* dt_us */ 1000);
}

void loop() {
    if (tick_due()) {
        control_step();      // writes g_y, g_u
        CtrlLink::emit();    // one row, never blocks
    }
    CtrlLink::poll();        // at most one command per call
}
```

`emit()` reads the channels through their addresses, so it must be called from
the same context that writes them — `loop()`, not an ISR. Names are at most 8
characters. A channel table producing a row longer than the transmit buffer is
rejected by `begin()` rather than silently dropping every sample.

## Measured

On an UNO clone with a CH340G bridge, 1 Mbaud, four int16 channels at 1 kHz:

| | |
|---|---|
| sample rate | 1000.2 Hz over 5 s |
| rows | 4983 sent, 4983 received |
| dropped rows | 0 |
| tick gaps | 0 |
| link used | 21 kB/s, 21% of 1 Mbaud |
| worst control-loop service delay | 344 µs on a 1000 µs period |

The CH340 is worth calling out: it is the bridge on most UNO clones and the one
usually assumed to be limited to low rates, and it ran 1 Mbaud without a single
lost row. The service delay is the honest cost of running the control law in
`loop()` alongside serial handling — sampling itself is rigid, driven by Timer2,
so a late computation shows up as jitter in `u`, not in `y`. `maxlate` reports
it, and is writable, so zero it before a run to measure that run.

## Setup

```
python3 -m venv .venv
./.venv/bin/pip install -r python/requirements.txt jupyterlab matplotlib ipykernel
./.venv/bin/python -m ipykernel install --user --name arduino-control \
    --display-name "Arduino Control (.venv)"
./.venv/bin/jupyter lab notebooks/control_demo.ipynb
```

## Layout

- `libraries/CtrlLink/` — the protocol, board side
- `python/ctrllink.py` — the protocol, host side
- `python/test_ctrllink.py` — host tests against a byte-faithful device simulation
- `ControlDemo/` — AS5600 position loop at 1 kHz, sampled at 5 kHz
- `notebooks/control_demo.ipynb` — worked demo: link health, open- and closed-loop
  steps, a gain sweep, changing the loop rate

```
arduino-cli compile --fqbn arduino:avr:uno --libraries ./libraries ControlDemo
arduino-cli upload  --fqbn arduino:avr:uno --libraries ./libraries -p <port> ControlDemo
```

`ControlDemo` takes Timer2 for the 5 kHz sampler, so `analogWrite` on pins 3 and
11 stops working; pins 9 and 10 (Timer1) and 5 and 6 (Timer0) are unaffected.
