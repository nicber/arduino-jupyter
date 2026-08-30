"""Host side of the CtrlLink protocol.

Talks to an Arduino running the CtrlLink library over one serial port. The
device describes its own tunable parameters and telemetry channels, so this
module has no knowledge of any particular sketch: adding a gain to the firmware
makes it appear here with no change on this side.

    from ctrllink import CtrlLink

    dev = CtrlLink('/dev/tty.usbmodem1101')
    dev.kp, dev.ki = 2.5, 0.1
    df = dev.step('ref', 1024, pre=0.1, post=0.9)
    df.plot(x='t', y=['ref', 'y'])

Telemetry rows are fixed-width hex, so a capture is decoded in one numpy call
rather than parsed line by line; a 1 kHz stream costs almost nothing to receive.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import serial

# Wire type -> big-endian numpy dtype. The hex width of a field is twice the
# itemsize, and every width is even, so a run of rows unhexlifies as one block.
_TYPES = {
    'i8': np.dtype('>i1'),
    'u8': np.dtype('>u1'),
    'i16': np.dtype('>i2'),
    'u16': np.dtype('>u2'),
    'i32': np.dtype('>i4'),
    'u32': np.dtype('>u4'),
    'f32': np.dtype('>f4'),
}

# Replies that end a command's response.
_TERMINATORS = ('# ok', '# err', '# data')

# Errors that mean the device did not receive what was sent, rather than that it
# received it and objected. Only these are worth retrying.
_GARBLED = ('unknown command', 'command too long', 'needs')

# Seconds between the bytes of an outgoing command.
#
# Telemetry runs at 1 Mbaud without trouble, but the return path is fragile: a
# byte lands every 10 us, the AVR's USART holds two, and the 5 kHz sampler and
# nI2C's TWI interrupt between them keep interrupts disabled for longer than
# that. Sent back to back, a few percent of command bytes are simply lost.
# Spacing them out fixes it completely, and commands are far too rare and short
# for the delay to matter -- a `set` takes about 6 ms to send.
_BYTE_GAP = 0.0005


class CtrlLinkError(RuntimeError):
    pass


def find_port(hint=None):
    """Guesses which serial port the board is on.

    Bluetooth adapters and debug consoles also present as serial ports, so the
    search is limited to USB ones. `hint` narrows it further by substring, which
    is what to reach for when more than one board is plugged in.
    """
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports()
             if any(tag in p.device for tag in
                    ('usbserial', 'usbmodem', 'ttyUSB', 'ttyACM', 'wchusbserial'))]

    if hint:
        ports = [p for p in ports if hint in p]

    if not ports:
        raise CtrlLinkError('no USB serial port found -- is the board plugged in?')
    if len(ports) > 1:
        raise CtrlLinkError(f'several USB serial ports found ({", ".join(ports)}); '
                            f'pass one explicitly or narrow it with a hint')
    return ports[0]


def _ended(buf):
    """True once a complete "# end ..." line is in the buffer."""
    at = buf.find(b'# end ')
    return at >= 0 and buf.find(b'\n', at) >= 0


@dataclass
class Column:
    name: str
    type: str
    scale: float
    unit: str

    @property
    def dtype(self) -> np.dtype:
        return _TYPES[self.type]

    @property
    def width(self) -> int:
        """Width of this field in hex characters."""
        return self.dtype.itemsize * 2


class CtrlLink:
    def __init__(self, port=None, baud=1_000_000, reset_wait=1.8, timeout=1.0):
        if port is None:
            port = find_port()

        # Opening the port asserts DTR, which resets an UNO. Nothing the device
        # says before it has rebooted and run setup() is worth reading.
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(reset_wait)
        self.ser.reset_input_buffer()

        self.info = self.sync()
        self._params = self._read_params()
        self.channels = self._read_channels()

    # ------------------------------------------------------------- plumbing

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _readline(self, deadline) -> str | None:
        """One line, or None once `deadline` has passed."""
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                return raw.decode('ascii', 'replace').rstrip('\r\n')
        return None

    def _send(self, line):
        """Writes a command with its bytes spaced out. See _BYTE_GAP."""
        for byte in (line + '\n').encode('ascii'):
            self.ser.write(bytes([byte]))
            if _BYTE_GAP:
                time.sleep(_BYTE_GAP)
        self.ser.flush()

    def cmd(self, line, timeout=2.0, tries=3) -> list[str]:
        """Sends a command and returns its reply lines, terminator included.

        A reply saying the device did not understand the command means bytes
        were lost on the way in, so the command is resent. An error that means
        the device understood and objected is raised straight away.

        Telemetry rows arriving while the reply is in flight are discarded, so
        this is safe to call mid-capture -- but during a capture prefer the
        `events` argument to capture(), which keeps the rows.
        """
        for attempt in range(tries):
            self._send(line)

            deadline = time.monotonic() + timeout
            reply = []

            while True:
                text = self._readline(deadline)
                if text is None:
                    raise CtrlLinkError(f'timed out waiting for a reply to {line!r}')
                if not text.startswith('#'):
                    continue  # a telemetry row overtaking the reply
                reply.append(text)
                if not text.startswith(_TERMINATORS):
                    continue

                if not text.startswith('# err'):
                    return reply

                reason = text[6:]
                if attempt + 1 < tries and any(g in reason for g in _GARBLED):
                    break  # the command was mangled in transit; send it again
                raise CtrlLinkError(f'{line!r}: {reason}')

    def sync(self, timeout=4.0) -> str:
        """Drains whatever the device was saying and confirms it is listening."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.ser.reset_input_buffer()
            try:
                for text in self.cmd('id', timeout=0.5):
                    if text.startswith('# id '):
                        return text[5:]
            except CtrlLinkError:
                continue
        raise CtrlLinkError('no response to "id" -- wrong port, wrong baud, '
                            'or the sketch is not running CtrlLink')

    # ------------------------------------------------------------ discovery

    def _read_params(self) -> dict:
        params = {}
        for text in self.cmd('params'):
            if text.startswith('# p '):
                name, type_, value = text[4:].split(None, 2)
                params[name] = type_
        return params

    def _read_channels(self) -> list[Column]:
        chans = []
        for text in self.cmd('chans'):
            if text.startswith('# c '):
                _, name, type_, scale, *unit = text[4:].split()
                chans.append(Column(name, type_, float(scale),
                                    unit[0] if unit else ''))
        return chans

    @property
    def params(self) -> dict:
        """Every parameter and its current value, read back from the device."""
        return {name: self.get(name) for name in self._params}

    def get(self, name):
        for text in self.cmd(f'get {name}'):
            if text.startswith('# v '):
                _, value = text[4:].split(None, 1)
                return self._coerce(name, value)
        raise CtrlLinkError(f'no value in the reply for {name!r}')

    def set(self, name, value, tries=3):
        """Sets a parameter and confirms the device stored what was asked.

        A corrupted command is usually rejected outright, but a mangled *value*
        would be accepted in silence -- so the echoed value is checked rather
        than trusted.
        """
        for attempt in range(tries):
            for text in self.cmd(f'set {name} {value}'):
                if not text.startswith('# v '):
                    continue
                _, echoed = text[4:].split(None, 1)
                stored = self._coerce(name, echoed)
                if self._agrees(stored, value):
                    return stored
                break
            if attempt + 1 == tries:
                raise CtrlLinkError(
                    f'set {name} to {value!r} but the device reports {stored!r}')
        return None

    @staticmethod
    def _agrees(stored, requested):
        try:
            wanted = float(requested)
        except (TypeError, ValueError):
            return False
        # The device prints floats to six decimals, so an exact match is not
        # available; integers must agree exactly.
        return abs(float(stored) - wanted) <= max(1e-6, abs(wanted) * 1e-6)

    def _coerce(self, name, text):
        return float(text) if self._params[name] == 'f32' else int(text)

    # Parameters as attributes, so a notebook reads `dev.kp = 2.5`. Anything not
    # in the device's table falls through to normal attribute handling.
    def __getattr__(self, name):
        params = self.__dict__.get('_params', {})
        if name in params:
            return self.get(name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        params = self.__dict__.get('_params', {})
        if name in params:
            self.set(name, value)
        else:
            super().__setattr__(name, value)

    def __dir__(self):
        return list(super().__dir__()) + list(self.__dict__.get('_params', {}))

    # -------------------------------------------------------------- capture

    def capture(self, duration, events=(), poll=0.005):
        """Streams for `duration` seconds and returns a DataFrame.

        `events` is a sequence of (delay_s, name, value): each parameter is set
        that many seconds after the stream starts. The device reports the exact
        tick each set landed on, so the host's scheduling jitter does not enter
        the measurement -- see `df.attrs['marks']`.
        """
        pending = sorted(events, key=lambda e: e[0])

        self.ser.reset_input_buffer()
        header = self.cmd('start')
        columns, dt_us, dec = self._parse_header(header)

        buf = bytearray()
        started = time.monotonic()
        deadline = started + duration

        while True:
            now = time.monotonic()

            while pending and now - started >= pending[0][0]:
                _, name, value = pending.pop(0)
                self._send(f'set {name} {value}')

            if now >= deadline:
                break

            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                buf += chunk
            elif not pending:
                time.sleep(poll)

        # Keep reading until the device confirms it stopped, so the tail of the
        # stream and the final row counts are both in hand. The whole line has
        # to be there, not just its first few bytes: waiting only for the "# end"
        # prefix hands the decoder a line cut off mid-field.
        #
        # A "stop" can be lost on the way in like any other command, and there is
        # no reply to retry against until the stream actually ends, so it is
        # simply repeated until the device answers.
        for _ in range(4):
            self._send('stop')

            tail_deadline = time.monotonic() + 1.5
            while not _ended(buf) and time.monotonic() < tail_deadline:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
                if chunk:
                    buf += chunk
                else:
                    time.sleep(poll)

            if _ended(buf):
                break
        else:
            raise CtrlLinkError('device did not stop streaming')

        return self._decode(buf, columns, dt_us, dec)

    def step(self, name, value, pre=0.1, post=0.9, back=None):
        """Captures a step response, with `t = 0` at the step itself.

        Holds for `pre` seconds, sets `name` to `value`, holds for `post` more.
        If `back` is given the parameter is restored afterwards.
        """
        df = self.capture(pre + post, events=[(pre, name, value)])

        marks = [m for m in df.attrs['marks'] if m[1] == name]
        if marks:
            # Shifted in tick space, not seconds: subtracting two floats that
            # are each a tick times a period leaves a rounding residue, and a
            # t of -5e-17 puts the step sample on the wrong side of t < 0.
            origin = self._mark_tick(df, marks[0][0])
            df['t'] = (df.attrs['tick'] - origin) * (df.attrs['dt_us'] * 1e-6)

        if back is not None:
            self.set(name, back)

        return df

    @staticmethod
    def _mark_tick(df, raw_tick):
        """Unwrapped tick of a mark, which the device reports as raw 16 bits.

        The frame's own ticks have been unwrapped, so the two live in different
        spaces and cannot simply be subtracted.
        """
        tick = df.attrs['tick']

        if len(tick) == 0:
            return 0

        here = np.flatnonzero((tick & 0xFFFF) == raw_tick)
        if len(here):
            return int(tick[here[0]])

        # Decimation or a dropped row can leave the marked tick with no row of
        # its own; place it by how far it is past the first tick of the capture.
        return int(tick[0] + ((raw_tick - (tick[0] & 0xFFFF)) & 0xFFFF))

    # --------------------------------------------------------------- decode

    @staticmethod
    def _parse_header(lines):
        columns, dt_us, dec = [], None, 1

        for text in lines:
            if text.startswith('# col '):
                name, type_, scale, *unit = text[6:].split()
                columns.append(Column(name, type_, float(scale),
                                      unit[0] if unit else ''))
            elif text.startswith('# rate '):
                fields = dict(f.split('=') for f in text[7:].split())
                dt_us = int(fields['dt_us'])
                dec = int(fields['dec'])

        if not columns or dt_us is None:
            raise CtrlLinkError('device did not send a usable stream header')

        return columns, dt_us, dec

    def _decode(self, buf, columns, dt_us, dec):
        import pandas as pd

        dtype = np.dtype([(c.name, c.dtype) for c in columns])
        width = sum(c.width for c in columns)

        # println() emits CRLF and rows emit bare LF; hex never contains CR, so
        # dropping every CR up front makes both kinds of line uniform.
        lines = buf.replace(b'\r', b'').split(b'\n')

        marks, notes = [], []
        stats = {'rows': None, 'drops': None}
        rows = []

        for line in lines:
            if line.startswith(b'#'):
                text = line.decode('ascii', 'replace')
                if text.startswith('# mark '):
                    tick, name, value = text[7:].split(None, 2)
                    marks.append((int(tick), name, value))
                elif text.startswith('# note '):
                    notes.append(text[7:])
                elif text.startswith('# end '):
                    stats.update((k, int(v)) for k, v in
                                 (f.split('=', 1) for f in text[6:].split()
                                  if '=' in f))
            elif len(line) == width:
                rows.append(line)

        raw = bytes.fromhex(b''.join(rows).decode('ascii')) if rows else b''

        # The wire is big-endian, so frombuffer hands back big-endian fields.
        # pandas refuses to index those on a little-endian machine, and some of
        # its paths quietly return the wrong values instead of raising, so the
        # array is brought into native order before anything else touches it.
        arr = np.frombuffer(raw, dtype=dtype).astype(dtype.newbyteorder('='))

        tick = self._unwrap(arr['tick'].astype(np.int64))

        df = pd.DataFrame({'t': tick * dt_us * 1e-6})
        for column in columns:
            if column.name == 'tick':
                continue
            values = arr[column.name]
            df[column.name] = values * column.scale if column.scale != 1.0 else values

        # A gap in the tick sequence is a row the device dropped or one lost on
        # the way in. Either way it is a hole in the timeseries, not a pause.
        gaps = int(np.count_nonzero(np.diff(tick) != dec)) if len(tick) > 1 else 0

        df.attrs.update(dt_us=dt_us, dec=dec, marks=marks, notes=notes,
                        gaps=gaps, tick=tick,
                        units={c.name: c.unit for c in columns},
                        **stats)
        return df

    @staticmethod
    def _unwrap(tick):
        """Undoes the device's 16-bit tick rollover."""
        if len(tick) < 2:
            return tick
        wraps = np.concatenate([[0], np.cumsum(np.diff(tick) < 0)])
        return tick + wraps * 65536
