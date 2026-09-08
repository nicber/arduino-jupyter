"""The bench: this particular rig's build, connect and unit conventions.

Everything here is plumbing. It lives out of the notebook so that a notebook
cell contains a controller and an experiment and nothing else.

    from bench import *

    dev = sync_board()
    dev.gains(kp=0.002, ki=0.05)
    dev.ref = dev.deg(45)
    df = dev.step('ref', dev.deg(90))

`sync_board()` compiles if a source file changed, uploads if the binary changed,
and reopens the link -- which resets the board. Every cell calls it, so every
cell starts from the sketch's own defaults and no cell depends on the one above
it having been run.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import time
from pathlib import Path

from ctrllink import CtrlLink, CtrlLinkError, find_port

__all__ = ['sync_board', 'Bench', 'CtrlLinkError',
           'MODE_OPEN', 'MODE_PID', 'MODE_RAMP', 'POSITION', 'CURRENT']

FQBN      = 'arduino:avr:uno'
_HERE     = Path(__file__).resolve().parent.parent
SKETCH    = _HERE / 'ControlDemo'
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# `mode` picks the controller, `target` picks the feedback it closes on.
MODE_OPEN, MODE_PID, MODE_RAMP = 0, 1, 2
POSITION,  CURRENT             = 0, 1

# AS5600 STATUS bits.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

_link = None


# ----------------------------------------------------------------- the device

class Bench(CtrlLink):
    """A CtrlLink that knows what this rig's numbers mean.

    The board's parameters are all in its own units -- counts, ADC LSBs, gains
    per sample. These turn them into the units an experiment is designed in.
    """

    def channel(self, name):
        for column in self.channels:
            if column.name == name:
                return column
        raise CtrlLinkError(f'no channel called {name!r}')

    # ------------------------------------------------------------ setpoints

    def deg(self, degrees):
        """Degrees of shaft angle -> a `ref` for target = POSITION."""
        return degrees / self.channel('y_uw').scale

    def ma(self, milliamps):
        """Milliamps -> a `ref` for target = CURRENT."""
        return milliamps / self.channel('i').scale

    def rev_per_s(self, revs):
        """Revolutions per second -> a `refrate`, for target = POSITION.

        `refrate` is added to `ref` once per control period, so the speed a
        given rate produces depends on how fast the loop runs.
        """
        return self.deg(revs * 360.0) * self.dt

    # The other direction, for the two channels whose units follow `target`.
    def as_deg(self, values):
        """`ref` or `e`, captured with target = POSITION -> degrees."""
        return values * self.channel('y_uw').scale

    def as_ma(self, values):
        """`ref` or `e`, captured with target = CURRENT -> milliamps."""
        return values * self.channel('i').scale

    # --------------------------------------------------------------- tuning

    def gains(self, kp=0.0, ki=0.0, kd=0.0):
        """PID gains in continuous time: ki per second, kd in seconds.

        The board's gains are per sample, because that is what its arithmetic
        does. dt converts. Set them through `dev.kp` directly to work in the
        board's own terms instead.
        """
        dt = self.dt
        self.kp, self.ki, self.kd = kp, ki * dt, kd / dt

    def smooth(self, which, tau):
        """Set a filter by time constant in seconds; tau = 0 turns it off.

        `which` is 'y' (position), 'i' (current) or 'e' (the error the
        derivative term sees). The board holds the pole itself, alpha, because
        that is what the filter multiplies by.
        """
        dt = self.dt
        self.set(f'alpha_{which}', dt / (tau + dt) if tau > 0 else 1.0)

    def zero(self):
        """Call the shaft's present position zero.

        `y` reads as `offset - counts` wrapped, so moving `offset` down by the
        present `y` puts `offset` on the present count and `y` on zero.
        """
        self.offset = (self.offset - self.y) % 4096
        self.y_uw = 0

    def rest(self):
        """Open loop, zero command. Where every experiment should end."""
        self.mode = MODE_OPEN
        self.uff = 0

    # ------------------------------------------------------------- bringup

    def bringup(self, motor=True, u=120):
        """Checks the hardware, one subsystem at a time.

        Each line is a thing that can be wrong on its own: the link, the loop,
        the magnet, the I2C bus, the current sense, the actuator. Run it first,
        and after any change to the wiring -- a controller tuned against a
        sensor that is not reading is a long afternoon.
        """
        results = []

        def report(label, ok, detail):
            results.append(ok)
            tag = {True: 'ok', False: 'FAIL', None: 'note'}[ok]
            print(f'  [{tag:>4}]  {label:<14}  {detail}')

        print(f'bringup: {self.info}')

        self.rest()

        # 1. The control loop, and whether it is keeping to its period.
        df = self.capture(1.0, warn=False)
        span = df['t'].iloc[-1] - df['t'].iloc[0]
        rate = len(df) / span
        want = 1e6 / df.attrs['dt_us']
        report('control loop', abs(rate - want) < want * 0.02,
               f'{rate:.0f} Hz against {want:.0f} nominal, '
               f'{df.attrs["missed"]} missed, {df.attrs["drops"]} dropped')

        report('timing margin', df.attrs['maxlate'] < df.attrs['dt_us'] // 2,
               f'worst service delay {df.attrs["maxlate"]} us of '
               f'{df.attrs["dt_us"]} us')

        # 2. The magnet, as the AS5600 itself sees it. This is the check that
        #    catches a magnet mounted too far from the die, which otherwise
        #    shows up only as a noisy angle nobody trusts.
        status = self.mstat
        if not status & _MAGNET_PRESENT:
            report('magnet', False, 'not detected -- is it mounted over the chip?')
        elif status & _MAGNET_WEAK:
            report('magnet', False, 'too weak (AGC at maximum) -- move it closer')
        elif status & _MAGNET_STRONG:
            report('magnet', False, 'too strong (AGC at minimum) -- move it away')
        else:
            report('magnet', True, 'detected, AGC in range')

        # 3. The bus carrying the angle, separately from the magnet on the end
        #    of it: a pull-up problem and a mounting problem look the same in
        #    the data and are fixed in different places.
        report('i2c bus', df.attrs['serr'] == 0 and df.attrs['sovr'] == 0,
               f'{df.attrs["serr"]} transfer errors, {df.attrs["sovr"]} overruns')

        spread = df['y_uw'].max() - df['y_uw'].min()
        report('angle', None if spread < 0.5 else True,
               f'{df["y_uw"].iloc[-1]:.1f} deg, moved {spread:.2f} deg over the '
               f'second' + ('  (turn the magnet to see it follow)'
                            if spread < 0.5 else ''))

        # 4. Current sense at rest. A sensor reading far from zero with nothing
        #    driven is an offset that will be integrated into every measurement
        #    after it.
        rest_ma = df['i'].mean()
        report('current sense', abs(rest_ma) < 50,
               f'{rest_ma:+.1f} mA at rest (noise {df["i"].std():.1f} mA)')

        # 5. The actuator, and with it the whole chain: a command out, motion
        #    and current back.
        if motor:
            print(f'  driving the motor at u = {u} for 0.4 s ...')
            self.zero()
            self.uff = u
            spun = self.capture(0.4, warn=False)
            self.rest()

            turned = abs(spun['y_uw'].iloc[-1] - spun['y_uw'].iloc[0]) / 360.0
            drawn = spun['i'].abs().max()
            report('motor', turned > 0.05 or drawn > rest_ma + 50,
                   f'{turned:.2f} rev, {drawn:.0f} mA peak')
        else:
            report('motor', None, 'skipped (motor=False)')

        bad = results.count(False)
        print(f'\n{"all checks passed" if not bad else f"{bad} check(s) FAILED"}')
        return not bad


# ------------------------------------------------------------- build and flash

def _sources_hash():
    """Fingerprint of everything the sketch is built from.

    Contents rather than timestamps: a git checkout rewrites mtimes without
    changing a line, and would otherwise trigger a pointless rebuild.
    """
    digest = hashlib.sha256()
    files = sorted(list(SKETCH.glob('*.ino')) +
                   [p for p in LIBRARIES.rglob('*') if p.suffix in ('.h', '.cpp', '.c')])
    for path in files:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _run(argv, what):
    done = subprocess.run(argv, capture_output=True, text=True)
    if done.returncode:
        raise RuntimeError(f'{what} failed:\n{(done.stdout + done.stderr).strip()}')
    return done.stdout + done.stderr


def _load_state():
    try:
        return json.loads((BUILD_DIR / 'sync-state.json').read_text())
    except (OSError, ValueError):
        return {}


def _save_state(state):
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    (BUILD_DIR / 'sync-state.json').write_text(json.dumps(state, indent=1))


def _wait_for_port(hint=None, timeout=2.0):
    """find_port(), but tolerant of a board that is still re-enumerating.

    The default is short because an absent board should be reported at once; the
    long wait is only worth it just after an upload, when the bridge may
    genuinely take a few seconds to come back.
    """
    deadline = time.monotonic() + timeout
    while True:
        try:
            return find_port(hint)
        except CtrlLinkError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.3)


def sync_board(port=None, force_compile=False, force_upload=False, verbose=True):
    """Bring the board and the link up to date, and reconnect. Returns a Bench.

    Compiles only when a source file has actually changed, uploads only when the
    resulting binary differs from what this port was last given, and always
    reopens the link -- which resets the board, so the loop starts from the
    sketch's defaults whether or not anything needed flashing. That reset is the
    point of calling it at the top of every cell.

    force_compile and force_upload each override their own check.
    """
    global _link

    def say(message):
        if verbose:
            print(message)

    state = _load_state()
    hex_file = BUILD_DIR / f'{SKETCH.name}.ino.hex'
    sources = _sources_hash()
    notes = []

    if force_compile or not hex_file.exists() or state.get('sources') != sources:
        output = _run(['arduino-cli', 'compile', '--fqbn', FQBN,
                       '--libraries', str(LIBRARIES),
                       '--build-path', str(BUILD_DIR),
                       str(SKETCH)], 'compile')
        notes.append('compiled')
        state['sources'] = sources
        _save_state(state)

    binary = hashlib.sha256(hex_file.read_bytes()).hexdigest()

    if port is None:
        try:
            port = _wait_for_port()
        except CtrlLinkError:
            raise CtrlLinkError(
                'the sketch is built, but no board is reachable: no USB serial '
                'port was found. Plug it in and run this again -- the build is '
                'cached, so it will go straight to uploading.') from None

    uploaded = state.get('uploaded', {})

    if force_upload or uploaded.get(port) != binary:
        # Uploading needs the port to itself, and resets the board regardless.
        if _link is not None:
            _link.close()
            _link = None
        _run(['arduino-cli', 'upload', '--fqbn', FQBN, '-p', port,
              '--input-dir', str(BUILD_DIR), str(SKETCH)], 'upload')
        notes.append('uploaded')
        uploaded[port] = binary
        state['uploaded'] = uploaded
        _save_state(state)
        port = _wait_for_port(timeout=15.0)  # some bridges drop off the bus while resetting

    if _link is not None:
        _link.close()

    _link = Bench(port)
    say(f'{port}: {_link.info}' + (f'  ({", ".join(notes)})' if notes else ''))
    return _link
