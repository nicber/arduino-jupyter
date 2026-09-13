"""Tercera tanda: transmisión por encuesta contra HardwareSerial, y 2 Mbaud.

Necesita ControlDemo compilado con -DCTRL_PROFILE (y -DEXP_BAUD para otra velocidad).
Uso: python exp_loop3.py puerto [baud] [etiqueta]
"""
import json
import sys
import time

import exp_loop as e
from ctrllink import CtrlLink

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
tag = sys.argv[3] if len(sys.argv) > 3 else str(baud)

dev = CtrlLink(port, baud=baud)
print(dev.info)

# Reintentos a mano: a 2 Mbaud un byte corrupto sale como «no existe ese
# parametro», que ctrllink no reintenta. Se cuentan, porque eso también es un dato.
retries = {'n': 0}
_get, _set = dev.get, dev.set


def _retry(fn, *a):
    for i in range(6):
        try:
            return fn(*a)
        except Exception:
            retries['n'] += 1
            e.raw_send(dev, '')
            time.sleep(0.1)
            dev.ser.reset_input_buffer()
    return fn(*a)


dev.__dict__['get'] = lambda name: _retry(_get, name)
dev.__dict__['set'] = lambda name, value: _retry(_set, name, value)
_read_all = e.read_all
e.read_all = lambda d: _retry(_read_all, d)
row_len = 4 + sum(c.width for c in dev.channels)
out = {'bench': {}, 'runs': []}

try:
    dev.set('ctl_mode', 0)
    dev.set('ctl_uff', 0)

    for n in (23, 45, 62):
        r = []
        for _ in range(6):
            dev.set('x_bench', n)
            time.sleep(0.05)
            r.append((dev.get('x_b_cli'), dev.get('x_b_sei'), dev.get('x_b_pol')))
        out['bench'][n] = r
        print(f'bench n={n}: HardwareSerial cerrada {sorted(x[0] for x in r)}  '
              f'HardwareSerial abierta {sorted(x[1] for x in r)}  '
              f'encuesta {sorted(x[2] for x in r)}')

    y = dev.get('ang_y_uw')
    dev.set('pid_kp', 0.3)
    dev.set('pid_ki', 0.0)
    dev.set('pid_kd', 0.0)
    dev.set('ctl_target', 0)
    dev.set('ctl_ref', y)
    dev.set('ctl_mode', 1)

    for div, hz in ((10, '500 Hz'), (5, '1 kHz')):
        dev.set('loop_div', div)
        for mode in (0, 2):
            for poll in (0, 1):
                dev.set('x_txpoll', poll)
                label = f'{tag} PID {hz} tx={"encuesta" if poll else "Serial"}'
                o = e.run(dev, label, mode, stream=True, row_len=row_len)
                o['txpoll'] = poll
                o['hz'] = hz
                out['runs'].append(o)
    dev.set('x_txpoll', 0)
    dev.set('loop_div', 10)
finally:
    try:
        dev.set('x_mode', 0)
        dev.set('ctl_mode', 0)
        dev.set('ctl_uff', 0)
    except Exception:
        pass
    print(f'reintentos de comando: {retries["n"]}')
    out['retries'] = retries['n']
    with open(f'exp_loop3_{tag}.json', 'w') as f:
        json.dump(out, f, indent=1)
    dev.close()
