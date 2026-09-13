"""Segunda tanda: el costo por byte de la UART, y de dónde sale la pérdida de bytes de comando.

Necesita ControlDemo compilado con -DCTRL_PROFILE. Uso: python exp_loop2.py [puerto]
"""
import json
import sys
import time

import exp_loop as e
from ctrllink import CtrlLink

port = sys.argv[1] if len(sys.argv) > 1 else None
dev = CtrlLink(port)
print(dev.info)
out = {}


def bench(n, reps=6):
    r = []
    for _ in range(reps):
        dev.set('x_bench', n)
        time.sleep(0.05)
        r.append((dev.get('x_b_cli'), dev.get('x_b_sei'), dev.get('x_b_fl')))
    return r


def burst(label, n=50, start_stream=False):
    dev.ser.reset_input_buffer()
    buf = bytearray()
    if start_stream:
        e.raw_send(dev, 'start')
        time.sleep(0.3)
    for _ in range(n):
        # Sin espaciar, como los mandaría cualquier terminal.
        dev.ser.write(b'get x_mode\n')
        t = time.monotonic() + 0.02
        while time.monotonic() < t:
            buf += dev.ser.read(max(1, dev.ser.in_waiting))
    t = time.monotonic() + 0.5
    while time.monotonic() < t:
        buf += dev.ser.read(max(1, dev.ser.in_waiting))
    if start_stream:
        for _ in range(4):
            e.raw_send(dev, 'stop')
            t = time.monotonic() + 1.0
            while b'# end' not in buf and time.monotonic() < t:
                buf += dev.ser.read(max(1, dev.ser.in_waiting))
            if b'# end' in buf:
                break
        time.sleep(0.1)
        dev.ser.reset_input_buffer()
    # Una ráfaga que perdió un '\n' deja media línea en el buffer del dispositivo;
    # una línea vacía la cierra antes del comando siguiente.
    for _ in range(2):
        e.raw_send(dev, '')
        time.sleep(0.1)
    buf += dev.ser.read(dev.ser.in_waiting)
    dev.ser.reset_input_buffer()

    text = buf.decode('ascii', 'replace')
    ok = text.count('# v x_mode')
    err = text.count('# err')
    print(f'rafaga [{label}]: {ok}/{n} respuestas, {err} errores, {n - ok - err} sin respuesta')
    out[label] = (ok, err)


try:
    dev.set('ctl_mode', 0)
    dev.set('ctl_uff', 0)

    for n in (23, 45, 62) if '--no-bench' not in sys.argv else ():
        r = bench(n)
        out[f'bench{n}'] = r
        print(f'bench n={n}: write cerrada {sorted(x[0] for x in r)}  '
              f'write abierta {sorted(x[1] for x in r)}  hasta vaciar {sorted(x[2] for x in r)}')

    burst('sin flujo, muestreador andando, modo 0')
    dev.set('x_cmd', 3)
    time.sleep(0.05)
    burst('sin flujo, muestreador PAUSADO')
    dev.set('x_cmd', 4)
    time.sleep(0.05)
    burst('flujo dec=1, modo 0', start_stream=True)
    dev.set('dec', 10)
    burst('flujo dec=10, modo 0', start_stream=True)
    dev.set('dec', 1)
    dev.set('x_mode', 1)
    burst('sin flujo, modo 1 (ctl en ISR, cerrada)')
    dev.set('x_mode', 2)
    burst('sin flujo, modo 2 (ctl en ISR, sei)')
    dev.set('x_mode', 0)
finally:
    with open('exp_loop2.json', 'w') as f:
        json.dump(out, f, indent=1)
    dev.close()
