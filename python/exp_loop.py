"""Experimento: dónde se va el tiempo del lazo, y qué cambia con el control en la ISR.

Necesita ControlDemo compilado con -DCTRL_PROFILE (ver el bloque del experimento en
el sketch). Uso:

    python exp_loop.py [puerto]

Cada corrida pone los contadores en cero, deja correr el lazo unos segundos --con
o sin flujo, y opcionalmente con una ráfaga de comandos sin espaciar--, congela los
contadores y los lee con el flujo ya parado. Imprime una tabla por corrida y deja
todo en exp_loop.json.
"""

import json
import sys
import time

from ctrllink import CtrlLink

STATS = ['ctl', 'emt', 'fmt', 'wr', 'pol', 'plc', 'tun', 'mag', 'isr', 'lat', 'lte']
EXTRA = ['x_ovr', 'x_loops', 'x_h100', 'x_h200', 'x_h400', 'x_h800', 'x_hbig',
         'loop_late', 'loop_missed', 'ang_busovr', 'ang_buserr']

MODES = {
    0: 'loop()        ',
    1: 'ISR           ',
    2: 'ISR+sei       ',
    3: 'ISR+emit      ',
    4: 'ISR+emit+sei  ',
}


def raw_send(dev, line, gap=0.0005):
    for b in (line + '\n').encode():
        dev.ser.write(bytes([b]))
        if gap:
            t = time.perf_counter() + gap
            while time.perf_counter() < t:
                pass
    dev.ser.flush()


def read_all(dev):
    values = {}
    for text in dev.cmd('params', timeout=5.0):
        if text.startswith('# p '):
            parts = text.split()
            values[parts[2]] = parts[5]
    return values


def run(dev, label, mode, secs=5.0, stream=False, burst=0, row_len=None):
    dev.set('x_mode', mode)
    dev.set('x_cmd', 1)
    time.sleep(0.05)

    rows = gaps = garbled = replies = errors = 0

    if stream:
        dev.ser.reset_input_buffer()
        raw_send(dev, 'start')
        buf = bytearray()
        t_end = time.monotonic() + secs
        t_burst = time.monotonic() + secs / 2
        sent = 0
        while time.monotonic() < t_end:
            if burst and sent < burst and time.monotonic() >= t_burst:
                # Sin espaciar: los bytes llegan cada 10 us, como los mandaría
                # cualquier terminal.
                dev.ser.write(b'get x_mode\n')
                sent += 1
                t_burst = time.monotonic() + 0.02
            buf += dev.ser.read(max(1, dev.ser.in_waiting))

        raw_send(dev, 'set x_cmd 2')
        time.sleep(0.1)
        for _ in range(4):
            raw_send(dev, 'stop')
            deadline = time.monotonic() + 1.0
            while b'# end' not in buf and time.monotonic() < deadline:
                buf += dev.ser.read(max(1, dev.ser.in_waiting))
            if b'# end' in buf:
                break
        time.sleep(0.1)
        buf += dev.ser.read(dev.ser.in_waiting)

        last = None
        data = False
        for line in buf.decode('ascii', 'replace').split('\n'):
            if line.startswith('# data'):
                data = True
                continue
            if line.startswith('# v x_mode'):
                replies += 1
            elif line.startswith('# err'):
                errors += 1
            if line.startswith('#') or not data or not line:
                continue
            if row_len and len(line) != row_len or any(c not in '0123456789ABCDEF' for c in line):
                garbled += 1
                continue
            tick = int(line[:4], 16)
            if last is not None and ((tick - last) & 0xFFFF) != 1:
                gaps += 1
            last = tick
            rows += 1
        dev.ser.reset_input_buffer()
    else:
        time.sleep(secs)
        dev.set('x_cmd', 2)

    v = read_all(dev)
    out = {'label': label, 'mode': mode, 'secs': secs, 'stream': stream, 'burst': burst,
           'rows': rows, 'gaps': gaps, 'garbled': garbled,
           'replies': replies, 'errors': errors}
    for k in STATS:
        n = int(v[f'x_{k}_n'])
        out[k] = {'max': int(v[f'x_{k}']), 'avg': (int(v[f'x_{k}_s']) / n) if n else 0.0, 'n': n}
    for k in EXTRA:
        out[k] = int(v[k])
    report(out)
    return out


def report(o):
    head = f"== {o['label']}  modo {o['mode']} {MODES[o['mode']].strip()}"
    if o['stream']:
        head += f"  filas={o['rows']} huecos={o['gaps']} rotas={o['garbled']}"
    if o['burst']:
        head += f"  rafaga: {o['replies']}/{o['burst']} respuestas, {o['errors']} err"
    print(head)
    print('   ' + '  '.join(f"{k}:{o[k]['avg']:.0f}/{o[k]['max']}" for k in STATS if o[k]['n']))
    loops_s = o['x_loops'] / o['secs']
    print(f"   loops/s={loops_s:.0f} ovr={o['x_ovr']} late={o['loop_late']} "
          f"missed={o['loop_missed']} busovr={o['ang_busovr']} buserr={o['ang_buserr']} "
          f"hist(<100,<200,<400,<800,+)={[o[k] for k in ['x_h100','x_h200','x_h400','x_h800','x_hbig']]}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    plan = sys.argv[2] if len(sys.argv) > 2 else 'all'
    dev = CtrlLink(port)
    print(dev.info)

    row_len = 4 + sum(c.width for c in dev.channels)
    results = []

    def pid_on():
        # El PID sobre la posición en la que esté el eje: el error arranca en cero
        # y el cálculo corre entero, que es lo que se quiere medir.
        y = dev.get('ang_y_uw')
        dev.set('pid_kp', 0.3)
        dev.set('pid_ki', 0.0)
        dev.set('pid_kd', 0.0)
        dev.set('ctl_target', 0)
        dev.set('ctl_ref', y)
        dev.set('ctl_mode', 1)

    def open_loop(u):
        dev.set('ctl_mode', 0)
        dev.set('ctl_uff', u)

    try:
        if plan in ('all', 'base'):
            open_loop(0)
            for mode in range(5):
                results.append(run(dev, 'abierto, sin flujo', mode, stream=False))
                results.append(run(dev, 'abierto, flujo', mode, stream=True, row_len=row_len))

        if plan in ('all', 'pid'):
            pid_on()
            for mode in range(5):
                results.append(run(dev, 'PID, flujo 500 Hz', mode, stream=True, row_len=row_len))
            open_loop(0)

        if plan in ('all', 'burst'):
            pid_on()
            for mode in range(5):
                results.append(run(dev, 'PID, flujo + rafaga', mode, stream=True,
                                   burst=50, row_len=row_len))
            open_loop(0)

        if plan in ('all', '1k'):
            dev.set('loop_div', 5)
            pid_on()
            for mode in range(5):
                results.append(run(dev, 'PID, flujo 1 kHz', mode, stream=True, row_len=row_len))
            open_loop(0)
            dev.set('loop_div', 10)
    finally:
        try:
            dev.set('x_mode', 0)
            open_loop(0)
        except Exception:
            pass
        dev.close()
        with open('exp_loop.json', 'w') as f:
            json.dump(results, f, indent=1)


if __name__ == '__main__':
    main()
