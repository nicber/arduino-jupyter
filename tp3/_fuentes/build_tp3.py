#!/usr/bin/env python
"""Construye tp3_control_motor.ipynb, el enunciado del TP3. BORRADOR.

    python tp3/_fuentes/build_tp3.py            # deja tp3/tp3_control_motor.ipynb

QUÉ ES. El tercer TP del motor: el TP1 lo modeló, el TP2 lo identificó en el banco
(K y tau alrededor de un punto de trabajo) y éste cierra el lazo de velocidad. El
hilo es la robustez: a lazo abierto cada diferencia entre el motor y su modelo
aparece entera en la velocidad; a lazo cerrado no, y se mide cuánto y a qué precio.
Retoma con el motor propio lo que la práctica 3 hizo a mano con un motor de libro
(lazo_abierto_vs_cerrado, ruido_sensor) y usa el diseño de la práctica 6.

DECISIONES (cátedra, oct. 2026):
  - Se diseña sobre el modelo y se valida en el banco real.
  - Del TP2 se trae K y tau en UN punto de trabajo. Que el modelo valga sólo ahí es
    el primer "error de identificación", y sale gratis (ítem 3.1).
  - El único cambio físico del motor es frenar el disco con un dedo: es una
    perturbación de carga, no repetible, así que se la pide cualitativa (1.5, 2.5).
    Los cambios de parámetros salen del punto de trabajo (3.1), de diseñar con un
    modelo equivocado a propósito (3.2) y del banco simulado (3.12).
  - Bancos con actuadores mezclados (B puente, B′ transistor): los requisitos son
    relativos al modelo de cada banco, y la bajada (3.10) se contesta según el actuador.
  - Controladores: un proporcional primero, y después el alumno elige cómo cumplir
    requisitos en el dominio del tiempo. El enunciado no nombra un PI.
  - D(s) en la placa es una ganancia por una cadena de bloques de un cero y un polo,
    (s + z)/(s + p), configurables. Los alumnos configuran, no editan el sketch.
  - Un PI sobre la POSICIÓN se inestabiliza si no se lo diseña con cuidado, y lo
    tienen que descubrir ellos (parte B del ejercicio 3). Se plantea como seguimiento
    de un ángulo que avanza a w_e, y no como un servo alrededor de cero: así el eje
    gira siempre para el mismo lado --sirve en un banco B′--, la planta es la del TP2
    con un integrador más, K/(s (tau s + 1)), donde el modelo vale, y no hay zona
    muerta. El 3.6 les hace cargar el bloque (1/tau, 0), el mejor PI de velocidad, sin
    avisar: L queda con dos integradores y MF = 0. El 3.7 pide el porqué: por Routh,
    estable si y sólo si z < 1/tau, sin que importe Kc.
  - Recorrido esencial marcado (●) de 5 a 6 h; el resto es profundización (○). Al
    entrar la parte B, el escalón grande y la bajada pasaron a profundización (3.10).
  - Registro impersonal, como CONVENCIONES.md y hardware.ipynb (el TP1 vosea).

LO QUE TODAVÍA NO EXISTE (y este enunciado supone). El firmware no tiene lazo de
velocidad ni cadena de bloques. Nombres PROVISIONALES, a confirmar al implementarlo:
  - ctl_target = velocidad, `ctl_ref` en rad/s, `ctl_uff` en % como prealimentación;
  - `vel_win`: períodos de 2 ms sobre los que se diferencia el ángulo;
  - `dev.compensador(Kc, [(z, p), ...])`, hasta tres bloques, z y p en rad/s, p = 0
    para un integrador; cada bloque discretizado por Tustin;
  - un interruptor del anti-windup del integrador (ítem 3.10);
  - el seguimiento de un ángulo (parte B): el lazo sobre el ángulo con una referencia
    que avanza a w_e --`ctl_rate` de MODE_RAMP ya hace eso en ControlDemo--, un salto
    de esa referencia (`ctl_fase`) y el error en un canal, en radianes;
  - canales de telemetría con la referencia, la velocidad que usa el lazo y u.
Preguntas abiertas del firmware: si hace falta un bloque sin cero (pasabajos) o sin
polo; dónde se calculan los coeficientes (del lado de la computadora, como el resto);
la velocidad en punto fijo; el flash que queda en ControlDemo. La referencia de lo
que tiene que hacer está en `lazo_simulado.py`, que es además lo que hoy necesita el
ítem 3.12: el banco simulado todavía no cierra el lazo.

REQUISITOS, y por qué estos. Escalón de ensayo: de w0 a la mitad del escalón del
TP2, para dejarle recorrido al actuador. R2 (ts <= tau) da K Kp >= 2 a un
proporcional, que queda con 33 % de error; R1 (2 %) le pide K Kp >= 49, que satura
y sacude el comando. Con un PI que cancela el polo queda una ventana de diseño de
un factor dos: 3/K <= Kc <= (100 - u0)/salto. R4 es lo que le pone techo.

CLAVE (clave_tp3.py, contra el banco simulado: B′, K = 6,07 (rad/s)/%, tau = 0,81 s,
escalón de ensayo de 417,7 a 478,4 rad/s). En un banco real cambian los números;
se tiene que repetir el sentido de cada comparación.
  1.1 Lazo abierto: ts = 3 tau = 2,4 s (medido 2,75); llega a 488 en vez de 478: la
      K del modelo es el promedio del escalón del TP2, no la de este tramo.
  1.2 Kp = 2/K = 0,33: ts 0,81 s y error 33 % predichos; 0,89 s y 29 % medidos.
      u(0+) = u0 + Kp salto = 60 %.
  1.4 Kp >= 49/K = 8,1: u(0+) = 530 %, satura; error 1,7 %, std(u) 8,5 %. Ningún Kp
      cumple R1 y R4 a la vez. El ruido llega a u por U/V = -D S (práctica 6, 3 (g)).
  1.5 Carga de 40 uN·m: lazo abierto cae de 488 a 439; el P, de 481 a 465 con u de
      49 a 54 %. No vuelve: Y/W en continua es K_l K/(1 + K Kp), no cero.
  2.1 R1: tipo 1, o K D(0) >= 49. R2: polos dominantes con sigma >= 3/tau. R3:
      zeta >= 0,59. R4: ganancia de alta frecuencia D(inf) <= (100 - u0)/salto = 0,99.
  2.2 Referencia: D = Kc (s + 1/tau)/s. L = Kc K/(tau s), T = 1/(tau_c s + 1) con
      tau_c = tau/(K Kc). Ventana 0,49 <= Kc <= 0,99. Con Kc = 0,8: ts 0,50 s
      predicho, 0,47 medido; Mp 1,2 %; u hasta 89 %; std(u) 0,84 %. MF = 90 grados.
  2.5 Con el PI la velocidad vuelve a 478,4 y u pasa de 48,6 a 56,7 %: el integrador
      está antes del punto donde entra la carga (práctica 6, problema 6 (c)).
  2.6 Cero en 2/tau: Mp 8,4 %; en 4/tau: 17,6 % (no cumple R3). Atraso con beta = 20
      y prealimentación u0: error 0,8 %, cumple todo. Ventana de velocidad con el PI:
      std(u) 3,0 % (1 período), 0,84 % (5), 0,14 % (25).
  3.1 El mismo PI, saltos iguales: desde 150 rad/s ts 0,27 s y Mp 2,9 %; desde 418,
      0,47 s; desde 539, 0,78 s. Error nulo en los tres. K local, de los u de régimen:
      15, 7 y 4. A lazo abierto, para 150 rad/s el modelo pide un comando negativo y
      para 211, un 6 %: el motor ni arranca.
  3.2 Diseñando con K a la mitad: ts 0,31 s y satura; con K al doble: 0,88 s; error
      nulo en los dos. A lazo abierto el mismo error de K da -100 % y +38 % del salto.
      Con tau x0,7 el cero ya no cancela: Mp 5,3 % y una cola (ts 1,16 s).
  3.3 Sin retardo L = Kc K/(tau s): MG infinito. Con Td ~ Ts/2 + N Ts/2 + 1 ms y
      vel_win = 25: Td = 27 ms, wc = 6,0 rad/s, MF = 80,7 grados, MG = 9,7 en
      58 rad/s, margen de retardo 234 ms. Con vel_win = 5: Td = 7 ms, MG = 38.
  3.4 vel_win = 25: x10 todavía se establece (Mp 13 %), x20 oscila sostenido entre los
      topes. Desde 150 rad/s, donde K es 2,5 veces mayor, x3 ya da Mp 17 %. No diverge:
      el recorte lo convierte en un ciclo límite. Con vel_win = 5 lo que aparece antes
      es el ruido: std(u) 8 % con x10.
  3.5 Theta/U = K/(s (tau s + 1)). tau s^2 + s + K Kp = 0: zeta = 1/(2 sqrt(K Kp tau)),
      así que Kp = 1/(K tau) = 0,204 %/rad da zeta = 0,5, wn = 1/tau, Mp 16 %, ts 4,8 s,
      MF = 51,7 grados. Medido: Mp 9,6 %, ts 7,3 s (la K local es mayor que la del
      modelo y la planta no es lineal). Error de seguimiento: -6,5 rad, una vuelta
      entera. Sale de que u_ff no es exacto: equivale a una perturbación constante a la
      entrada de la planta, y vale (lo que le sobra a u_ff)/Kp. A lazo abierto ese
      mismo error de velocidad se integra y el ángulo se aleja sin límite.
  3.6 Bloque (1/tau, 0) con Kc = 0,204: L = Kc K/(tau s^2), polos en +-j 1,24 rad/s,
      MF = 0 (-0,1 grados con el retardo). Medido: el error oscila con período de unos
      5 s y no se apaga: +-16 rad a los 20 s, +-116 rad a los 60 s, con u entre 12 y
      84 %. Con Kc = 0,8 tampoco se apaga: +-7,5 rad a los 20 s, a 2,45 rad/s.
  3.7 tau s^3 + s^2 + K Kc s + K Kc z = 0. Routh: K Kc > tau K Kc z, o sea z < 1/tau,
      para cualquier Kc. Centro de las asíntotas: (z - 1/tau)/2, que cambia de signo
      ahí mismo. En velocidad el cero en 1/tau dejaba UN integrador (MF 90 grados); acá
      deja dos. z = 2/tau: par en +0,22 +- j1,49, crece hasta los topes en menos de
      20 s. z = 1/(2 tau): estable, zeta = 0,2, MF 19 grados, Mp 68 %.
  3.8 Dos referencias. PI lento, Kc = 0,204 y z = 0,1/tau: MF 44 grados, Mp 19 %, ts
      9,0 s, y el integrador tarda decenas de segundos en absorber el error de u_ff.
      PI más adelanto, Kc = 2 con (0,5/tau, 0) y (1/tau, 10/tau): MF 59 grados, Mp 21 %,
      ts 5,6 s, error nulo; u(0+) del salto +12,6 %. Lo que se paga por el integrador:
      fase, que se recupera con una cola lenta o con más esfuerzo de control.
  3.10 De 200 a 560: con anti-windup sin sobrepico, ts 1,09 s; sin él llega a 588
      (Mp 7,6 %), ts 1,57 s. De 560 a 300 en B′: u = 0 durante 0,83 s, ts 1,62 s (a
      lazo abierto 3,6 s); sin anti-windup cae hasta 205 y tarda 3,5 s. En un banco B
      el comando puede ir a negativo y la bajada no depende del rozamiento: no está
      simulado.
  3.12 Inercia x2: Mp 7,9 %, ts 2,0 s (el cero dejó de cancelar). Fuente de 4,2 V: ts
      0,94 s, u llega al tope; a lazo abierto va de 337 a 398 en vez de 418 a 478.
"""
import sys
from pathlib import Path

import nbformat as nbf

cells = []
def md(src):   cells.append(nbf.v4.new_markdown_cell(src.strip('\n')))
def code(src): cells.append(nbf.v4.new_code_cell(src.strip('\n')))


md(r"""
# Trabajo Práctico 3 · Control de velocidad del motor identificado

**Control Clásico y por Variables de Estado** · Ingeniería Electrónica ·
Universidad Nacional de Río Negro · 2026

---

### De qué se trata

En el TP1 se modeló un motor de continua y en el TP2 se identificó el del banco: una
ganancia $K$ y una constante de tiempo $\tau$ alrededor de un punto de trabajo. En este TP
ese modelo se usa para lo que se lo hizo: **diseñar un controlador de velocidad**, cargarlo
en la placa y comprobar sobre el motor si hace lo que el modelo predijo.

La pregunta que recorre los tres ejercicios es qué pasa cuando **el motor no es el del
modelo**: porque se lo identificó en un solo punto de trabajo, porque se lo identificó con
error, porque algo frena el eje, porque el lazo real tiene cosas que el modelo no tiene, o
porque al mismo controlador se le pide otra cosa.
A lazo abierto cada una de esas diferencias aparece entera en la velocidad. A lazo cerrado
no, y el TP pide medir cuánto se reduce y cuál es "el precio".

No hay temas nuevos. Se usan la práctica 3 (realimentación, sensibilidad, perturbaciones y
ruido), la práctica 5 (márgenes de estabilidad) y la práctica 6 (diseño de compensadores).

### Cómo se trabaja

Cada ítem sigue el mismo orden: **predecir** con el modelo, **medir** en el banco y
**explicar** la diferencia. La predicción va primero y queda escrita antes de medir: una
predicción hecha después de ver la medición no dice nada sobre el modelo.

### Qué se entrega

Este mismo notebook, completado, con:

- las cuentas de diseño y las predicciones, con sus números;
- los gráficos con **título, ejes con unidades y leyenda**, y en cada ensayo de lazo cerrado
  la velocidad **y** la señal de control $u$;
- una tabla *predicho / medido* por ejercicio;
- y **una conclusión escrita** por ejercicio.

### Recorrido

Los ítems marcados con ● forman el recorrido esencial, de unas 5 a 6 horas. Los marcados
con ○ son de profundización: cada uno retoma un ítem esencial y lo lleva más lejos.

### Dónde buscar los comandos

En `recetario_control.ipynb` (§3.3, §3.5, §5.3, §5.5 y §5.6) y en `notebooks/hardware.ipynb`,
sección 5, para todo lo que es capturar y guardar ensayos.
""")

md(r"""
<div class="alert alert-block alert-danger">
<b>BORRADOR.</b> El lazo de velocidad y la cadena de bloques todavía no están en el
firmware, y tampoco el seguimiento de un ángulo de la parte B del ejercicio 3. Los nombres
de la interfaz son provisionales, y el ítem 3.12
necesita que el banco simulado cierre el lazo. Ver la cabecera de
<code>tp3/_fuentes/build_tp3.py</code>. Esta celda se quita al publicar.
</div>
""")

code(r"""
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

print('python-control:', ctrl.__version__)

plt.rcParams.update({'figure.figsize': (9, 3.8), 'font.size': 12,
                     'axes.grid': True, 'grid.alpha': 0.3, 'lines.linewidth': 2})
""")

md(r"""
---
## El lazo

### La planta

El modelo es el del TP2: alrededor del punto de trabajo $(u_0,\ \omega_0)$,

$$
\frac{\Delta\Omega(s)}{\Delta U(s)} = G(s) = \frac{K}{\tau s + 1},
$$

con $u$ en por ciento del comando máximo y $\omega$ en rad/s. $\Delta$ indica la desviación
respecto del punto de trabajo: $\Delta u = u - u_0$, $\Delta\omega = \omega - \omega_0$. Los
datos son los del escalón de identificación, de $u_0$ a $u_1$, que llevó la velocidad de
$\omega_0$ a $\omega_1$.
""")

code(r"""
# Los datos del banco propio, del TP2. COMPLETAR.
BIDIR = None            # True con puente en H (banco B), False con transistor (banco B′)

u0, u1 = None, None     # %      el escalón de identificación
w0, w1 = None, None     # rad/s  las velocidades de régimen antes y después
K, tau = None, None     # (rad/s)/% y s

Ts = 0.002              # s, el período del lazo en la placa (500 Hz)
""")

md(r"""
### Lo que hace la placa

Cada $T_s = 2$ ms la placa:

1. calcula la velocidad como la diferencia del ángulo medido sobre $N$ períodos,
   $\omega_m[k] = \dfrac{\theta[k]-\theta[k-N]}{N\,T_s}$;
2. forma el error $e = r - \omega_m$;
3. calcula $u = u_{ff} + D(e)$, donde $u_{ff}$ es una prealimentación constante;
4. recorta $u$ a lo que el actuador puede dar: de $0$ a $100\,\%$ con un transistor (banco B′),
   de $-100$ a $100\,\%$ con un puente en H (banco B).

```
                                    u_ff
                                     │
  r ──►(+)── e ──►  D(s)  ──────────►(+)──► recorte ── u ──► motor ──┬──► ω
        ▲ −                                                          │
        │                                                            │
        └───── ω_m ◄── diferencia del ángulo sobre N períodos ◄── θ ─┘
```

### El controlador: una cadena de bloques

$D(s)$ no es un controlador fijo. Es una ganancia por una cadena de hasta tres bloques
iguales, cada uno con **un cero y un polo** que se eligen:

$$
D(s) = K_c\,\prod_{i=1}^{3}\frac{s+z_i}{s+p_i},\qquad z_i,\ p_i \ge 0 .
$$

Un bloque con $z_i = p_i$ no hace nada, y uno con $p_i = 0$ integra. Con eso se arman los
compensadores de la práctica 6:

| Compensador | Bloques $(z,\ p)$ |
|---|---|
| proporcional | ninguno |
| PI, $K_c\,\dfrac{s+z}{s}$ | $(z,\ 0)$ |
| adelanto | $(z,\ p)$ con $z<p$ |
| atraso | $(z,\ p)$ con $z>p$ |
| adelanto-atraso | un bloque de cada uno |
| PID con filtro, $K_c\,\dfrac{(s+z_1)(s+z_2)}{s\,(s+p)}$ | $(z_1,\ 0)$ y $(z_2,\ p)$ |

$K_c$ está en $\%/(\text{rad/s})$, y los ceros y polos en rad/s.

### La interfaz

```python
dev.vel_win = 5                                 # N: la ventana de la velocidad, en períodos
dev.compensador(Kc, [(z1, p1), (z2, p2)])       # la ganancia y los bloques que se usen
dev.ctl_uff = u0                                # la prealimentación, en %
df = dev.step('ctl_ref', w_e, pre=3, post=3)    # escalón de referencia en rad/s; t = 0 en el escalón
```

El lazo abierto es el caso $K_c = 0$: sale $u = u_{ff}$.

### El escalón de ensayo y los requisitos

Todos los diseños se juzgan con el mismo ensayo: un escalón de referencia de $\omega_0$ a

$$
\omega_e = \frac{\omega_0+\omega_1}{2},
$$

la mitad del escalón del TP2, para dejarle recorrido al actuador. Se pide:

- **(R1)** error de régimen no mayor que el $2\,\%$ del salto;
- **(R2)** tiempo de establecimiento al $5\,\%$ no mayor que $\tau$, es decir, tres veces más
  rápido que a lazo abierto;
- **(R3)** sobrepico no mayor que el $10\,\%$;
- **(R4)** el comando no llega a ninguno de sus topes durante el ensayo y, ya en régimen, su
  desviación estándar no supera $1\,\%$.

R4 no es un requisito sobre la velocidad sino sobre el actuador: mientras $u$ no se recorta
el lazo es lineal y la predicción vale, y un comando que se sacude gasta al actuador sin
mover al eje.

> Al medir tiempos de establecimiento con `step_info`, pasar `SettlingTimeThreshold=0.05`.
""")

md(r"""
---
## Ejercicio 1 · Lazo abierto, proporcional, y lo que un proporcional no puede

**1.1 ●  Lazo abierto.** El modelo, invertido, dice qué comando da cada velocidad:
$u = u_0 + (r-\omega_0)/K$. Predecir el tiempo de establecimiento al $5\,\%$ y el valor final
del escalón de ensayo. Medirlo con $K_c = 0$ y ese comando como prealimentación. ¿Llega a
$\omega_e$?

**1.2 ●  Proporcional.** Con $u = u_0 + K_p\,(r-\omega_m)$, obtener la transferencia de lazo
cerrado de $\Delta R$ a $\Delta\Omega$, su constante de tiempo y su error de régimen, como
funciones de $K\,K_p$. Elegir el menor $K_p$ que cumple R2. Predecir con él el error de
régimen y el comando en $t=0^+$.

**1.3 ●  Medir** el escalón de ensayo con ese $K_p$ y comparar tiempo de establecimiento,
error de régimen y comando máximo con la predicción.

**1.4 ●  El que cumple R1.** Calcular el $K_p$ que exige R1 y el comando en $t=0^+$ que le
corresponde. Medir el ensayo con ese $K_p$: ¿qué pasa con $u$ durante el escalón y en
régimen? Mostrar que ningún $K_p$ cumple R1 y R4 a la vez. Un ruido $v$ en la medición
llega al comando por $U/V = -D\,S$ (práctica 6, problema 3): ¿cuánto vale esa transferencia
en alta frecuencia con un proporcional?

**1.5 ●  Una carga.** Con la referencia fija en $\omega_e$, capturar unos segundos y, durante
la captura, frenar el disco apoyando apenas la yema de un dedo sobre su borde liso, un par
de segundos. Hacerlo a lazo abierto y con el proporcional de 1.2. La carga es un par
resistente que entra a la entrada de la planta. ¿Qué hace $u$ en cada caso? ¿Vuelve la
velocidad a $\omega_e$ mientras dura la carga? Explicarlo con la ganancia en continua de la
transferencia de la carga a la velocidad.

**1.6 ○  Sensibilidad.** Con $S^T_K = \dfrac{K}{T}\dfrac{\partial T}{\partial K}$, estimar
cuánto cambia la velocidad de régimen, a lazo abierto y con el proporcional de 1.2, si la
ganancia real es un $20\,\%$ menor que la identificada. Comparar con el valor exacto. ¿Por
qué no coinciden?

<div class="alert alert-block alert-info">
<b>En la industria.</b> La prealimentación constante $u_0$ es lo que en los primeros
controladores de proceso se llamó <i>reset manual</i>: el operador la ajustaba hasta anular
el error, y la volvía a ajustar cada vez que cambiaba la carga. El lazo abierto de 1.1 es un
variador de velocidad sin realimentación, o un ventilador sin tacómetro: la velocidad queda
a merced de la carga, de la tensión de alimentación y de la temperatura.
</div>
""")

code(r"""
# COMPLETAR ejercicio 1
""")

md(r"""
| Ejercicio 1 | predicho | medido |
|---|---|---|
| lazo abierto: $t_s$ (5 %), valor final | | |
| P de 1.2: $K_p$, $t_s$ (5 %), error, $u$ máximo | | |
| P de 1.4: $K_p$, error, $u$ máximo, desviación estándar de $u$ | | |

**Conclusión del ejercicio 1:** *(escribirla acá: qué puede un proporcional, qué no, y por qué)*
""")

md(r"""
---
## Ejercicio 2 · Un controlador que cumpla los requisitos

Acá no se indica qué compensador usar: se elige, y se justifica la elección.

**2.1 ●  Traducir los requisitos.** Llevar R1, R2 y R3 a condiciones sobre el lazo: el tipo
o la ganancia en continua de $L(s) = D(s)\,G(s)$, y la región del plano $s$ donde tienen que
quedar los polos dominantes. Para R4, mostrar que el comando en $t=0^+$ del ensayo es el de
antes del escalón más $D(\infty)$ por el salto de referencia, y obtener de ahí una cota
para la ganancia de alta frecuencia del compensador.

**2.2 ●  Diseñar.** Elegir una arquitectura con hasta tres bloques y diseñarla sobre el modelo,
por lugar de las raíces o por respuesta en frecuencia. Entregar $K_c$ y los pares $(z_i,\ p_i)$,
y la predicción: polos de lazo cerrado, sobrepico, tiempo de establecimiento, error de
régimen, comando en $t=0^+$ y margen de fase.

**2.3 ●  Verificar sobre el modelo.** Simular el escalón de ensayo con `python-control` y
graficar la velocidad y el comando, que sale de $U/R = D\,S$. Comprobar R1 a R3 y que el
comando no llega a los topes.

**2.4 ●  Medir.** Cargar el compensador en la placa y medir el escalón de ensayo. Completar la
tabla *predicho / simulado / medido* para R1 a R4. Si algún requisito no se cumple, corregir
el diseño y dejar escrito qué se cambió y qué indicaba el modelo: un ajuste por tanteo, sin
esa explicación, no cuenta como diseño.

**2.5 ●  La carga otra vez.** Repetir el ensayo de 1.5 con este controlador. ¿Vuelve la
velocidad a $\omega_e$ mientras dura la carga? ¿Qué hace $u$? Si vuelve, ¿qué tiene este
compensador que no tenía el proporcional, y en qué lugar del lazo tiene que estar respecto
del punto donde entra la carga? Si no vuelve, ¿cuánto queda, y es lo que predice el modelo?

**2.6 ○  Otra arquitectura, o la ventana.** Una de las dos:

- diseñar un segundo compensador de arquitectura distinta que también cumpla, y comparar
  los dos en el ensayo y frente a la carga;
- con el compensador de 2.2, medir la desviación estándar de $u$ en régimen con `vel_win`
  en 1, 5 y 25 períodos. ¿Qué se gana y qué se paga con una ventana larga? El ejercicio 3
  vuelve sobre esto.

**2.7 ○  Un bloque por dentro.** La placa no resuelve ecuaciones diferenciales: cada bloque es
una ecuación en diferencias. Reemplazando $s = \dfrac{2}{T_s}\,\dfrac{1-q^{-1}}{1+q^{-1}}$
(la aproximación de Tustin, con $q^{-1}$ el retardo de un período) en $\dfrac{s+z}{s+p}$,
obtener $y[k]$ en función de $x[k]$, $x[k-1]$ e $y[k-1]$. Programarla en Python y comparar
su respuesta al escalón con la del bloque continuo, con $p$ mucho menor que $2/T_s$ y con
$p$ comparable. ¿Hasta qué polo se le puede creer al diseño hecho en $s$?

<div class="alert alert-block alert-info">
<b>En la industria.</b> Un servoamplificador o un variador no ofrece «un PID»: ofrece una
cadena de bloques configurables --ganancia, integrador, adelanto-atraso, filtros-- y el
compensador se arma eligiendo cuáles usar. Adentro, cada bloque es una ecuación en
diferencias de orden bajo. Encadenar secciones de primer y segundo orden, en lugar de
programar un único cociente de polinomios de orden alto, es la práctica habitual: es mucho
menos sensible al redondeo de los coeficientes. La acción integral, por su parte, nació
como <i>reset automático</i>: lo que en 1.1 se ajustaba a mano.
</div>
""")

code(r"""
# COMPLETAR ejercicio 2
""")

md(r"""
| Ejercicio 2 | predicho | simulado | medido |
|---|---|---|---|
| R1: error de régimen | | | |
| R2: $t_s$ (5 %) | | | |
| R3: sobrepico | | | |
| R4: $u$ máximo en el ensayo | | | |
| R4: desviación estándar de $u$ en régimen | — | — | |

**Conclusión del ejercicio 2:** *(escribirla acá: por qué esa arquitectura, y dónde el
motor se apartó del modelo)*
""")

md(r"""
---
## Ejercicio 3 · El mismo controlador, otra planta

El compensador de 2.2 **no se toca** en este ejercicio, salvo donde se lo dice.

### A. La planta no es la del modelo

**3.1 ●  Otro punto de trabajo.** Repetir un escalón del mismo tamaño que el de ensayo desde
una velocidad baja, del orden de $\omega_0/3$, y desde $\omega_1$ con medio salto. Hacerlo
con el compensador y a lazo abierto, con el modelo invertido de 1.1. Tabular valor final,
tiempo de establecimiento y sobrepico. ¿Qué se mantuvo a lazo cerrado y qué cambió? Con los
comandos de régimen de antes y después de cada escalón, estimar la ganancia $K$ local de
cada tramo y compararla con la identificada.

**3.2 ●  Un modelo equivocado.** Rehacer el diseño de 2.2, con el mismo procedimiento,
suponiendo que la identificación dio una $K$ del doble de la real, y otra vez con la mitad.
Predecir, sobre el modelo propio, qué pasa con el tiempo de establecimiento y con el
comando en cada caso, y medir uno de los dos. Calcular qué error de régimen daría ese mismo
error de $K$ a lazo abierto. ¿Cuál de las dos estrategias necesita conocer $K$ con precisión,
y para qué la necesita la otra?

**3.3 ●  Lo que el modelo no tiene.** Calcular el margen de ganancia de $L = D\,G$ con el
modelo identificado. El resultado no es creíble: al lazo real le falta, por lo menos, un
retardo. Estimarlo como media retención ($T_s/2$), más medio ancho de la ventana de
velocidad ($N\,T_s/2$), más alrededor de 1 ms entre el sensor y el PWM. Con `vel_win = 25`,
agregar $e^{-sT_d}$ al lazo y calcular la frecuencia de cruce, el margen de fase, el margen
de ganancia y el margen de retardo. ¿Por cuánto se podría multiplicar $K_c$?

**3.4 ●  Buscar el límite.** Con `vel_win = 25`, multiplicar $K_c$ por 2, 5, 10, 20… y medir
el ensayo cada vez, hasta que la oscilación no se apague. Comparar ese factor con el margen
de ganancia de 3.3. Repetir desde la velocidad baja de 3.1: ¿dónde aparece antes la
oscilación, y por qué, a la luz de la $K$ local? La amplitud no crece sin límite: ¿qué la
detiene?

### B. Se le pide otra cosa: seguir un ángulo

Hasta acá se reguló la velocidad. Ahora la referencia es un **ángulo que avanza** a velocidad
constante, $\theta_r(t) = \omega_e\,t$, y el lazo se cierra sobre el ángulo medido:
$e = \theta_r - \theta$. El eje gira siempre en el mismo sentido y alrededor del mismo punto
de trabajo, así que el ensayo vale con cualquiera de los dos actuadores. La planta, del
comando al ángulo, es la del TP2 con un integrador más:

$$
\frac{\Delta\Theta(s)}{\Delta U(s)} = \frac{K}{s\,(\tau s+1)} .
$$

El ángulo se usa como lo entrega el sensor: en este lazo no hay ventana de velocidad. $K_c$
pasa a estar en $\%/\text{rad}$, y la prealimentación es el comando que, según el modelo, da
$\omega_e$: el de 1.1. El ensayo es un **salto de una vuelta** ($2\pi$ rad) en la referencia
mientras avanza, y se miran el error de seguimiento $e(t)$ y el comando.

```python
dev.seguir_angulo(w_e, uff=u_e)                       # la referencia avanza a w_e rad/s
df = dev.step('ctl_fase', 2*np.pi, pre=5, post=30)    # salto de una vuelta; `e` es el error, en rad
```

> Si en un ensayo de esta parte el error crece en lugar de apagarse, se lo corta volviendo
> a $K_c = 0$.

**3.5 ●  Un proporcional.** Con $D = K_p$, obtener la ecuación característica y elegir $K_p$
para $\zeta = 0{,}5$. Predecir el sobrepico y el tiempo de establecimiento al $5\,\%$ ante el
salto de una vuelta, y medirlos. Antes del salto, con la referencia avanzando, ¿el error de
seguimiento es nulo? La planta es de tipo 1: explicar de dónde sale ese error, a qué
entrada del lazo equivale y de qué depende su tamaño. ¿Qué haría ese error a lazo abierto?

**3.6 ●  Anular el error de seguimiento.** En el lazo de velocidad el error de régimen se
anuló con un integrador, y un PI con el cero sobre el polo de la planta, $z = 1/\tau$, es
ahí un diseño natural: cancela el polo y deja un lazo de primer orden. Cargar ese mismo
bloque, $(1/\tau,\ 0)$, con la ganancia de 3.5, y medir el salto de una vuelta durante 30 s
por lo menos. Describir qué hacen el error y el comando.

**3.7 ●  Explicar lo que se vio.** Escribir $L(s)$ con ese compensador: ¿cuántos integradores
tiene, y cuánto vale su margen de fase? Con un cero cualquiera, $D = K_c\,(s+z)/s$, obtener
la ecuación característica y, por Routh, la condición de estabilidad. ¿Depende de $K_c$?
Comprobarla con el centro de las asíntotas del lugar de las raíces. Simular sobre el modelo
con $z = 2/\tau$ y con $z = 1/(2\tau)$, y medir uno de los dos en el banco. ¿Por qué el cero
que en velocidad era una buena elección acá no lo es?

**3.8 ●  Con cuidado.** Diseñar un compensador que anule el error de seguimiento con un margen
de fase no menor que $40^\circ$ y un sobrepico no mayor que el $30\,\%$ ante el salto de una
vuelta, sin que el comando llegue a los topes. Alcanza con un PI si su cero se elige bien;
con un bloque más se puede recuperar fase. Predecir, medir, y comparar con el proporcional
de 3.5: ¿qué se pagó por el integrador?

**3.9 ●  Balance.** Completar la tabla del final y escribir la conclusión.

### Profundización

**3.10 ○  Escalón grande, y la bajada.** Con el lazo de velocidad, `vel_win = 5` y el
compensador de 2.2, medir un escalón de subida lo bastante grande para que el comando quede
en el tope varias décimas de segundo, con el anti-windup de la placa activado y desactivado.
Después, un escalón de bajada del mismo tamaño. Indicar qué actuador tiene el banco y
explicar, con el recorte de $u$, qué limita la bajada. ¿Hay algún compensador que la haga
más rápida?

**3.11 ○  Ruido o estabilidad.** Repetir 3.3 y 3.4 con `vel_win = 5`. El margen de ganancia
predicho es varias veces mayor: ¿se lo alcanza? ¿Qué aparece antes? Relacionarlo con 1.4 y
con 2.6.

**3.12 ○  Otro motor.** En el banco simulado se pueden cambiar los parámetros del motor. Con
el compensador de 2.2 y con el lazo abierto de 1.1, repetir el escalón de ensayo con la
inercia al doble y con la fuente a un $85\,\%$ de su tensión. ¿Qué cambia en cada estrategia?
¿Cuál de los dos cambios se ve en $u$ y cuál no?

<div class="alert alert-block alert-info">
<b>En la industria.</b> La ganancia que cambia con el punto de trabajo (3.1) se trata con
<i>ganancia programada</i>: una tabla de ajustes por zona, o un bloque que invierte la curva
estática del actuador. Los márgenes de 3.3 se especifican --6 dB y 45° son valores usuales--
no porque el modelo los necesite, sino como presupuesto para lo que el modelo no tiene. El
ensayo de 3.4 es, en esencia, lo que hace el autoajuste de un controlador comercial: lleva el
lazo a una oscilación controlada y lee de ahí la ganancia límite.
<br/><br/>
Seguir un ángulo que avanza es lo que hace un <i>eje electrónico</i>: dos motores que giran
sincronizados sin un eje mecánico que los una, como en una impresora rotativa o en una
bobinadora. Y lo que aparece en 3.6 explica la estructura de casi todos los
servoamplificadores: un lazo de velocidad PI por dentro y, por fuera, un lazo de posición
sólo proporcional. El integrador va en el lazo de velocidad.
<br/><br/>
La bajada de 3.10 es la razón por la que un variador trae rampas de aceleración y de frenado
configurables, y una resistencia de frenado cuando la carga tiene que detenerse rápido:
ningún controlador da la autoridad que el actuador no tiene.
</div>
""")

code(r"""
# COMPLETAR ejercicio 3
""")

md(r"""
| Frente a… | Lazo abierto | Lazo cerrado | Qué costó |
|---|---|---|---|
| una carga en el eje (1.5, 2.5) | | | |
| otro punto de trabajo (3.1) | | | |
| una $K$ mal identificada (3.2) | | | |
| un retardo que no está en el modelo (3.3, 3.4) | | | |
| seguir un ángulo en lugar de una velocidad (3.5 a 3.8) | | | |

**Conclusión del ejercicio 3:** *(escribirla acá: qué hizo robusto al lazo, frente a qué no
lo es, qué cambió al cerrar el lazo sobre el ángulo, y qué decisiones de diseño tomó el
modelo y cuáles lo que al modelo le falta)*
""")

md(r"""
---
## Antes de entregar

- [ ] Cada predicción está escrita antes que su medición, con el número.
- [ ] Todo ensayo de lazo cerrado muestra la velocidad y el comando.
- [ ] Los tiempos de establecimiento dicen su banda.
- [ ] Está dicho qué actuador tiene el banco, y `vel_win` en cada ensayo.
- [ ] Las tres tablas están completas.
- [ ] Todos los gráficos tienen título, ejes con unidades y leyenda.
- [ ] **Cada ejercicio tiene su conclusión escrita.**

> Si una medición no coincide con la predicción, no se la ajusta ni se la esconde: se dice
> cuánto se aparta y qué se probó para entender por qué. En este TP esa diferencia es el tema.
""")


nb = nbf.v4.new_notebook(cells=cells, metadata={
    'kernelspec': {'display_name': 'Python 3', 'language': 'python', 'name': 'python3'},
    'language_info': {'name': 'python'},
})

salida = Path(sys.argv[1]) if len(sys.argv) > 1 else \
    Path(__file__).resolve().parents[1] / 'tp3_control_motor.ipynb'
nbf.validate(nb)
nbf.write(nb, salida)
print(f'{salida}: {len(cells)} celdas')
