#!/usr/bin/env python3
"""
Ball Balancer — Simulador HEADLESS do laco PID (sem hardware, sem pygame)
=========================================================================
Valida NUMERICAMENTE se os ganhos PID estabilizam a bola, ANTES de gravar no
firmware. Usa apenas a biblioteca padrao do Python — roda em qualquer lugar.

  * O controlador e um porte FIEL de main/pid.c:
      - erro = medida - setpoint
      - derivada no erro com filtro passa-baixa de 1a ordem (d_tau = 0.03 s)
      - anti-windup por back-calculation
      - saturacao da saida em +-TILT_LIMIT
  * A planta e o modelo linearizado deduzido em TEORIA.md (secao 2):
      x'' = -A * u,   A = (5/7)*g = 7007 mm/s^2
    (u = inclinacao comandada pelo PID; eixo unico — os eixos X e Y sao
     independentes e identicos, entao 1 eixo valida a matematica dos dois.)

Uso:
  python sim_pid.py                         # ganhos do FIRMWARE (control.c)
  python sim_pid.py 3e-4 1.2e-5 4e-4        # ganhos da SIMULACAO (PIDSimba)
  python sim_pid.py 8e-4 2e-5 1.2e-2 --x0 70 --dist 25 --t 12

Argumentos:
  kp ki kd        ganhos a testar (default = firmware: 8e-4 2e-5 1.2e-2)
  --x0 <mm>       deslocamento inicial da bola em relacao ao centro (default 70)
  --dist <mm/s>   "chute" de velocidade aplicado em t=t/2 (perturbacao, default 0)
  --t <s>         duracao da simulacao (default 10)
  --hz <Hz>       frequencia do laco (default 50, igual ao firmware)
"""

import sys
import math

# ── Constantes (espelham o firmware / TEORIA.md) ─────────────────────────────
G_MM        = 9810.0            # gravidade (mm/s^2)
ROLL_FACTOR = 5.0 / 7.0         # esfera macica rolando sem deslizar
A_PLANT     = ROLL_FACTOR * G_MM  # ganho da planta (mm/s^2 por unidade de slope)
TILT_LIMIT  = 0.25              # clamp da saida (= TILT_LIMIT no control.c)
PID_D_TAU   = 0.03              # constante do filtro derivativo (= pid.c)

# Ganhos padrao do FIRMWARE (main/control.c)
FW_KP, FW_KI, FW_KD = 8.0e-4, 2.0e-5, 1.2e-2


class PID:
    """Porte fiel de main/pid.c (derivada filtrada + anti-windup)."""
    def __init__(self, kp, ki, kd, out_min, out_max, d_tau=PID_D_TAU):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.d_tau = d_tau
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.prev_err = 0.0
        self.primed = False
        self.d_lpf = 0.0

    def update(self, meas, setpoint, dt):
        if dt <= 0.0:
            dt = 1e-3
        err = meas - setpoint

        deriv = 0.0
        if self.primed:
            deriv = (err - self.prev_err) / dt
            if self.d_tau > 0.0:
                a = dt / (self.d_tau + dt)
                self.d_lpf += a * (deriv - self.d_lpf)
                deriv = self.d_lpf
        self.prev_err = err
        self.primed = True

        # anti-windup por INTEGRACAO CONDICIONAL (porte de main/pid.c, Fase 1):
        # so integra quando nao agrava a saturacao corrente (independe de Ki).
        out = self.kp * err + self.ki * (self.integ + err * dt) + self.kd * deriv
        sat_hi = out > self.out_max
        sat_lo = out < self.out_min
        freeze = (sat_hi and err > 0.0) or (sat_lo and err < 0.0)
        if not freeze:
            self.integ += err * dt

        out = self.kp * err + self.ki * self.integ + self.kd * deriv
        return max(self.out_min, min(self.out_max, out))


def simulate(kp, ki, kd, x0_mm, dist_mm_s, t_total, hz):
    """Laco fechado: PID (porte do firmware) + planta (modelo linear da TEORIA).
    Retorna (tempos, posicoes_erro, saidas_u)."""
    dt = 1.0 / hz
    pid = PID(kp, ki, kd, -TILT_LIMIT, TILT_LIMIT)

    # estado: posicao relativa ao centro (setpoint = 0) e velocidade
    x = float(x0_mm)
    v = 0.0
    dist_t = t_total / 2.0  # instante da perturbacao

    ts, xs, us = [], [], []
    n = int(t_total * hz)
    for k in range(n):
        t = k * dt
        # perturbacao impulsiva (chute de velocidade)
        if dist_mm_s and abs(t - dist_t) < dt / 2.0:
            v += dist_mm_s

        u = pid.update(x, 0.0, dt)   # setpoint = 0 (centro)
        acc = -A_PLANT * u           # planta: x'' = -A*u  (ver TEORIA.md sec.4)
        v += acc * dt                # integracao semi-implicita (Euler)
        x += v * dt

        ts.append(t); xs.append(x); us.append(u)
    return ts, xs, us


def metrics(ts, xs, x0, dt):
    """Overshoot, tempo de acomodacao (2%), tempo de resposta (5 mm) e residual."""
    final = xs[-1]
    # overshoot: maior excursao para o lado oposto do deslocamento inicial
    if x0 >= 0:
        peak = min(xs)                      # quanto passou abaixo de 0
        overshoot = max(0.0, -peak) / abs(x0) * 100.0 if x0 else 0.0
    else:
        peak = max(xs)
        overshoot = max(0.0, peak) / abs(x0) * 100.0 if x0 else 0.0

    def last_outside(band):
        for i in range(len(xs) - 1, -1, -1):
            if abs(xs[i]) > band:
                return ts[i] + dt
        return 0.0

    tol = max(0.02 * abs(x0), 0.5)          # banda de 2% (minimo 0.5 mm)
    settle = last_outside(tol)              # tempo de acomodacao (2%)
    t_resp = last_outside(5.0)              # tempo para entrar/ficar em +-5 mm
    return overshoot, settle, t_resp, abs(final), tol


def ascii_plot(ts, xs, x0, width=64, height=16):
    """Grafico ASCII de posicao (erro ao centro) vs tempo."""
    lo = min(min(xs), 0.0)
    hi = max(max(xs), 0.0, x0)
    span = (hi - lo) or 1.0
    # amostra colunas
    grid = [[' '] * width for _ in range(height)]
    zero_row = int((hi - 0.0) / span * (height - 1))
    for c in range(width):
        idx = int(c / (width - 1) * (len(xs) - 1))
        val = xs[idx]
        r = int((hi - val) / span * (height - 1))
        r = max(0, min(height - 1, r))
        grid[r][c] = '*'
    # linha do zero (setpoint)
    for c in range(width):
        if grid[zero_row][c] == ' ':
            grid[zero_row][c] = '-'
    out = []
    for r, row in enumerate(grid):
        val = hi - r / (height - 1) * span
        out.append(f"{val:7.1f} |" + ''.join(row))
    out.append(' ' * 8 + '+' + '-' * width)
    out.append(' ' * 9 + f"0{' ' * (width - 8)}{ts[-1]:.1f}s")
    return '\n'.join(out)


def run_case(name, kp, ki, kd, x0, dist, t_total, hz):
    dt = 1.0 / hz
    ts, xs, us = simulate(kp, ki, kd, x0, dist, t_total, hz)
    ov, settle, t_resp, sserr, tol = metrics(ts, xs, x0, dt)
    umax = max(abs(min(us)), abs(max(us)))
    sat = "SIM (controle saturou em +-0.25)" if umax >= TILT_LIMIT - 1e-6 else "nao"

    # previsao analitica (TEORIA.md sec.4)
    wn = math.sqrt(A_PLANT * kp)
    zeta = (kd / 2.0) * math.sqrt(A_PLANT / kp) if kp > 0 else float('inf')

    print("=" * 72)
    print(f"  {name}")
    print(f"  KP={kp:.3e}  KI={ki:.3e}  KD={kd:.3e}")
    print("-" * 72)
    print(f"  Planta:  x'' = -A*u   com A = (5/7)g = {A_PLANT:.0f} mm/s^2")
    print(f"  Previsao linear (PD): wn = {wn:.2f} rad/s   zeta = {zeta:.2f}", end="")
    if zeta < 1:    print("  (subamortecido -> overshoot)")
    elif zeta < 1.3: print("  (~critico -> ideal)")
    else:            print("  (sobreamortecido -> lento)")
    print(f"  Deslocamento inicial: {x0:.0f} mm" +
          (f"   + perturbacao {dist:.0f} mm/s em t={t_total/2:.1f}s" if dist else ""))
    print("-" * 72)
    resid_pct = sserr / abs(x0) * 100.0 if x0 else 0.0
    print(f"  Overshoot:            {ov:6.1f} %")
    print(f"  Tempo de resposta:    {t_resp:6.2f} s   (entra e fica em +-5 mm do centro)")
    print(f"  Tempo de acomodacao:  {settle:6.2f} s   (banda +-{tol:.1f} mm)")
    print(f"  Erro residual final:  {sserr:6.2f} mm   ({resid_pct:.0f}% do inicial)")
    print(f"  Saturacao do tilt:    {sat}  (|u|max = {umax:.3f})")
    # estavel = convergiu para perto do centro sem divergir
    estavel = sserr < 0.08 * abs(x0) and abs(xs[-1]) < abs(x0)
    if estavel:
        verdict = "ESTAVEL — a bola converge ao centro"
    elif sserr < abs(x0):
        verdict = "LENTO — converge, mas nao assenta nesta janela (suba KP / reduza KD)"
    else:
        verdict = "INSTAVEL — nao converge"
    print(f"  >>> {verdict}")
    print()
    print(ascii_plot(ts, xs, x0))
    print()
    return estavel


def main():
    args = [a for a in sys.argv[1:]]
    # ganhos posicionais (ate 3 floats no inicio)
    gains = []
    rest = []
    for a in args:
        if len(gains) < 3 and not a.startswith('--'):
            try:
                gains.append(float(a)); continue
            except ValueError:
                pass
        rest.append(a)

    def opt(flag, default, cast=float):
        if flag in rest:
            i = rest.index(flag)
            if i + 1 < len(rest):
                return cast(rest[i + 1])
        return default

    x0   = opt('--x0', 70.0)
    dist = opt('--dist', 0.0)
    t    = opt('--t', 15.0)
    hz   = opt('--hz', 50.0)

    if len(gains) == 3:
        kp, ki, kd = gains
        run_case("GANHOS FORNECIDOS", kp, ki, kd, x0, dist, t, hz)
    else:
        # comparacao didatica: firmware vs simulacao recomendada
        print("Nenhum ganho informado — comparando FIRMWARE x SIM recomendada.")
        print("(passe 'kp ki kd' para testar os seus.)\n")
        run_case("FIRMWARE (control.c) — conservador",
                 FW_KP, FW_KI, FW_KD, x0, dist, t, hz)
        run_case("SIM recomendada (TEORIA.md sec.5) — zeta~1",
                 3.0e-4, 1.2e-5, 4.0e-4, x0, dist, t, hz)


if __name__ == '__main__':
    main()
