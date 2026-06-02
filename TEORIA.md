# Teoria de Controle — Ball Balancer 3RPS

Documento de fundamentação teórica do projeto, voltado a **relatório e
apresentação**. Cobre o modelo da planta, a função de transferência, o projeto
do PID, **a dedução de cada ganho (Kp, Ki, Kd)** e a discretização (controle
digital) — tudo amarrado ao código real do firmware (`main/pid.c`,
`main/control.c`) e da simulação (`tools/PIDSimba.py`).

> Convenção de notação: variáveis físicas em mm e segundos. O eixo X e o eixo Y
> são **idênticos e independentes** (dois PIDs iguais, um por eixo), então toda a
> análise é feita para **um eixo** e replicada.

---

## 1. O problema de controle

Uma bola apoiada sobre uma superfície que pode ser inclinada. Inclinando a mesa
de um pequeno ângulo, a gravidade gera uma força que acelera a bola. O objetivo
é manter a bola numa posição de referência (o centro), apesar de perturbações
(empurrões, imperfeições mecânicas, ruído do sensor).

```
   referência r ──►(+)── e ──► [ PID ] ──► u (inclinação) ──► [ PLANTA ] ──► x (posição)
                    ▲−                                                         │
                    └─────────────────── sensor (tela resistiva) ◄────────────┘
```

- **`e = x − r`** : erro de posição (no firmware, `err = meas − setpoint`, em mm).
- **`u`** : saída do controlador = componente do **vetor normal** da mesa (`nx`/`ny`),
  que para pequenos ângulos equivale à **inclinação (slope)** do plano.
- **Planta** : dinâmica da bola rolando + cinemática + atuadores.

---

## 2. Modelo da planta (bola rolando num plano inclinado)

### 2.1. Dinâmica de Newton–Euler

Para uma **esfera maciça** que rola **sem deslizar** sobre um plano inclinado de
ângulo `α`, somam-se a 2ª lei de Newton (translação) e a equação de Euler (rotação):

$$ m\,\ddot{x} = m g \sin\alpha - F_{at}, \qquad I\,\dot\omega = F_{at}\,r, \qquad \dot{x} = \omega r $$

Para a esfera maciça, o momento de inércia é `I = (2/5) m r²`. Eliminando a força
de atrito `F_at` e usando `ẍ = r ω̇`:

$$ \left(m + \frac{I}{r^2}\right)\ddot{x} = m g \sin\alpha
   \;\Longrightarrow\; \frac{7}{5}\,\ddot{x} = g\sin\alpha $$

$$ \boxed{\;\ddot{x} = \frac{5}{7}\, g \sin\alpha \;\approx\; \frac{5}{7}\, g\,\alpha\;} $$

A fração **5/7 ≈ 0,714** é exatamente a constante `ROLL_FACTOR = 5.0/7.0` usada
na simulação física (`tools/PIDSimba.py`). A aproximação `sin α ≈ α` vale para os
pequenos ângulos de operação (a saída é limitada a `|u| ≤ 0,25 rad ≈ 14°`).

### 2.2. Relação entre a saída do controlador e o ângulo

A saída do PID, `u = nx`, é a componente x do vetor normal unitário da mesa.
Para pequenas inclinações, **`nx ≈ tan α ≈ α`** (a inclinação do plano). Logo,
usando `g = 9810 mm/s²`:

$$ \ddot{x} \;=\; \underbrace{\tfrac{5}{7}\,g}_{A}\; u,
   \qquad A = \tfrac{5}{7}\times 9810 \approx 7007\ \text{mm/s}^2 $$

### 2.3. Função de transferência da planta

Aplicando Laplace com condições iniciais nulas a `ẍ = A·u`:

$$ \boxed{\;P(s) = \frac{X(s)}{U(s)} = \frac{A}{s^2},\qquad A \approx 7007\ \text{mm/s}^2\;} $$

A planta é um **duplo integrador**. Implicações de projeto:

- **Tipo 2** ⇒ erro estacionário **nulo** a degrau e a rampa de referência.
- Os dois polos estão **na origem** (`s = 0`): a planta é **marginalmente estável**.
  Realimentação puramente proporcional coloca os polos de malha fechada sobre o
  eixo imaginário → **oscilação não amortecida**. É por isso que o termo
  **derivativo é obrigatório** (ele injeta a fase/amortecimento que falta).

---

## 3. O controlador PID

### 3.1. Forma contínua

$$ u(t) = K_p\,e(t) + K_i\!\int_0^t\! e(\tau)\,d\tau + K_d\,\dot{e}(t) $$

$$ C(s) = K_p + \frac{K_i}{s} + K_d\,s $$

No firmware (`main/pid.c`) o derivativo é **filtrado** por um passa-baixa de 1ª
ordem (constante `d_tau = 0,03 s`) para não amplificar o ruído do sensor:

$$ C(s) = K_p + \frac{K_i}{s} + \frac{K_d\,s}{1 + \tau_d\,s},\qquad \tau_d = 0{,}03\ \text{s} $$

### 3.2. Papel de cada termo nesta planta

| Termo | Função aqui | Consequência de exagerar |
|---|---|---|
| **P** (`Kp`) | Define a rigidez/velocidade da resposta (`ωn`). Sozinho → oscila. | Oscilação crescente |
| **D** (`Kd`) | **Essencial**: adiciona amortecimento (freia pela velocidade da bola). | Lentidão, sensível a ruído |
| **I** (`Ki`) | Remove desvio residual por assimetria mecânica / mesa fora de nível. | Windup, instabilidade lenta |

---

## 4. Malha fechada e mapeamento para 2ª ordem

Ignorando temporariamente o filtro do derivativo e o integral (`Ki` é pequeno —
ver §6.3), a malha fechada com `P(s) = A/s²` e o trecho **PD** dá:

$$ \frac{X(s)}{R(s)} = \frac{C(s)P(s)}{1 + C(s)P(s)}
   \;\Rightarrow\; \text{equação característica: } \; s^2 + A K_d\,s + A K_p = 0 $$

Comparando com a forma canônica de 2ª ordem `s² + 2ζωₙs + ωₙ²`:

$$ \boxed{\;\omega_n = \sqrt{A\,K_p}\;}\qquad
   \boxed{\;\zeta = \frac{K_d}{2}\sqrt{\frac{A}{K_p}}\;} $$

- `ωn` (frequência natural) ⇒ **quão rápido** a bola converge.
- `ζ` (amortecimento) ⇒ **overshoot**: `ζ<1` oscila, `ζ=1` crítico (sem overshoot),
  `ζ>1` sobreamortecido (lento).
- Tempo de acomodação (2 %): `t_s ≈ 4 / (ζ ωn)`.

---

## 5. Como chegamos em cada ganho (dedução)

A estratégia é **inverter** as fórmulas acima: escolher o comportamento desejado
(`ωn`, `ζ`) e calcular os ganhos. Usamos `A = 7007 mm/s²`.

### 5.1. `Kp` — a partir da velocidade de resposta desejada

Queremos a bola assentando em ~3 s sem brusquidão ⇒ `ωn ≈ 1,5 rad/s`. De `ωn² = A Kp`:

$$ K_p = \frac{\omega_n^2}{A} = \frac{1{,}5^2}{7007} \approx 3{,}2\times10^{-4} $$

→ valor adotado na simulação: **`SIM_KP = 3,0×10⁻⁴`**.

### 5.2. `Kd` — a partir do amortecimento desejado (crítico, sem overshoot)

Para `ζ = 1` (resposta crítica, sem ultrapassagem), isolando `Kd`:

$$ K_d = 2\sqrt{\frac{K_p}{A}} = 2\sqrt{\frac{3{,}0\times10^{-4}}{7007}} \approx 4{,}1\times10^{-4} $$

→ valor adotado na simulação: **`SIM_KD = 4,0×10⁻⁴`**.

### 5.3. `Ki` — a partir do tempo integral (correção lenta de viés)

O integral só precisa eliminar **offset lento** (mesa levemente desnivelada). Usamos
um **tempo integral longo** `Ti = Kp/Ki` para não interferir na dinâmica rápida:

$$ T_i = \frac{K_p}{K_i} = \frac{3{,}0\times10^{-4}}{1{,}2\times10^{-5}} = 25\ \text{s}
   \quad\Rightarrow\quad K_i = 1{,}2\times10^{-5} $$

→ valor adotado na simulação: **`SIM_KI = 1,2×10⁻⁵`**.

### 5.4. Validação (a teoria fecha com o código)

Substituindo `SIM_KP, SIM_KD` nas fórmulas de §4:

$$ \omega_n=\sqrt{7007\cdot3{,}0\!\times\!10^{-4}}\approx1{,}45\ \text{rad/s},\qquad
   \zeta=\frac{4{,}0\!\times\!10^{-4}}{2}\sqrt{\frac{7007}{3{,}0\!\times\!10^{-4}}}\approx0{,}97 $$

$$ t_s \approx \frac{4}{\zeta\,\omega_n} \approx \frac{4}{0{,}97\cdot1{,}45} \approx 2{,}8\ \text{s} $$

Isso reproduz **exatamente** o comportamento documentado no `PIDSimba.py`
("amortecimento ~crítico, sem overshoot, assenta ~2–3 s"). **A teoria e a
implementação são consistentes.**

### 5.5. Os ganhos *padrão do firmware* são diferentes — e por quê

`main/control.c` parte de **`KP=8×10⁻⁴`, `KI=2×10⁻⁵`, `KD=1,2×10⁻²`**. Pela
análise linear isso dá `ζ ≈ 17,8` (fortemente sobreamortecido). Não é um erro: é
uma escolha **conservadora e robusta** para o *hardware real*, por dois motivos:

1. **Saturação proposital do derivativo**: com a saída limitada a `±0,25`, o termo
   D sozinho satura quando `Kd·v ≥ 0,25`, isto é, a partir de `v ≈ 0,25/0,012 ≈ 21 mm/s`.
   Acima disso o D atua como **freio de velocidade quase bang-bang** — muito eficaz
   contra empurrões e imune a pequenas variações de ganho. (Esse `~21 mm/s` está
   anotado no comentário de `pid.c`.)
2. **Ponto de partida seguro**: o firmware é tunado **no rig** com o PID Tuner ao
   vivo (subir `Kp` até oscilar, adicionar `Kd`, depois um toque de `Ki`). Os
   defaults precisam ser mansos para não derrubar a bola na primeira energização.

> Resumo: os ganhos de **simulação** (§5.1–5.3) são projetados analiticamente
> contra o modelo ideal; os de **firmware** são conservadores e refinados
> experimentalmente. As duas escolhas vêm da mesma teoria.

---

## 6. Controle digital (discretização)

O laço roda a **`50 Hz`** (`LOOP_HZ = 50`), logo `T = 0,02 s`. A frequência de
Nyquist é 25 Hz ≫ `ωn/2π ≈ 0,23 Hz`, então a amostragem é folgada e a aproximação
contínua de §4–5 é válida.

### 6.1. Integral — Euler para a frente

$$ I[k] = I[k-1] + e[k]\,T \quad\Longleftrightarrow\quad s \to \frac{z-1}{T} $$

No código: `p->integ += err * dt;`

### 6.2. Derivada — diferença regressiva + filtro passa-baixa

Derivada bruta por diferença regressiva, depois suavizada por um IIR de 1ª ordem:

$$ d[k] = \frac{e[k]-e[k-1]}{T}, \qquad
   d_{lpf}[k] = d_{lpf}[k-1] + \alpha\big(d[k]-d_{lpf}[k-1]\big) $$

com o coeficiente derivado de `τd`:

$$ \alpha = \frac{T}{\tau_d + T} = \frac{0{,}02}{0{,}03+0{,}02} = 0{,}4 $$

No código (`pid.c`): `float a = dt/(d_tau+dt); d_lpf += a*(deriv − d_lpf);`.
O filtro corta ruído acima de `1/(2πτd) ≈ 5,3 Hz`, preservando a dinâmica da bola.

### 6.3. Anti-windup por *back-calculation*

Quando a saída satura no limite `±0,25`, o integrador é **puxado de volta** pela
quantidade exata que excedeu, evitando acúmulo (windup) que causaria overshoot
gigante ao sair da saturação:

$$ \text{se } u>u_{max}:\quad I \mathrel{-}= \frac{u-u_{max}}{K_i},\quad u=u_{max} $$

No código (`pid.c`): `if (out>out_max){ p->integ -= (out-out_max)/p->ki; out=out_max; }`.

### 6.4. Lei de controle discreta completa

$$ \boxed{\;u[k] = K_p\,e[k] + K_i\,I[k] + K_d\,d_{lpf}[k]\;}
   \quad\text{seguida de saturação + anti-windup} $$

A cada borda de "sem toque", o controlador é **resetado** (`pid_reset`) para não
arrastar estado integral/derivativo de uma sessão para a próxima.

---

## 7. Da inclinação aos motores — cinemática inversa 3RPS

O PID produz um **vetor de inclinação** `(nx, ny)`; a mesa é movida por 3 motores
em arranjo **3RPS** (3× Revolute-Prismatic-Spherical). A função
`machine_theta(perna, hz, nx, ny)` (`main/kinematics.c`, porte fiel do projeto do
Aaed Musa) resolve, em forma fechada, o **ângulo de cada motor** que realiza aquela
inclinação:

$$ (n_x, n_y) \;\xrightarrow{\text{cinemática inversa}}\; (\theta_A,\theta_B,\theta_C) $$

Do ponto de vista de controle, essa cinemática é um **mapa estático não-linear**
(sem dinâmica própria significativa): ela não altera a *ordem* do laço, apenas
traduz o comando de inclinação em posições angulares. Por isso a análise linear
de §4 (feita no domínio da inclinação) permanece válida em 1ª ordem.

A conversão final para passos:

$$ \text{passos} = (\theta_{orig} - \theta)\cdot\frac{200\cdot\text{microsteps}}{360°},
   \qquad \theta_{orig} = \texttt{machine\_theta}(\cdot,\ n_x{=}0,\ n_y{=}0) $$

---

## 8. O atuador — motores de passo com perfil trapezoidal

Cada motor segue a posição-alvo com um **perfil trapezoidal de velocidade**
(`main/steppers.c`, timer de 10 kHz): acelera até `max_speed` e desacelera a tempo
de parar **exatamente** no alvo. A velocidade máxima da qual ainda se consegue
frear a tempo é:

$$ v_{lim} = \sqrt{2\,a\,d} $$

onde `d` é a distância restante (passos) e `a` a aceleração. Sem essa rampa, uma
partida brusca causaria **perda de passos** (o motor "escorrega"). Em equilíbrio
usa-se `1500 passos/s` e `20000 passos/s²`; no homing, valores menores.

O atuador adiciona um pequeno atraso de fase, mas sua banda (centenas de Hz) é
muito superior à dinâmica da bola (~0,23 Hz), então não compromete a estabilidade
projetada.

---

## 9. Estabilidade e margens

- **Planta `A/s²`** isolada: marginalmente estável (2 polos na origem).
- **Com PD**: equação característica `s² + A Kd s + A Kp = 0`. Pelo critério de
  Routh–Hurwitz, todos os coeficientes são positivos ⇒ **malha estável** para
  quaisquer `Kp, Kd > 0`. O derivativo é o que garante a estabilidade.
- **Com o I** (pequeno): adiciona um polo lento em malha aberta; mantido com `Ti`
  longo (25–40 s) para não reduzir a margem de fase de forma relevante.
- **Amostragem**: `T = 0,02 s` introduz atraso ≈ `T/2 = 0,01 s`, desprezível
  frente a `1/ωn ≈ 0,7 s`.

---

## 10. Procedimento prático de sintonia (no hardware)

Feito ao vivo pela tela **PID** do `calibrate.py` (sem rebuild — comandos
`PID`/`PID SAVE` pela serial):

1. Zere `Ki` e `Kd`. Suba `Kp` até a bola **oscilar** de forma sustentada em torno
   do centro (ganho crítico `Ku`, período `Tu`).
2. Adicione `Kd` para **amortecer** a oscilação (alvo: sem overshoot ⇒ `ζ ≈ 1`).
3. Adicione um **toque** de `Ki` só para eliminar o desvio residual (mesa torta).
4. `PID SAVE` (ESPAÇO) grava no **NVS** — persiste entre reinicializações.

Como ponto de partida quantitativo, Ziegler–Nichols clássico sugere
`Kp ≈ 0,6 Ku`, `Ti ≈ 0,5 Tu`, `Td ≈ 0,125 Tu` — útil como chute inicial antes do
ajuste fino acima.

---

## 11. Tabela-resumo dos parâmetros

| Símbolo | Onde | Valor | Origem |
|---|---|---|---|
| `A = (5/7)g` | planta | ≈ 7007 mm/s² | Newton–Euler (esfera maciça) |
| `T` | laço | 0,02 s (50 Hz) | `LOOP_HZ` |
| `τd` | filtro D | 0,03 s | `PID_D_TAU` |
| `α` | filtro D discreto | 0,4 | `T/(τd+T)` |
| `SIM_KP` | simulação | 3,0×10⁻⁴ | `ωn ≈ 1,5 rad/s` (§5.1) |
| `SIM_KD` | simulação | 4,0×10⁻⁴ | `ζ = 1` crítico (§5.2) |
| `SIM_KI` | simulação | 1,2×10⁻⁵ | `Ti = 25 s` (§5.3) |
| `KP` | firmware | 8,0×10⁻⁴ | conservador, tunado no rig (§5.5) |
| `KI` | firmware | 2,0×10⁻⁵ | `Ti = 40 s` |
| `KD` | firmware | 1,2×10⁻² | freio com saturação proposital (§5.5) |
| `u_max` | saturação | ±0,25 | `TILT_LIMIT` |
| `ωn` (sim) | resposta | ≈ 1,45 rad/s | derivado |
| `ζ` (sim) | resposta | ≈ 0,97 | derivado (≈ crítico) |
| `t_s` (sim) | resposta | ≈ 2,8 s | `4/(ζωn)` |

---

## 12. Referências

- Aaed Musa, *Ball Balancer* — Instructables: <https://www.instructables.com/Ball-Balancer/>
  (cinemática 3RPS e arquitetura original; código em [docs/referencia/](docs/referencia/)).
- Ogata, *Engenharia de Controle Moderno* — 2ª ordem, Routh, discretização.
- Franklin, Powell, Workman, *Digital Control of Dynamic Systems* — Euler/Tustin,
  projeto no domínio `z`.
