# Ball Balancer 3RPS — ESP32-S3

Mesa equilibrista controlada por **PID duplo (X e Y)** com **cinemática inversa 3RPS**.
A bola é detectada por uma tela resistiva de 4 fios; três motores NEMA 17 inclinam a
plataforma para manter a bola no centro.

> **Toda a teoria de controle** (modelo da planta, função de transferência, dedução
> dos ganhos `Kp/Ki/Kd`, discretização, anti-windup) está em **[TEORIA.md](TEORIA.md)**.
> Este README cobre apenas **montagem, build e operação**.

---

## 1. Visão geral

```
            bola (tela resistiva)
                  │  x_mm, y_mm  (50 Hz)
                  ▼
     ┌──────────────────────────────┐
     │  PID_X(x_mm) → nx             │   firmware ESP32-S3
     │  PID_Y(y_mm) → ny             │   (laço de 50 Hz)
     │  cinemática 3RPS → θA,θB,θC   │
     │  steppers (perfil trapezoidal)│
     └──────────────────────────────┘
                  │  STEP/DIR
                  ▼
        3× NEMA 17 inclinam a mesa
```

O firmware roda **fechado e autônomo** no ESP32. As ferramentas do PC servem para
**ver, calibrar e ajustar ao vivo** — nada de rebuild para mudar ganhos ou calibração.

---

## 2. Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 |
| Sensor de posição | Tela resistiva 4 fios — VSDISPLAY VS084TP-A1 (8,4 pol) |
| Dimensões da tela | 187 mm × 141 mm (largura × altura) |
| Atuadores | 3× NEMA 17 + drivers STEP/DIR (arranjo 3RPS) |
| Drivers de passo | A4988 / DRV8825 / TMC2208 (interface STEP/DIR idêntica) |

### Pinos — Tela resistiva

Cabo flat com o conector voltado para a **direita** (olhando a superfície da mesa).

| Pino FPC | Sinal | GPIO | Canal ADC |
|---|---|---|---|
| 1 | Y+ | 4 | ADC1_CH3 (sense de X) |
| 2 | X- | 5 | ADC1_CH4 (sense de Y) |
| 3 | Y- | 8 | — (saída digital) |
| 4 | X+ | 9 | Só saída (ADC não funciona neste módulo) |

### Pinos — Drivers de passo

| Motor | STEP | DIR |
|---|---|---|
| A | 15 | 16 |
| B | 17 | 18 |
| C | 40 | 41 |
| ENABLE (comum, ativo LOW) | 10 | — |

### Orientação física

```
         [cabo FPC aqui]
   TL ──────────────── TR
   │                    │
   │       MESA         │
   │     (superfície)   │
   BL ──────────────── BR
```

Convenção de coordenadas após calibração:
- `x_mm = 0` → borda esquerda · `x_mm = 187` → borda direita
- `y_mm = 0` → borda superior · `y_mm = 141` → borda inferior

---

## 3. Estrutura do projeto

```
PIDcV2/
├── README.md              Este arquivo (montagem + operação)
├── TEORIA.md              Teoria de controle digital (PID, FT, ganhos)
├── CMakeLists.txt         Projeto ESP-IDF (target esp32s3)
├── sdkconfig.defaults     Defaults do SDK (sdkconfig é gerado no build)
├── main/                  Firmware C (ESP-IDF)
│   ├── main.c             Entry point + modo MAPPING/CONTROL
│   ├── control.c/.h       Laço 50 Hz + parser de comandos seriais
│   ├── pid.c/.h           PID por eixo (deriv. filtrada + anti-windup)
│   ├── kinematics.c/.h    Cinemática inversa 3RPS
│   ├── steppers.c/.h      Driver STEP/DIR (perfil trapezoidal 10 kHz)
│   ├── touch_screen.c/.h  Leitura resistiva 4 fios + calibração runtime
│   └── cal_store.c/.h     Persistência NVS (ganhos PID + calibração)
├── tools/                 Ferramentas Python (PC)
│   ├── calibrate.py/.bat  FERRAMENTA PRINCIPAL: calibração da tela, bola ao
│   │                      vivo, orientação (gravar) e ajuste de PID em tempo real
│   ├── PIDSimba.py/.bat   SIMULAÇÃO física visual (motores + bola), precisa pygame
│   ├── sim_pid.py         VALIDADOR offline do PID (headless, só stdlib)
│   ├── config.py          Porta COM, dimensões e orientação dos eixos
│   └── requirements.txt   pygame, pyserial
└── docs/
    └── referencia/        Código Arduino original do Aaed Musa (.ino)
```

---

## 4. As duas ferramentas

Há **duas** aplicações Python, com papéis distintos. Ambas leem porta/dimensões de
`tools/config.py` e dependem de `pygame` + `pyserial`
(`pip install -r tools/requirements.txt`).

> **Build & flash** não têm GUI própria — usa-se o `idf.py` direto (ver §5). É o
> jeito mais confiável de gravar o firmware. Depois da primeira gravação,
> **tudo (calibração, orientação e PID) é ajustado ao vivo pelas ferramentas
> abaixo, sem nunca mais rebuildar**.

### 🛠️ `calibrate.py` — Ferramenta principal de operação

Abra por `tools/calibrate.bat` (ou `python tools/calibrate.py COM5`).
É o app do dia a dia e cobre os quatro recursos centrais. Telas:

| Tecla / Estado | Para quê |
|---|---|
| `ENTER` na splash | Inicia a **calibração** guiada (coleta 80 amostras em cada um dos 4 cantos) |
| `L` na splash | Pula direto ao LIVE usando a calibração de `config.py` |
| Em **CAL_RESULTS** | `S` salva no header (.h); **`N` envia ao NVS do ESP32 (sem rebuild)**; `ENTER` vai ao LIVE |
| **LIVE** | **Bola ao vivo** em mm + trilha. `T`/`X`/`Y` ajustam a **orientação**; **`G` grava** (config + header) |
| **PID** (tecla `P` no LIVE) | **Ajuste de KP/KI/KD em tempo real** contra o firmware: `Q/A`,`W/S`,`E/D` mudam os ganhos, `Tab` o passo, **`ESPAÇO` salva no NVS** |
| **BALANCE** (tecla `B` no LIVE) | Simulação isométrica dos 3 motores reagindo à bola real, com os ganhos atuais |

> O modo **PID** requer o firmware em **modo CONTROL** (padrão). Ao entrar, ele
> envia `?` ao ESP para ler os ganhos em vigor (`GAINS`) e sincronizar o painel —
> a partir daí cada ajuste é aplicado na hora e `ESPAÇO` persiste no NVS.

### 🧮 `sim_pid.py` — Validador do PID offline (sem hardware, sem pygame)

Para **testar se os ganhos PID funcionam ANTES de ir ao firmware**. Usa só a
biblioteca padrão do Python (roda em qualquer máquina, sem instalar nada). Porta
fiel do algoritmo de `pid.c` contra o modelo de planta da [TEORIA.md](TEORIA.md):
roda a resposta no laço fechado e imprime overshoot, tempo de resposta,
acomodação, erro residual e um gráfico ASCII.

```
python tools/sim_pid.py                      # compara firmware x ganhos recomendados
python tools/sim_pid.py 3e-4 1.2e-5 4e-4     # testa SEUS ganhos
python tools/sim_pid.py 8e-4 2e-5 1.2e-2 --x0 70 --dist 200 --t 15
```

`--x0` deslocamento inicial (mm) · `--dist` chute de velocidade no meio (mm/s) ·
`--t` duração (s). O resultado bate com a previsão analítica (`ωn`, `ζ`) impressa
junto — é a forma rápida de descobrir bons KP/KI/KD para depois enviá-los ao ESP
pela tela PID do `calibrate.py`.

> **Modelo:** plano linearizado de eixo único (`x'' = −A·u`, `A = (5/7)g`). Valida
> a *matemática do controlador*. A dinâmica não-linear completa (cinemática 3RPS,
> motores reais) está no `PIDSimba.py`.

### 🧪 `PIDSimba.py` — Simulação física dedicada (visual)

Abra por `tools/PIDSimba.bat`. Foco em **demonstração e ensino**:

- **Modo SIM**: bola virtual rolando no plano inclinado sob gravidade real
  (rolamento 5/7, atrito, colisão com bordas); cada motor é modelado como um
  stepper de verdade (perfil trapezoidal). Mostra o laço PID inteiro **sem hardware**.
- **Modo REAL**: lê a telemetria do ESP (`POS`/`CTRL`) e **espelha os ângulos reais**
  dos 3 motores que o firmware comandou.

Útil para apresentação/relatório — ver [TEORIA.md](TEORIA.md) para os ganhos de
simulação (`SIM_KP/KI/KD`), que diferem dos do firmware por modelarem uma planta
"mais quente" com `g = 9,81 m/s²`.

```
python tools/PIDSimba.py            # usa config.py
python tools/PIDSimba.py COM5       # porta explícita
python tools/PIDSimba.py --sim      # abre direto na simulação física
```

---

## 5. Fluxo de trabalho

### Primeira configuração (hardware novo)

1. Edite a porta COM em `tools/config.py`.
2. **Grave o firmware** (uma vez) com o ESP-IDF carregado:
   ```
   idf.py build
   idf.py -p COM5 flash monitor
   ```
   Confirme no monitor: `Ball Balancer — Control Mode @ 50 Hz`. Feche o monitor
   (Ctrl+]) para liberar a porta antes de abrir o `calibrate.py`.
3. `calibrate.bat` → `ENTER` → coloque a bola nos **4 cantos** → em CAL_RESULTS
   pressione **`N`** (envia ao NVS). → `Calibracao salva no NVS`.
4. Tela **LIVE**: confirme que a bola na tela bate com a física. Se não, ajuste
   `T` / `X` / `Y` e pressione **`G`** para gravar a orientação.
5. Tecla **`P`** (PID): ajuste `KP/KI/KD` ao vivo → **`ESPAÇO`** salva no NVS.

### Boots seguintes

O firmware **carrega calibração e ganhos do NVS automaticamente**. Não precisa
recalibrar nem rebuildar — só ligar.

| Situação | Ação |
|---|---|
| Mudou posição/orientação da tela | `calibrate.py`: recalibrar → `N`, e/ou `T`/`X`/`Y` → `G` |
| Quer mexer no PID | `calibrate.py`: tecla `P` → ajustar → `ESPAÇO` |
| Mesa empurra a bola para longe | Comando serial `SX -1` ou `SY -1` |
| Apagou a flash (`idf.py erase-flash`) | Refazer calibração + PID |

---

## 6. Ajuste ao vivo (sem rebuild)

Tudo abaixo é aplicado **na hora** e pode ser persistido no NVS. É o que torna
o tuning prático — você não recompila para testar um ganho.

### Protocolo serial (UART0, 115200 baud)

**Saída do firmware (50 Hz):**

```
POS,<x_raw>,<y_raw>,<x_mm>,<y_mm>            bola detectada
NOTOUCH,<x_raw>,<y_raw>                       sem contato
CTRL,<x>,<y>,<setx>,<sety>,<nx>,<ny>,<thA>,<thB>,<thC>
GAINS,<kp>,<ki>,<kd>                          após ajuste de ganho
SIGNS,<sx>,<sy>                               sinais de controle
CAL,<xmin>,<xmax>,<ymin>,<ymax>,<fx>,<fy>,<sw>
CORNER,<TL|TR|BL|BR>,<x_raw>,<y_raw>          canto capturado
SAVED,<pid|touch>                             confirmação de NVS salvo
```

**Comandos de entrada:**

| Comando | Efeito |
|---|---|
| `PID <kp> <ki> <kd>` | Ajusta os três ganhos (ambos os eixos) |
| `KP <v>` / `KI <v>` / `KD <v>` | Ajusta um ganho por vez |
| `PID SAVE` | Persiste os ganhos atuais no NVS |
| `SX <v>` / `SY <v>` | Sinal do controle por eixo (`<0` → −1, senão +1) |
| `CAL TL\|TR\|BL\|BR` | Captura o toque atual como aquele canto |
| `CAL APPLY` | Calcula a calibração a partir dos 4 cantos |
| `CAL SAVE` / `CAL SHOW` / `CAL RESET` | Persiste / exibe / reseta calibração |
| `SETCAL <xmin> <xmax> <ymin> <ymax> <fx> <fy> <sw>` | Aplica calibração calculada no PC |
| `?` | Exibe ganhos, sinais e calibração atuais |

> As telas Calibrar e PID do `calibrate.py` apenas montam esses comandos
> para você — dá para fazer tudo na mão por qualquer monitor serial também.

---

## 7. Modos de firmware

| Macro `BB_MODE` | Comportamento |
|---|---|
| `BB_MODE_CONTROL` (padrão) | Laço PID 50 Hz + cinemática + motores |
| `BB_MODE_MAPPING` | Só lê o touch e emite telemetria — sem motores |

Compilar em modo mapeamento (útil só para depurar a tela):
`idf.py build -DBB_MODE=BB_MODE_MAPPING`

---

## 8. Parâmetros de geometria

Definidos em `main/control.c`. **São placeholders do projeto de referência —
medir no rig real e substituir.**

| Constante | Valor atual | Significado |
|---|---|---|
| `GEO_D` | 50,8 mm | Raio da base (pivôs inferiores) |
| `GEO_E` | 79,4 mm | Raio da plataforma (juntas esféricas) |
| `GEO_F` | 44,45 mm | Comprimento do braço do motor |
| `GEO_G` | 93,2 mm | Comprimento da biela |
| `GEO_HZ` | 108,0 mm | Altura neutra da plataforma |
| `MICROSTEPS` | 16 | Microstepping dos drivers (DIP switches) |

A relação dos steppers (`SPEED_BALANCE`, `ACCEL_BALANCE`, etc.) e a justificativa
do perfil trapezoidal estão em [TEORIA.md](TEORIA.md).

---

## 9. Pendências

| Item | Status |
|---|---|
| Medir geometria real (`GEO_D/E/F/G/HZ`) | Pendente |
| Confirmar `MICROSTEPS` nos DIP switches | Pendente |
| Validar homing (posição 0 = mesa nivelada) | Pendente |
| Tunar KP/KI/KD no hardware (PID Tuner) | Pendente |
| Verificar `SX`/`SY` (a mesa corrige ou piora?) | Pendente |

---

## Créditos

Cinemática inversa 3RPS e arquitetura original baseadas no projeto
[Ball Balancer do Aaed Musa](https://www.instructables.com/Ball-Balancer/).
O código Arduino original está em [docs/referencia/](docs/referencia/).
