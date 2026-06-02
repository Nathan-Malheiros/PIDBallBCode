# Ball Balancer 3RPS — Análise de Arquitetura e Plano de Fases

> Revisão criteriosa da arquitetura atual (firmware ESP32-S3 + ferramentas Python),
> com foco em identificar **falhas conceituais**, fazer **auditoria por módulo** e
> propor um **plano de implementação incremental em 6 fases**.
> Nenhuma funcionalidade existente é quebrada; comandos e blobs NVS evoluem de forma
> retrocompatível.

## Acompanhamento das fases

| Fase | Tema | Status |
|---|---|---|
| 1 | Correções críticas | ✅ **Concluída** |
| 2 | Calibração, orientação e referencial | ⬜ Pendente |
| 3 | PID tuning em tempo real | ⬜ Pendente |
| 4 | Melhorias da cinemática | ⬜ Pendente |
| 5 | Diagnóstico avançado e robustez | ⬜ Pendente |
| 6 | Perfis de configuração + limpeza | ⬜ Pendente |

---

## Parte 0 — Veredito geral

O projeto é **bem-arquitetado para o estágio em que está**: separação limpa em módulos
(`pid`/`kinematics`/`steppers`/`touch_screen`/`cal_store`/`control`), perfil trapezoidal
próprio, anti-windup, filtro derivativo, persistência NVS e três ferramentas Python
coerentes com a teoria. A `TEORIA.md` é honesta e tecnicamente correta no modelo linear
(planta `A/s²`, dedução de `ωn`/`ζ`).

Mas há **três falhas conceituais estruturais** que bloqueiam exatamente os objetivos
desejados (configuração completa, comissionamento, robustez). Elas não são bugs de
digitação — são premissas de projeto que precisam ser revistas antes de qualquer código.

---

## Parte 1 — Falhas conceituais (explicadas antes de qualquer mudança)

### ❶ O referencial do **sensor** e o referencial do **atuador** estão desacoplados

Esta é a falha mais importante e a raiz da maioria dos objetivos.

- A orientação (`SWAP_XY/FLIP_X/FLIP_Y`) é aplicada **só ao sensor**, em
  `touch_screen.c:191-194`. Ela decide o que `x_mm`/`y_mm` significam.
- A cinemática `machine_theta(nx, ny)` opera num **referencial fixo da plataforma**,
  definido pela posição física das 3 pernas (`tools/calibrate.py:125`,
  `LEG_AZ = [90°, 210°, 330°]` — perna A em +Y).
- A **única** ponte entre os dois é `s_sign_x`/`s_sign_y` em `control.c:407-408`: apenas
  inverte sinais por eixo.

**Por que isso é um problema conceitual:** dois sinais (`±1, ±1`) só cobrem 4 das 8
combinações eixo-espelho, e **nenhuma rotação arbitrária**. Se a tela for montada girada
em relação às pernas, não existe combinação de `SX/SY` que faça a mesa empurrar a bola na
direção certa em ambos os eixos simultaneamente — você acerta um eixo e o outro fica
cruzado (a correção de X induz movimento em Y). Pior: girar o sensor em 90° (`SWAP_XY`)
"resolve" mudando o que o PID enxerga, mas isso **gira o referencial de controle sem girar
o de atuação** — eles passam a divergir silenciosamente.

Como as 3 pernas estão a 120° e a tela é retangular, **não existe alinhamento perfeito**
entre os dois referenciais por construção. Para pequenos ângulos o mapa `(nx,ny)→tilt` é
quase ortogonal, então o sistema "funciona o suficiente", mas o **acoplamento entre eixos**
(objetivo #3) nasce exatamente daqui, não do PID.

**Conclusão:** o objetivo #1 (rotação 0/90/180/270, swap, sentido positivo de inclinação
por eixo, qual motor é A/B/C, inversão por motor) exige uma **matriz de transformação
explícita entre o frame do sensor e o frame da cinemática**, persistida em NVS — não dá
para emular com dois sinais. Isto precisa existir antes da "ferramenta de comissionamento".

### ❷ Não existe homing — "passo 0 = mesa nivelada" é uma suposição não verificada

Em `control.c:358-367` o firmware energiza os motores, assume `current = 0`
(de `steppers.c:121`) e move para o tilt zero. **Não há limit switch, sensor de nível, nem
referência mecânica.** Se no boot os braços não estiverem fisicamente na posição que
corresponde ao passo 0, todo o referencial de inclinação está deslocado — a mesa parte
torta e o PID luta contra um viés constante (que o integral tenta corrigir, mas só dentro
de `±0,25`).

Isto está na lista de pendências ("Validar homing"), mas é **conceitual**: o objetivo #1
("definição da posição neutra / mesa nivelada") **é** o problema de homing. Sem uma forma
de estabelecer a referência mecânica (limit switch, batente, ou offset salvo + procedimento
manual de "mova até nivelar e grave"), "posição neutra configurável" não tem como ser
confiável entre reinicializações.

### ❸ Múltiplas fontes de verdade para calibração e ganhos

Hoje os mesmos números vivem em **quatro** lugares que divergem:

| Dado | Lugares | Estado real |
|---|---|---|
| Calibração da tela | `touch_screen.h` (`665…2981`), `config.py` (`499…3081`), NVS, cantos coletados | **já divergentes** entre `.h` e `config.py` |
| Ganhos PID | `control.c` (`#define KP…`), `config.py`/`calibrate.py` (`8e-4…`), NVS | NVS tem prioridade em runtime |
| Orientação | `touch_screen.h`, `config.py` | gravados juntos pela tecla `G`, mas separados do NVS |

O NVS é a verdade em runtime (`control.c:331-354`), mas as ferramentas leem de `config.py`
(referência estática). Resultado: o que a `calibrate.py` desenha pode não ser o que o
firmware está usando. Há ainda **dois algoritmos de calibração diferentes**: o firmware faz
média por aresta + auto-detecção de swap (`control.c:141-192`), enquanto a `calibrate.py`
faz simples min/max + swap manual e envia via `SETCAL`, **ignorando** o caminho `CAL APPLY`
(`calibrate.py:1270-1306`). O README diz que a ferramenta "apenas monta esses comandos" —
não é verdade, ela tem lógica própria divergente.

**Conclusão:** antes de construir "perfis" (objetivo #7) e "comissionamento" (objetivo #2),
é preciso eleger **o NVS como fonte única** e fazer as ferramentas *lerem do firmware*
(via `?`/`CAL SHOW`/telemetria), não de `config.py`.

---

## Parte 2 — Auditoria por módulo

### `pid.c` / `pid.h` — sólido, com um acoplamento perigoso

O núcleo está correto e fiel à TEORIA (derivada filtrada de 1ª ordem + integração Euler +
clamp). Pontos:

- **🔴 Anti-windup por back-calculation acoplado a um `Ki` minúsculo é instável na prática.**
  Em `pid.c:65-71`, quando a saída satura, faz `integ -= (out-out_max)/ki`. Com `Ki = 2e-5`
  e `Kd = 1,2e-2`, o termo D **satura sozinho** a partir de ~21 mm/s (a própria TEORIA §5.5
  admite isso). A cada saturação por velocidade, o excesso (causado por P+D, não pelo I) é
  descontado do integrador dividido por `Ki`: um excesso de 0,1 vira `integ -= 5000`. Ou
  seja, **todo empurrão rápido chuta o integrador para milhares** e ele leva muito tempo
  para desfazer, deixando um viés residual e recuperação lenta. A back-calculation está
  descontando do integrador a saturação que **não foi causada por ele**.
  - *Correção recomendada:* trocar por **integração condicional** (não integrar quando a
    saída está saturada *e* o erro empurra mais para dentro da saturação), ou usar uma
    constante de tracking `Tt` independente de `Ki`
    (`integ += (sat(out)-out)/Tt * dt`). Isso desacopla o anti-windup do valor de `Ki`.
- **🟡 Reset duro na perda de toque.** Quem chama `pid_reset` é o `control.c`
  (`control.c:409-413`) a cada borda de "sem toque". Resistiva com bola rolando dá
  *dropouts* intermitentes → cada um zera integral+derivada e a mesa salta para nível.
  Precisa de *hold*/debounce (a `calibrate.py` já faz isso com `PRESENCE_HOLD=0.35` em
  `calibrate.py:123` — o firmware deveria ter o mesmo).
- **🟡 Limites não configuráveis em runtime.** `out_min/out_max` (= `±TILT_LIMIT`) e `d_tau`
  são fixos em compilação. O objetivo #4 pede ajustar "limites de saída/integração/filtros"
  ao vivo — hoje exige rebuild. Faltam comandos seriais e campos no blob NVS para eles.
- **🟢 Sem unwrap de NaN:** já há guarda `isnan/isinf` no derivativo. Bom.

### `control.c` — o "cérebro", mas concentra responsabilidades demais

- **🔴 `printf` concorrente em duas tasks.** O loop de 50 Hz imprime `POS`/`CTRL`
  (`control.c:418-427`) enquanto a `cmd_task` imprime `GAINS`/`SAVED`/`CAL`
  (`control.c:102-120`, chamadas de dentro de `cmd_handle`). Sem mutex, linhas podem
  **interleavar** e corromper o que o parser Python lê. Some-se a isso
  `setvbuf(stdout, _IONBF)` em `main.c:141`: cada `printf` é uma escrita bloqueante; se o
  buffer de TX da UART encher (o driver é instalado com **tx_buffer=0**, `control.c:380`),
  o `printf` **bloqueia o loop de controle** → jitter. Ao expandir a telemetria (objetivo
  #5) isso piora. *Correção:* serializar a saída (fila/mutex) + buffer de TX real.
- **🟡 Acesso concorrente não protegido a `s_pid_*` e à calibração.** `cmd_task` escreve
  `kp/ki/kd` e chama `touch_set_cal` (7 campos) enquanto o loop lê. Escritas de `float`
  único são atômicas no S3, mas a calibração é multi-campo: um `SETCAL` no meio de um
  `touch_read` pode misturar `min` novo com `max` velho → mm transitório errado. Baixa
  probabilidade, mas é sujeira de concorrência. Um pequeno mutex resolve.
- **🟡 Mapeamento controle→atuador rígido.** `control.c:308-318` (`apply_tilt`) assume todos
  os motores com mesma direção e mesmo `s_ang_orig` escalar. Não há inversão por motor, nem
  reassinalamento A/B/C, nem offset por motor (objetivo #1).
- **🟡 Geometria e constantes hardcoded em `#define`** (`control.c:24-59`): `GEO_*`,
  `MICROSTEPS`, `TILT_LIMIT`, velocidades. Tudo exige rebuild. Para o objetivo #6
  (calibração geométrica) e #7 (perfis), precisam virar variáveis em NVS.
- **🟢 Carregamento NVS no boot com fallback para defaults:** correto e bem feito.

### `kinematics.c` — porte fiel, sem validação de domínio

- **🟡 Sem proteção de domínio do `acos`.** `kinematics.c:43-62`: se a geometria estiver
  errada ou o tilt for grande, o argumento de `acos` sai de `[-1,1]` → `NaN`. Há guarda *a
  jusante* em `apply_tilt` (`isfinite` → cai para nível), mas **silenciosa** e *por perna* —
  pode resultar em mesa parcialmente comandada (uma perna ao nível, outras não) sem nenhum
  aviso na telemetria. Para o objetivo #6 (singularidades, limites, saturação) é preciso
  detectar e **reportar** o evento, e idealmente saturar o tilt no maior valor factível em
  vez de zerar.
- **🟡 Sem cálculo de limites físicos.** Não há como saber, a priori, qual `|nx|,|ny|`
  máximo a geometria atual suporta. O `TILT_LIMIT=0.25` é um chute global, não derivado dos
  `GEO_*`.
- **🟢 Matemática correta** e idêntica ao porte Python — bom para validação cruzada.

### `steppers.c` — bom perfil trapezoidal, custo de CPU no ISR

- **🟡 `esp_rom_delay_us` (busy-wait) dentro do callback do timer de 10 kHz**
  (`steppers.c:63-68`). Com 3 motores e troca de DIR, gasta dezenas de µs por tick segurando
  a CPU. Funciona a 1500 passos/s, mas é a abordagem que o próprio header recomenda migrar
  para **RMT/MCPWM** (geração de pulso por hardware) — relevante se quiser mais
  resolução/velocidade.
- **🟡 Sem soft-limits de passo.** `stepper_move_to` aceita qualquer alvo
  (`steppers.c:155-159`). Sem limit switch nem clamp, um comando ruim (NaN tratado, ou
  geometria errada) pode tentar levar o braço a um batente. Objetivo #8.
- **🟢 Lógica de rampa/parada exata correta** (para em `current==target`, zera `accum`).

### `touch_screen.c` — funcional, ineficiente e sem rejeição de outliers

- **🟡 Reconfiguração pesada por leitura.** `gpio_config()` é chamado ~7×/ciclo e
  `adc_oneshot_config_channel` 2×/ciclo (`touch_screen.c:104-124`), redundante (já
  configurado no init). A 50 Hz funciona, mas é desperdício e adiciona latência/jitter.
- **🟡 Filtragem só IIR, sem mediana.** Resistiva 4 fios com bola de aço dá *spikes*. Só há
  IIR `α=0,25` (`touch_screen.c:199-205`) — não rejeita outliers, só os suaviza. Uma
  **mediana de N amostras** (a coleta de calibração já usa mediana; a leitura normal, não)
  reduziria muito o ruído que o objetivo #8 cita.
- **🟡 Detecção de toque sem histerese.** Limiar único `300` (`touch_screen.c:169`) →
  *chattering* perto do limiar. Histerese (entra em 400, sai em 250) + contagem de N
  leituras consecutivas estabiliza.
- **🟢 Guarda de `span==0`** já existe (evita divisão por zero/NaN no PID). Bom.

### `cal_store.c` — correto, mas rígido

- **🟡 Blobs de tamanho fixo sem versão.** `cal_pid_t`/`cal_touch_t` são gravados por
  `sizeof` (`cal_store.c:34,46`). Ao expandir a struct (limites do PID, geometria,
  mapeamento de motor, perfis), o `sz == sizeof` rejeita blobs antigos e você perde a
  calibração silenciosamente. **Falta um campo de versão/migração** — pré-requisito para os
  objetivos #1, #4 e #7 sem quebrar o que já está salvo.
- **🟡 Sem suporte a múltiplas chaves/namespaces para perfis** (objetivo #7).

### `main.c` — ok; `MAPPING` é código morto divergente

- O modo `BB_MODE_MAPPING` (`main.c:35-134`) duplica leitura/telemetria do `control.c` e
  tem seu próprio `cal_obs`. Mantém-se, mas é uma segunda implementação a manter em
  sincronia.

### Ferramentas Python — três portes do PID que **não batem entre si**

- **🔴 Inconsistência de PID:** `pid.c` e `sim_pid.py` têm filtro derivativo;
  **`calibrate.py` (`calibrate.py:164-187`) e `PIDSimba.py` (linha 164-180) NÃO têm** —
  usam derivada crua. Logo a tela BALANCE e o PIDSimba **não reproduzem o firmware**, apesar
  de os comentários afirmarem "porte fiel". Para uma ferramenta de tuning isso é enganoso.
- **🟡 PC calcula posição/erro por conta própria** (`calibrate.py:970-983`) usando
  `self.cal` + orientação local, em vez de consumir `CTRL` (que o firmware já emite com
  `x_mm`, `setpoint`, `nx/ny`, `θA/B/C`). Se NVS ≠ `config.py`, o gráfico mente. O parser
  ignora `CTRL`, `SIGNS`, `CORNER`, `CAL_HINT` (`calibrate.py:230-251`).
- **🟡 Acoplamento da ferramenta com o estado do firmware:** `SETCAL` + `CAL SAVE` com
  `time.sleep(0.15)` (`calibrate.py:1300-1303`) — frágil; não confirma com o eco antes de
  prosseguir.

---

## Parte 3 — Onde cada objetivo encalha hoje (resumo)

| Objetivo | Bloqueio atual |
|---|---|
| #1 Config completa (motor A/B/C, inversão, neutro, rotação 0–270, swap, sentido) | Falha ❶ (sem matriz sensor→atuador) + ❷ (sem homing) + struct NVS sem campos/versão |
| #2 Ferramenta de comissionamento | Não há comandos para mover **motor individual**, inverter por motor, offset por motor; ferramenta lê de `config.py`, não do firmware |
| #3 Auditoria PID | back-calculation acoplado a `Ki`; reset duro; limites fixos; acoplamento de eixos vem de ❶ |
| #4 Tuning em tempo real | KP/KI/KD já dá; **faltam** limites de saída/integral/filtros e gráficos de saída-do-PID (PC recalcula em vez de ler `CTRL`) |
| #5 Telemetria avançada | `CTRL` existe mas é subutilizado; sem flags de saturação/anti-windup; `printf` concorrente corrompe linhas; protocolo CSV posicional é frágil |
| #6 Cinemática | sem proteção/telemetria de singularidade; sem limites derivados da geometria; `GEO_*` hardcoded |
| #7 Perfis | NVS sem versão nem múltiplas chaves |
| #8 Robustez | sem homing, sem soft-limits, sem watchdog de loop, sem histerese/mediana no toque, reset duro |
| #9 Qualidade | 3 PIDs divergentes; 4 fontes de verdade; `control.c` concentra parser+geometria+telemetria+loop |

---

## Parte 4 — Plano de implementação em 6 fases

Princípios respeitados: incremental, sem quebrar o que existe, protocolo retrocompatível
(comandos novos *adicionados*, nunca removidos), NVS versionado (blobs antigos continuam
carregando). Prioridade: 🔴 alta, 🟡 média, 🟢 baixa.

### Fase 1 — Correções críticas (fundação de segurança e integridade) ✅ CONCLUÍDA

> **Implementado em 2026-06-02.** Resumo do que foi feito:
>
> 1. **NVS versionado** (`cal_store.c/.h`): blobs agora têm cabeçalho
>    `{magic, version, size}`. O load preenche só `min(salvo, atual)` bytes, então
>    expandir as structs em fases futuras **mantém** os campos antigos e usa default
>    nos novos. Blobs legados (formato v0 sem cabeçalho) são **detectados e migrados**
>    automaticamente — nada de calibração perdida. API pública inalterada.
> 2. **Anti-windup por integração condicional** (`pid.c`): substituída a
>    back-calculation `integ -= (out-out_max)/ki` (que, com `Ki` minúsculo e D
>    saturando, estourava o integrador) por congelamento da integração quando ela
>    agravaria a saturação. Independe de `Ki`. Porte espelhado em `tools/sim_pid.py`.
> 3. **Mediana + histerese no toque** (`touch_screen.c/.h`): leitura por **mediana**
>    de 8 amostras (rejeita spikes, antes era média); detecção com **histerese**
>    (`ON=400`/`OFF=250`) + **debounce** de 2 leituras (mata chattering no limiar).
> 4. **Hold de toque** (`control.c`): em dropout, segura a última inclinação por
>    `300 ms` antes de resetar o PID/nivelar — falhas intermitentes da tela não
>    fazem mais a mesa pular.
> 5. **Saída serial serializada** (`control.c`): toda telemetria/eco passa por
>    `tlm_printf()` — formata a linha inteira e emite numa única `uart_write_bytes()`
>    sob mutex, em driver com **buffer de TX (1 KB)**. Fim das linhas intercaladas
>    entre as duas tasks e do `printf` bloqueante travando o laço.
> 6. **Proteção da calibração compartilhada** (`touch_screen.c`): spinlock curtíssimo
>    torna a atualização dos 7 campos atômica vs. a leitura no laço (snapshot em
>    `touch_read`). Ganhos/sinais ficam como `float` (atômicos por largura).
>
> **Validação:** `tools/sim_pid.py` (regressão + perturbação que satura o D) confirma
> recuperação monotônica, sem o estouro do integrador antigo, mantendo o
> comportamento documentado na `TEORIA.md`. *(Build ESP-IDF não verificado nesta
> máquina — sem toolchain; mudanças revisadas manualmente.)*

**Arquivos:** `pid.c/.h`, `control.c`, `touch_screen.c`, `cal_store.c/.h`, `main.c`.

**Mudanças:**
1. **Serializar a saída serial** — uma única função `tlm_printf`/fila protegida por mutex;
   instalar UART com buffer de TX real e voltar a `stdout` *line-buffered* (ou rota
   dedicada). Elimina linhas corrompidas e o bloqueio do loop.
2. **Anti-windup desacoplado de `Ki`** — integração condicional ou tracking `Tt` próprio em
   `pid_update`.
3. **Hold/debounce de toque** no `control.c` (espelhar `PRESENCE_HOLD`) — parar de resetar o
   PID e saltar para nível a cada dropout.
4. **Histerese + mediana** em `touch_read`.
5. **Versionar os blobs NVS** (`cal_store`): campo `version` + `magic`, com migração que
   preserva blobs antigos. *Pré-requisito de todas as fases seguintes.*
6. **Mutex** para escrita de ganhos/calibração compartilhados.

**Riscos:** mexer no anti-windup muda a resposta sintonizada (revalidar com `sim_pid.py`);
buffer de TX consome RAM.
**Benefícios:** telemetria confiável, controle sem jitter, NVS evolutivo, sensor estável.
**Prioridade:** 🔴 — nada robusto se constrói sem isto.

### Fase 2 — Sistema de calibração, orientação e referencial (objetivos #1 e #2)

**Arquivos:** `control.c` (novo módulo sugerido `frame.c/.h`), `kinematics.c/.h`,
`cal_store.c/.h`, `steppers.c/.h`, `calibrate.py`, `config.py`.

**Mudanças:**
1. **Matriz/descritor de transformação sensor→controle→atuador** (resolve falha ❶):
   persistir rotação {0,90,180,270}, swap, sinal por eixo e — separadamente —
   **mapeamento de motor** (qual físico é A/B/C), **inversão por motor** e **offset de passo
   por motor**. Aplicado em `apply_tilt` com `θ_orig` por perna.
2. **Comandos seriais novos** (retrocompatíveis): `MOTOR <id> STEP <±n>` (mover motor
   individual para identificá-lo fisicamente), `MOTOR <id> INVERT`, `MOTOR MAP A=..`,
   `HOME SET` (grava posição atual como neutro = solução pragmática para ❷ sem hardware
   extra), `FRAME ROT 90`, etc.
3. **Procedimento de comissionamento na `calibrate.py`**: nova tela "Mecânica" — mover cada
   motor, o usuário confirma qual respondeu, inverte, ajusta offset, define frente/trás/
   esq/dir e grava. A ferramenta passa a **ler do firmware** (`?`/`CAL SHOW`/`CTRL`),
   aposentando `config.py` como fonte (vira só fallback).
4. **Unificar o algoritmo de calibração**: a `calibrate.py` usa o caminho
   `CAL TL/TR/BL/BR`+`CAL APPLY` do firmware (uma só matemática), em vez de `SETCAL` próprio.

**Riscos:** maior superfície de NVS (mitigado pela Fase 1); risco de inverter motor errado
(mitigado por mover-e-confirmar).
**Benefícios:** entrega integral dos objetivos #1 e #2; elimina o acoplamento de eixos
estrutural.
**Prioridade:** 🔴/🟡.

### Fase 3 — PID tuning em tempo real completo (objetivos #4 e #5)

**Arquivos:** `pid.c/.h`, `control.c`, `cal_store.c/.h`, `calibrate.py`.

**Mudanças:**
1. Comandos para **limites de saída**, **limite de integral**, **`d_tau`/filtro** e
   **setpoint** ao vivo; persistir todos no blob PID versionado.
2. **Telemetria expandida** (protocolo extensível, *key=value* além do CSV legado):
   `x_raw,y_raw,x_mm,y_mm,err_x,err_y,u_x,u_y,nx,ny,θA,θB,θC,sat_x,sat_y,iwind_x,iwind_y`.
   Manter `POS`/`CTRL` antigos para não quebrar nada.
3. `calibrate.py` passa a **consumir `CTRL`/telemetria** e plotar **erro, saída do PID,
   setpoint e posição reais do firmware** (não mais recálculo no PC); logging em arquivo.
4. **Unificar os 3 PIDs Python** num só módulo com o filtro derivativo (acabar com a
   divergência `calibrate.py`/`PIDSimba.py`).

**Riscos:** banda serial (mitigar com taxa de telemetria configurável).
**Benefícios:** tuning fiel e observável; fim do "gráfico que mente".
**Prioridade:** 🟡.

### Fase 4 — Melhorias da cinemática (objetivo #6)

**Arquivos:** `kinematics.c/.h`, `control.c`, `cal_store.c/.h`, `tools/`.

**Mudanças:**
1. **`GEO_*`, `MICROSTEPS`, `TILT_LIMIT`, velocidades → variáveis NVS** (comandos `GEO ...`,
   `GEOSAVE`).
2. **Proteção e telemetria de singularidade**: detectar argumento de `acos` fora de domínio,
   **saturar no tilt máximo factível** (não zerar) e emitir flag na telemetria.
3. **Cálculo dos limites físicos** de `|nx|,|ny|` a partir da geometria → `TILT_LIMIT`
   derivado, não chutado.
4. **Calibração geométrica assistida** na ferramenta: usar a relação ângulo-comandado ×
   inclinação observada (via deslocamento da bola) para estimar/ajustar os `GEO_*` reais.

**Riscos:** geometria errada salva piora o controle (manter `GEO RESET`).
**Benefícios:** segurança contra NaN, tilt máximo aproveitado, fim dos placeholders.
**Prioridade:** 🟡.

### Fase 5 — Diagnóstico avançado e robustez (objetivo #8)

**Arquivos:** `control.c`, `steppers.c/.h`, `touch_screen.c`, `main.c`, `calibrate.py`.

**Mudanças:**
1. **Watchdog de aplicação** (TWDT alimentado + verificação de que o loop roda a 50 Hz;
   modo seguro = nivelar e desabilitar drivers em falha).
2. **Soft-limits de passo** por motor (clamp em `stepper_move_to`); reação segura a perda
   prolongada de toque.
3. **Slew-rate limit** opcional em `nx/ny` (limitar variação de tilt por ciclo) contra
   comandos abruptos.
4. **Painel de diagnóstico** na ferramenta: saturações, anti-windup ativo, taxa de loop,
   dropouts do sensor, distância dos limites mecânicos.

**Riscos:** modo seguro agressivo demais derruba a bola (tornar configurável).
**Benefícios:** operação segura, falhas observáveis.
**Prioridade:** 🟡.

### Fase 6 — Perfis de configuração + limpeza arquitetural (objetivos #7 e #9)

**Arquivos:** `cal_store.c/.h`, novo `config_profiles`, `control.c`, ferramentas,
`README`/`TEORIA`.

**Mudanças:**
1. **Perfis em NVS** (Laboratório/Demo/Teste/Competição): cada um agrega PID + orientação/
   frame + calibração + geometria. Comandos `PROFILE SAVE <n>`, `PROFILE LOAD <n>`,
   `PROFILE LIST`. Viável porque a Fase 1 versionou o NVS.
2. **Refatorar `control.c`**: extrair o **parser serial** (`cmd.c`), a **telemetria**
   (`tlm.c`) e o **frame/geometria** (`frame.c`) — `control.c` fica só com o laço. Reduz o
   acoplamento apontado no objetivo #9.
3. **Eliminar fontes de verdade duplicadas**: `config.py` e os `#define` de calibração viram
   *fallback* documentado; NVS é a verdade.
4. **Atualizar `README.md`/`TEORIA.md`** refletindo tudo (e corrigir a afirmação de "porte
   fiel" dos PIDs Python após a unificação).

**Riscos:** refator amplo — fazer por módulo, com `sim_pid.py` como rede de regressão do PID.
**Benefícios:** escalável, manutenível, sem divergências.
**Prioridade:** 🟢/🟡.

---

## Sequenciamento recomendado

```
Fase 1 (fundação)  →  Fase 2 (frame+comissionamento)  →  Fase 3 (tuning/telemetria)
                                                          ↘ Fase 4 (cinemática)
                                                          ↘ Fase 5 (robustez)
                                                                    ↘ Fase 6 (perfis+limpeza)
```

Fase 1 é pré-requisito de todas (NVS versionado + telemetria limpa). Fases 4 e 5 podem
correr em paralelo após a 2/3. A 6 fecha.
