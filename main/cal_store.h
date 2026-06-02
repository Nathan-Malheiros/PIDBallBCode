#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Persistent storage for runtime-calibrated values (NVS, namespace "bb_cal").
 * Call cal_store_init() once before any load/save operation.
 *
 * ── Versionamento (Fase 1) ───────────────────────────────────────────────────
 * Cada blob e gravado com um cabecalho { magic, version, size }. No load:
 *   - blob com magic valido  -> copiado respeitando min(tamanho salvo, atual),
 *     de modo que campos NOVOS (adicionados em fases futuras) mantem o default
 *     que o chamador preencheu ANTES de chamar o load;
 *   - blob legado (sem cabecalho, formato v0) -> migrado automaticamente e
 *     regravado no proximo save ja com cabecalho.
 * Assim, expandir as structs abaixo no futuro NUNCA descarta a calibracao salva.
 */

/* Versao corrente de cada payload. Incremente ao adicionar campos. */
#define CAL_PID_VERSION    1
#define CAL_TOUCH_VERSION  1

typedef struct {
    float kp, ki, kd;
} cal_pid_t;

typedef struct {
    int32_t x_raw_min, x_raw_max;
    int32_t y_raw_min, y_raw_max;
    int8_t  flip_x, flip_y, swap_xy;
} cal_touch_t;

/* Initialize NVS flash — erases partition only if version mismatch or full. */
void cal_store_init(void);

/* Returns true if a valid PID blob was found in NVS (versionado ou legado). */
bool cal_store_load_pid(cal_pid_t *out);
void cal_store_save_pid(const cal_pid_t *in);

/* Returns true if a valid touch calibration blob was found (versionado/legado). */
bool cal_store_load_touch(cal_touch_t *out);
void cal_store_save_touch(const cal_touch_t *in);
