#include "cal_store.h"

#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "cal";
static const char *NS  = "bb_cal";

/* ── Cabecalho de blob versionado ─────────────────────────────────────────────
 * Gravado ANTES do payload. magic distingue um blob novo de um blob legado
 * (formato v0, gravado por versoes anteriores do firmware sem cabecalho). */
#define CAL_MAGIC  0x42424331u   /* "BBC1" */

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;     /* tamanho do payload em bytes */
} blob_hdr_t;

/* Layout v0 (legado, sem cabecalho) — usado so para detectar/migrar.
 * NUNCA altere estas structs: elas fotografam o formato antigo no disco. */
typedef struct { float kp, ki, kd; } cal_pid_v0_t;
typedef struct {
    int32_t x_raw_min, x_raw_max, y_raw_min, y_raw_max;
    int8_t  flip_x, flip_y, swap_xy;
} cal_touch_v0_t;

void cal_store_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erased (version mismatch or partition full)");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

/* ── helpers ──────────────────────────────────────────────────────────────── */

static bool ns_open(nvs_open_mode_t mode, nvs_handle_t *h)
{
    return nvs_open(NS, mode, h) == ESP_OK;
}

/* Grava payload precedido do cabecalho versionado. */
static void blob_save(const char *key, uint16_t version,
                      const void *payload, size_t payload_sz)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READWRITE, &h)) { ESP_LOGE(TAG, "NVS open failed"); return; }

    uint8_t buf[128];
    size_t total = sizeof(blob_hdr_t) + payload_sz;
    if (total > sizeof(buf)) { ESP_LOGE(TAG, "blob too large"); nvs_close(h); return; }

    blob_hdr_t hdr = { CAL_MAGIC, version, (uint16_t)payload_sz };
    memcpy(buf, &hdr, sizeof hdr);
    memcpy(buf + sizeof hdr, payload, payload_sz);

    ESP_ERROR_CHECK(nvs_set_blob(h, key, buf, total));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}

/*
 * Carrega um blob. O chamador deve preencher *payload com DEFAULTS antes, para
 * que campos ausentes em versoes antigas mantenham seus valores.
 * Retorno: -1 nao encontrado/invalido; 0 legado (migrado); >=1 versao lida.
 */
static int blob_load(const char *key, void *payload, size_t payload_sz,
                     size_t legacy_sz)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READONLY, &h)) return -1;

    size_t sz = 0;
    if (nvs_get_blob(h, key, NULL, &sz) != ESP_OK) { nvs_close(h); return -1; }

    uint8_t buf[128];
    if (sz == 0 || sz > sizeof(buf)) { nvs_close(h); return -1; }
    if (nvs_get_blob(h, key, buf, &sz) != ESP_OK) { nvs_close(h); return -1; }
    nvs_close(h);

    /* Formato novo: cabecalho com magic. */
    if (sz >= sizeof(blob_hdr_t)) {
        blob_hdr_t hdr;
        memcpy(&hdr, buf, sizeof hdr);
        if (hdr.magic == CAL_MAGIC) {
            size_t stored = hdr.size;
            if (stored > sz - sizeof(blob_hdr_t)) stored = sz - sizeof(blob_hdr_t);
            size_t copy = stored < payload_sz ? stored : payload_sz;
            memcpy(payload, buf + sizeof hdr, copy);
            return (int)hdr.version;
        }
    }

    /* Formato legado v0: payload cru, sem cabecalho. */
    if (sz == legacy_sz) {
        size_t copy = legacy_sz < payload_sz ? legacy_sz : payload_sz;
        memcpy(payload, buf, copy);
        return 0;
    }

    return -1;
}

/* ── PID ──────────────────────────────────────────────────────────────────── */

bool cal_store_load_pid(cal_pid_t *out)
{
    int v = blob_load("pid", out, sizeof(*out), sizeof(cal_pid_v0_t));
    if (v < 0) return false;
    ESP_LOGI(TAG, "PID loaded (v%d)  kp=%.4g  ki=%.4g  kd=%.4g",
             v, (double)out->kp, (double)out->ki, (double)out->kd);
    return true;
}

void cal_store_save_pid(const cal_pid_t *in)
{
    blob_save("pid", CAL_PID_VERSION, in, sizeof(*in));
    ESP_LOGI(TAG, "PID saved (v%d)  kp=%.4g  ki=%.4g  kd=%.4g",
             CAL_PID_VERSION, (double)in->kp, (double)in->ki, (double)in->kd);
}

/* ── Touch calibration ────────────────────────────────────────────────────── */

bool cal_store_load_touch(cal_touch_t *out)
{
    int v = blob_load("touch", out, sizeof(*out), sizeof(cal_touch_v0_t));
    if (v < 0) return false;
    ESP_LOGI(TAG, "Touch cal loaded (v%d)  X[%ld-%ld]  Y[%ld-%ld]  flip=%d/%d  swap=%d",
             v, (long)out->x_raw_min, (long)out->x_raw_max,
             (long)out->y_raw_min, (long)out->y_raw_max,
             out->flip_x, out->flip_y, out->swap_xy);
    return true;
}

void cal_store_save_touch(const cal_touch_t *in)
{
    blob_save("touch", CAL_TOUCH_VERSION, in, sizeof(*in));
    ESP_LOGI(TAG, "Touch cal saved (v%d)  X[%ld-%ld]  Y[%ld-%ld]  flip=%d/%d  swap=%d",
             CAL_TOUCH_VERSION,
             (long)in->x_raw_min, (long)in->x_raw_max,
             (long)in->y_raw_min, (long)in->y_raw_max,
             in->flip_x, in->flip_y, in->swap_xy);
}
