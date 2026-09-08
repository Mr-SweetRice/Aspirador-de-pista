#include "memory_maps.h"

#include <math.h>
#include <stddef.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "odometry.h"

#define MAP_MAGIC 0x50414d4fU
#define MAP_INDEX_KEY "map_idx"
#define MAP_RECORD_MIN_STEP_M 0.02f
#define MAP_RECORD_MAX_STEP_M 0.30f
#define MAP_RECORD_50M_REQUIRED_POINTS 2501

_Static_assert(MEMORY_MAP_MAX_POINTS >= MAP_RECORD_50M_REQUIRED_POINTS,
               "O buffer de mapa deve comportar pelo menos 50 m com passos de 2 cm");

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t point_count;
    float distance_m;
    memory_map_point_t points[MEMORY_MAP_MAX_POINTS];
} stored_map_t;

typedef struct __attribute__((packed)) {
    uint8_t used[MEMORY_MAP_MAX_COUNT];
    char names[MEMORY_MAP_MAX_COUNT][MEMORY_MAP_NAME_MAX_LEN + 1];
} map_index_t;

/* Read cache is used by map transfer/start, never by the 1 kHz integration loop. */
static stored_map_t read_cache;
static bool read_cache_valid;
static uint8_t read_cache_slot;
static unsigned read_cache_revision;
static atomic_uint maps_revision;
static SemaphoreHandle_t read_cache_mutex;

static const char *TAG = "memory_maps";
static portMUX_TYPE record_mux = portMUX_INITIALIZER_UNLOCKED;
static memory_map_point_t record_points[MEMORY_MAP_MAX_POINTS];
static char record_name[MEMORY_MAP_NAME_MAX_LEN + 1];
static uint16_t record_point_count;
static uint16_t record_rejected_points;
static float record_distance_m;
static bool record_active;

static void map_key(uint8_t slot, char *out_key, size_t out_key_len)
{
    snprintf(out_key, out_key_len, "map%u", (unsigned int)slot);
}

static esp_err_t load_index(nvs_handle_t handle, map_index_t *index)
{
    memset(index, 0, sizeof(*index));
    size_t len = sizeof(*index);
    esp_err_t ret = nvs_get_blob(handle, MAP_INDEX_KEY, index, &len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (ret != ESP_OK || len != sizeof(*index)) {
        memset(index, 0, sizeof(*index));
        return ret == ESP_OK ? ESP_ERR_INVALID_SIZE : ret;
    }
    return ESP_OK;
}

static esp_err_t save_index(nvs_handle_t handle, const map_index_t *index)
{
    esp_err_t ret = nvs_set_blob(handle, MAP_INDEX_KEY, index, sizeof(*index));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    atomic_fetch_add(&maps_revision, 1);
    return ret;
}

static int find_slot(const map_index_t *index, const char *name)
{
    for (int i = 0; i < MEMORY_MAP_MAX_COUNT; ++i) {
        if (index->used[i] && strncmp(index->names[i], name, MEMORY_MAP_NAME_MAX_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

static int find_free_slot(const map_index_t *index)
{
    for (int i = 0; i < MEMORY_MAP_MAX_COUNT; ++i) {
        if (!index->used[i]) {
            return i;
        }
    }
    return -1;
}

static float calculate_map_distance(uint16_t total_points, const memory_map_point_t *points)
{
    float distance = 0.0f;
    for (uint16_t i = 1; i < total_points; ++i) {
        const float dx = points[i].x_m - points[i - 1].x_m;
        const float dy = points[i].y_m - points[i - 1].y_m;
        distance += sqrtf((dx * dx) + (dy * dy));
    }
    return distance;
}

static bool point_is_valid(memory_map_point_t point)
{
    return isfinite(point.x_m) && isfinite(point.y_m);
}

static float point_distance(memory_map_point_t a, memory_map_point_t b)
{
    const float dx = b.x_m - a.x_m;
    const float dy = b.y_m - a.y_m;
    return sqrtf((dx * dx) + (dy * dy));
}

static void sanitize_map_name(const char *name, char *out_name, size_t out_name_len)
{
    if (out_name == NULL || out_name_len == 0) {
        return;
    }
    memset(out_name, 0, out_name_len);
    if (name == NULL || name[0] == '\0') {
        strncpy(out_name, "mapa", out_name_len - 1);
        return;
    }
    strncpy(out_name, name, out_name_len - 1);
}

esp_err_t memory_maps_init(void)
{
    if (read_cache_mutex == NULL) {
        read_cache_mutex = xSemaphoreCreateMutex();
        if (read_cache_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    map_index_t index;
    ret = load_index(handle, &index);
    if (ret == ESP_ERR_NVS_NOT_FOUND || ret == ESP_ERR_INVALID_SIZE) {
        memset(&index, 0, sizeof(index));
        ret = save_index(handle, &index);
    }
    nvs_close(handle);
    return ret;
}

esp_err_t memory_maps_list(memory_map_info_t *maps, size_t max_maps, size_t *out_count)
{
    if (maps == NULL || out_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    map_index_t index;
    ret = load_index(handle, &index);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    size_t count = 0;
    bool index_dirty = false;
    for (uint8_t slot = 0; slot < MEMORY_MAP_MAX_COUNT && count < max_maps; ++slot) {
        if (!index.used[slot]) {
            continue;
        }

        stored_map_t *map = calloc(1, sizeof(stored_map_t));
        if (map == NULL) {
            continue;
        }
        size_t len = sizeof(*map);
        char key[8];
        map_key(slot, key, sizeof(key));
        ret = nvs_get_blob(handle, key, map, &len);
        if (ret != ESP_OK || len < offsetof(stored_map_t, points) || map->magic != MAP_MAGIC ||
            map->point_count > MEMORY_MAP_MAX_POINTS ||
            len < offsetof(stored_map_t, points) + (map->point_count * sizeof(memory_map_point_t))) {
            ESP_LOGW(TAG,
                     "Slot mapa invalido removido slot=%u ret=%s len=%u",
                     (unsigned int)slot,
                     esp_err_to_name(ret),
                     (unsigned int)len);
            index.used[slot] = 0;
            memset(index.names[slot], 0, sizeof(index.names[slot]));
            nvs_erase_key(handle, key);
            index_dirty = true;
            free(map);
            continue;
        }

        maps[count].slot = slot;
        strncpy(maps[count].name, index.names[slot], sizeof(maps[count].name) - 1);
        maps[count].name[sizeof(maps[count].name) - 1] = '\0';
        maps[count].point_count = map->point_count;
        maps[count].distance_m = calculate_map_distance(map->point_count, map->points);
        free(map);
        ++count;
    }

    *out_count = count;
    if (index_dirty) {
        save_index(handle, &index);
    }
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t memory_maps_save(const char *name,
                           uint16_t total_points,
                           const memory_map_point_t *points)
{
    if (name == NULL || points == NULL || name[0] == '\0' ||
        total_points == 0 ||
        total_points > MEMORY_MAP_MAX_POINTS) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    map_index_t index;
    ret = load_index(handle, &index);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    int slot = find_slot(&index, name);
    if (slot < 0) {
        slot = find_free_slot(&index);
    }
    if (slot < 0) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    stored_map_t *map = calloc(1, sizeof(stored_map_t));
    if (map == NULL) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }
    map->magic = MAP_MAGIC;
    map->point_count = total_points;
    map->distance_m = calculate_map_distance(total_points, points);
    const float distance_m = map->distance_m;
    memcpy(map->points, points, total_points * sizeof(memory_map_point_t));

    char key[8];
    map_key((uint8_t)slot, key, sizeof(key));
    ret = nvs_set_blob(handle, key, map, sizeof(*map));
    free(map);
    if (ret == ESP_OK) {
        index.used[slot] = 1;
        strncpy(index.names[slot], name, MEMORY_MAP_NAME_MAX_LEN);
        index.names[slot][MEMORY_MAP_NAME_MAX_LEN] = '\0';
        ret = save_index(handle, &index);
    }
    nvs_close(handle);

    ESP_LOGI(TAG,
             "Mapa salvo name=%s total=%u distance=%.3f first=[%.3f %.3f] last=[%.3f %.3f] ret=%s",
             name,
             (unsigned int)total_points,
             distance_m,
             points[0].x_m,
             points[0].y_m,
             points[total_points - 1].x_m,
             points[total_points - 1].y_m,
             esp_err_to_name(ret));
    return ret;
}

esp_err_t memory_maps_save_chunk(const char *name,
                                 uint16_t total_points,
                                 uint16_t offset,
                                 const memory_map_point_t *points,
                                 uint8_t point_count)
{
    if (name == NULL || points == NULL || name[0] == '\0' ||
        total_points > MEMORY_MAP_MAX_POINTS ||
        point_count > MEMORY_MAP_CHUNK_MAX_POINTS ||
        offset > total_points ||
        (uint32_t)offset + point_count > total_points) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    map_index_t index;
    ret = load_index(handle, &index);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    int slot = find_slot(&index, name);
    if (slot < 0) {
        slot = find_free_slot(&index);
    }
    if (slot < 0) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    stored_map_t *map = calloc(1, sizeof(stored_map_t));
    if (map == NULL) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    char key[8];
    map_key((uint8_t)slot, key, sizeof(key));

    bool reset_map = offset == 0 || !index.used[slot];
    if (!reset_map) {
        size_t len = sizeof(*map);
        esp_err_t load_ret = nvs_get_blob(handle, key, map, &len);
        if (load_ret != ESP_OK ||
            len < offsetof(stored_map_t, points) ||
            map->magic != MAP_MAGIC ||
            map->point_count != total_points ||
            len < offsetof(stored_map_t, points) + (map->point_count * sizeof(memory_map_point_t))) {
            reset_map = true;
        }
    }

    if (reset_map) {
        memset(map, 0, sizeof(*map));
        map->magic = MAP_MAGIC;
        map->point_count = total_points;
    }

    memcpy(&map->points[offset], points, point_count * sizeof(memory_map_point_t));
    map->distance_m = calculate_map_distance(map->point_count, map->points);

    ret = nvs_set_blob(handle, key, map, sizeof(*map));
    free(map);
    if (ret == ESP_OK) {
        index.used[slot] = 1;
        strncpy(index.names[slot], name, MEMORY_MAP_NAME_MAX_LEN);
        index.names[slot][MEMORY_MAP_NAME_MAX_LEN] = '\0';
        ret = save_index(handle, &index);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG,
             "Chunk mapa salvo name=%s slot=%u total=%u offset=%u count=%u ret=%s",
             name,
             (unsigned int)slot,
             (unsigned int)total_points,
             (unsigned int)offset,
             (unsigned int)point_count,
             esp_err_to_name(ret));
    return ret;
}

esp_err_t memory_maps_delete(uint8_t slot)
{
    if (slot >= MEMORY_MAP_MAX_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    map_index_t index;
    ret = load_index(handle, &index);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    char deleted_name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};
    strncpy(deleted_name, index.names[slot], sizeof(deleted_name) - 1);

    char key[8];
    map_key(slot, key, sizeof(key));
    esp_err_t erase_ret = nvs_erase_key(handle, key);
    if (erase_ret != ESP_OK && erase_ret != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return erase_ret;
    }

    index.used[slot] = 0;
    memset(index.names[slot], 0, sizeof(index.names[slot]));
    ret = save_index(handle, &index);
    nvs_close(handle);

    ESP_LOGI(TAG,
             "Mapa apagado slot=%u name=%s ret=%s",
             (unsigned int)slot,
             deleted_name[0] != '\0' ? deleted_name : "-",
             esp_err_to_name(ret));
    return ret;
}

esp_err_t memory_maps_load_chunk(uint8_t slot,
                                 uint16_t offset,
                                 memory_map_point_t *points,
                                 uint8_t max_points,
                                 uint16_t *out_total_points,
                                 uint8_t *out_point_count)
{
    if (slot >= MEMORY_MAP_MAX_COUNT || points == NULL || max_points == 0 ||
        out_total_points == NULL || out_point_count == NULL) return ESP_ERR_INVALID_ARG;
    if (read_cache_mutex == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(read_cache_mutex, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    const unsigned revision = atomic_load(&maps_revision);
    if (!read_cache_valid || read_cache_slot != slot || read_cache_revision != revision) {
        read_cache_valid = false;
        nvs_handle_t handle;
        ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
        if (ret != ESP_OK) goto done;
        size_t len = sizeof(read_cache);
        char key[8];
        map_key(slot, key, sizeof(key));
        ret = nvs_get_blob(handle, key, &read_cache, &len);
        nvs_close(handle);
        if (ret != ESP_OK) goto done;
        if (len < offsetof(stored_map_t, points) || read_cache.magic != MAP_MAGIC ||
            read_cache.point_count > MEMORY_MAP_MAX_POINTS ||
            len < offsetof(stored_map_t, points) + read_cache.point_count * sizeof(memory_map_point_t)) {
            ret = ESP_ERR_INVALID_SIZE;
            goto done;
        }
        read_cache_slot = slot;
        read_cache_revision = revision;
        read_cache_valid = true;
    }
    if (offset > read_cache.point_count) {
        ret = ESP_ERR_INVALID_SIZE;
        goto done;
    }
    const uint16_t remaining = read_cache.point_count - offset;
    const uint8_t count = remaining > max_points ? max_points : (uint8_t)remaining;
    memcpy(points, &read_cache.points[offset], count * sizeof(memory_map_point_t));
    *out_total_points = read_cache.point_count;
    *out_point_count = count;
done:
    xSemaphoreGive(read_cache_mutex);
    return ret;
}

esp_err_t memory_maps_record_start(const char *name)
{
    char clean_name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};
    sanitize_map_name(name, clean_name, sizeof(clean_name));

    portENTER_CRITICAL(&record_mux);
    memset(record_points, 0, sizeof(record_points));
    memset(record_name, 0, sizeof(record_name));
    strncpy(record_name, clean_name, MEMORY_MAP_NAME_MAX_LEN);
    record_point_count = 0;
    record_rejected_points = 0;
    record_distance_m = 0.0f;
    record_active = true;
    portEXIT_CRITICAL(&record_mux);

    ESP_LOGI(TAG, "Gravacao de mapa iniciada name=%s", clean_name);
    return ESP_OK;
}

esp_err_t memory_maps_record_stop(void)
{
    uint16_t point_count = 0;
    uint16_t rejected_points = 0;
    float distance_m = 0.0f;

    portENTER_CRITICAL(&record_mux);
    record_active = false;
    point_count = record_point_count;
    rejected_points = record_rejected_points;
    distance_m = record_distance_m;
    portEXIT_CRITICAL(&record_mux);

    ESP_LOGI(TAG,
             "Gravacao de mapa parada points=%u rejected=%u distance=%.3f",
             (unsigned int)point_count,
             (unsigned int)rejected_points,
             distance_m);
    return ESP_OK;
}

esp_err_t memory_maps_record_save(const char *name)
{
    uint16_t point_count = 0;
    char clean_name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};

    portENTER_CRITICAL(&record_mux);
    record_active = false;
    point_count = record_point_count;
    sanitize_map_name((name != NULL && name[0] != '\0') ? name : record_name, clean_name, sizeof(clean_name));
    portEXIT_CRITICAL(&record_mux);

    if (point_count < 2) {
        ESP_LOGW(TAG, "Gravacao de mapa sem pontos suficientes para salvar points=%u", (unsigned int)point_count);
        return ESP_ERR_INVALID_SIZE;
    }

    memory_map_point_t *snapshot = calloc(point_count, sizeof(memory_map_point_t));
    if (snapshot == NULL) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&record_mux);
    if (point_count > record_point_count) {
        point_count = record_point_count;
    }
    memcpy(snapshot, record_points, point_count * sizeof(memory_map_point_t));
    portEXIT_CRITICAL(&record_mux);

    esp_err_t ret = memory_maps_save(clean_name, point_count, snapshot);
    free(snapshot);

    ESP_LOGI(TAG,
             "Gravacao de mapa salva name=%s points=%u ret=%s",
             clean_name,
             (unsigned int)point_count,
             esp_err_to_name(ret));
    return ret;
}

esp_err_t memory_maps_record_update(void)
{
    portENTER_CRITICAL(&record_mux);
    const bool active = record_active;
    portEXIT_CRITICAL(&record_mux);
    if (!active) {
        return ESP_OK;
    }

    odometry_state_t odometry = {0};
    if (!odometry_get_state(&odometry)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!odometry.imu_available) {
        return ESP_OK;
    }

    const memory_map_point_t current = {
        .x_m = odometry.fused_x_m,
        .y_m = odometry.fused_y_m,
    };
    if (!point_is_valid(current)) {
        return ESP_OK;
    }

    bool buffer_full = false;
    portENTER_CRITICAL(&record_mux);
    if (record_active) {
        if (record_point_count == 0) {
            record_points[record_point_count++] = current;
        } else {
            const memory_map_point_t previous = record_points[record_point_count - 1];
            const float distance_m = point_distance(previous, current);
            if (distance_m > MAP_RECORD_MAX_STEP_M) {
                if (record_rejected_points < UINT16_MAX) {
                    ++record_rejected_points;
                }
            } else if (distance_m >= MAP_RECORD_MIN_STEP_M) {
                if (record_point_count < MEMORY_MAP_MAX_POINTS) {
                    record_points[record_point_count++] = current;
                    record_distance_m += distance_m;
                } else {
                    record_active = false;
                    buffer_full = true;
                }
            }
        }
    }
    portEXIT_CRITICAL(&record_mux);

    if (buffer_full) {
        ESP_LOGW(TAG, "Gravacao de mapa encerrada: limite de %u pontos", (unsigned int)MEMORY_MAP_MAX_POINTS);
    }
    return buffer_full ? ESP_ERR_NO_MEM : ESP_OK;
}

bool memory_maps_record_get_state(memory_map_record_state_t *out_state)
{
    if (out_state == NULL) {
        return false;
    }

    portENTER_CRITICAL(&record_mux);
    out_state->active = record_active;
    out_state->point_count = record_point_count;
    out_state->rejected_points = record_rejected_points;
    out_state->distance_m = record_distance_m;
    memset(out_state->name, 0, sizeof(out_state->name));
    strncpy(out_state->name, record_name, MEMORY_MAP_NAME_MAX_LEN);
    portEXIT_CRITICAL(&record_mux);
    return true;
}

esp_err_t memory_maps_record_load_chunk(uint16_t offset,
                                        memory_map_point_t *points,
                                        uint8_t max_points,
                                        uint16_t *out_total_points,
                                        uint8_t *out_point_count,
                                        memory_map_record_state_t *out_state)
{
    if (points == NULL || max_points == 0 || out_total_points == NULL || out_point_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&record_mux);
    const uint16_t total_points = record_point_count;
    if (offset > total_points) {
        portEXIT_CRITICAL(&record_mux);
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t remaining = total_points - offset;
    uint8_t count = remaining > max_points ? max_points : (uint8_t)remaining;
    if (count > 0) {
        memcpy(points, &record_points[offset], count * sizeof(memory_map_point_t));
    }
    *out_total_points = total_points;
    *out_point_count = count;
    if (out_state != NULL) {
        out_state->active = record_active;
        out_state->point_count = record_point_count;
        out_state->rejected_points = record_rejected_points;
        out_state->distance_m = record_distance_m;
        memset(out_state->name, 0, sizeof(out_state->name));
        strncpy(out_state->name, record_name, MEMORY_MAP_NAME_MAX_LEN);
    }
    portEXIT_CRITICAL(&record_mux);
    return ESP_OK;
}
