#ifndef MEMORY_MAPS_H
#define MEMORY_MAPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "memory_config.h"

typedef struct __attribute__((packed)) {
    float x_m;
    float y_m;
} memory_map_point_t;

typedef struct {
    uint8_t slot;
    char name[MEMORY_MAP_NAME_MAX_LEN + 1];
    uint16_t point_count;
    float distance_m;
} memory_map_info_t;

typedef struct {
    bool active;
    uint16_t point_count;
    uint16_t rejected_points;
    float distance_m;
    char name[MEMORY_MAP_NAME_MAX_LEN + 1];
} memory_map_record_state_t;

esp_err_t memory_maps_init(void);
esp_err_t memory_maps_list(memory_map_info_t *maps, size_t max_maps, size_t *out_count);
esp_err_t memory_maps_save(const char *name,
                           uint16_t total_points,
                           const memory_map_point_t *points);
esp_err_t memory_maps_save_chunk(const char *name,
                                 uint16_t total_points,
                                 uint16_t offset,
                                 const memory_map_point_t *points,
                                 uint8_t point_count);
esp_err_t memory_maps_delete(uint8_t slot);
esp_err_t memory_maps_load_chunk(uint8_t slot,
                                 uint16_t offset,
                                 memory_map_point_t *points,
                                 uint8_t max_points,
                                 uint16_t *out_total_points,
                                 uint8_t *out_point_count);
esp_err_t memory_maps_record_start(const char *name);
esp_err_t memory_maps_record_stop(void);
esp_err_t memory_maps_record_save(const char *name);
esp_err_t memory_maps_record_update(void);
bool memory_maps_record_get_state(memory_map_record_state_t *out_state);
esp_err_t memory_maps_record_load_chunk(uint16_t offset,
                                        memory_map_point_t *points,
                                        uint8_t max_points,
                                        uint16_t *out_total_points,
                                        uint8_t *out_point_count,
                                        memory_map_record_state_t *out_state);

#endif
