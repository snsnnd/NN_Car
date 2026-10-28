#ifndef NNCAR_MAP_FILE_H
#define NNCAR_MAP_FILE_H

#include "nncar/map.h"

#include <stddef.h>

/* Loads the nodes/edges JSON map format; the map owns its allocated segments. */
int nncar_map_load_json(const char *path, nncar_map_t *map,
                        char *error, size_t error_capacity);
int nncar_map_load_json_text(const char *json, size_t json_length,
                             nncar_map_t *map, char *error,
                             size_t error_capacity);
void nncar_map_free(nncar_map_t *map);

#endif
