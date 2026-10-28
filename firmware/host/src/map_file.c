#include "nncar/map_file.h"

#include "nncar/types.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *at;
    const char *end;
} json_cursor_t;

typedef struct {
    int id;
    float x;
    float y;
} map_node_t;

typedef struct {
    map_node_t *items;
    size_t count;
    size_t capacity;
} node_list_t;

typedef struct {
    int a;
    int b;
} map_edge_t;

typedef struct {
    map_edge_t *items;
    size_t count;
    size_t capacity;
} edge_list_t;

static void skip_space(json_cursor_t *cursor)
{
    while (cursor->at < cursor->end && isspace((unsigned char)*cursor->at)) {
        ++cursor->at;
    }
}

static int read_string(json_cursor_t *cursor, char *output, size_t capacity)
{
    skip_space(cursor);
    if (cursor->at >= cursor->end || *cursor->at != '"') {
        return -1;
    }
    ++cursor->at;
    size_t used = 0u;
    while (cursor->at < cursor->end && *cursor->at != '"') {
        unsigned char ch = (unsigned char)*cursor->at++;
        if (ch == '\\') {
            if (cursor->at >= cursor->end) return -1;
            ch = (unsigned char)*cursor->at++;
            switch (ch) {
            case '"': case '\\': case '/': break;
            case 'b': ch = '\b'; break;
            case 'f': ch = '\f'; break;
            case 'n': ch = '\n'; break;
            case 'r': ch = '\r'; break;
            case 't': ch = '\t'; break;
            case 'u':
                if ((size_t)(cursor->end - cursor->at) < 4u) return -1;
                /* Map identifiers are ASCII; skip valid JSON unicode escapes. */
                for (int i = 0; i < 4; ++i) {
                    if (!isxdigit((unsigned char)cursor->at[i])) return -1;
                }
                cursor->at += 4;
                ch = '?';
                break;
            default: return -1;
            }
        } else if (ch < 0x20u) {
            return -1;
        }
        if (output != NULL) {
            if (used + 1u >= capacity) return -1;
            output[used] = (char)ch;
        }
        ++used;
    }
    if (cursor->at >= cursor->end || *cursor->at != '"') return -1;
    ++cursor->at;
    if (output != NULL) output[used] = '\0';
    return 0;
}

static int read_number(json_cursor_t *cursor, double *value)
{
    skip_space(cursor);
    if (cursor->at >= cursor->end) return -1;
    const char *start = cursor->at;
    const char *number_end = start;
    if (*number_end == '-') ++number_end;
    if (number_end >= cursor->end) return -1;
    if (*number_end == '0') {
        ++number_end;
    } else if (*number_end >= '1' && *number_end <= '9') {
        do { ++number_end; }
        while (number_end < cursor->end && isdigit((unsigned char)*number_end));
    } else {
        return -1;
    }
    if (number_end < cursor->end && *number_end == '.') {
        ++number_end;
        const char *fraction_start = number_end;
        while (number_end < cursor->end && isdigit((unsigned char)*number_end)) ++number_end;
        if (number_end == fraction_start) return -1;
    }
    if (number_end < cursor->end && (*number_end == 'e' || *number_end == 'E')) {
        ++number_end;
        if (number_end < cursor->end && (*number_end == '+' || *number_end == '-')) ++number_end;
        const char *exponent_start = number_end;
        while (number_end < cursor->end && isdigit((unsigned char)*number_end)) ++number_end;
        if (number_end == exponent_start) return -1;
    }
    errno = 0;
    char *after = NULL;
    double parsed = strtod(cursor->at, &after);
    if (after != number_end || errno == ERANGE ||
        !isfinite(parsed)) {
        return -1;
    }
    cursor->at = after;
    *value = parsed;
    return 0;
}

static int skip_value(json_cursor_t *cursor, unsigned depth)
{
    if (depth > 32u) return -1;
    skip_space(cursor);
    if (cursor->at >= cursor->end) return -1;
    if (*cursor->at == '"') return read_string(cursor, NULL, 0u);
    if (*cursor->at == '[' || *cursor->at == '{') {
        const char open = *cursor->at++;
        const char close = open == '[' ? ']' : '}';
        skip_space(cursor);
        if (cursor->at < cursor->end && *cursor->at == close) {
            ++cursor->at;
            return 0;
        }
        for (;;) {
            if (open == '{') {
                if (read_string(cursor, NULL, 0u) != 0) return -1;
                skip_space(cursor);
                if (cursor->at >= cursor->end || *cursor->at++ != ':') return -1;
            }
            if (skip_value(cursor, depth + 1u) != 0) return -1;
            skip_space(cursor);
            if (cursor->at >= cursor->end) return -1;
            if (*cursor->at == close) {
                ++cursor->at;
                return 0;
            }
            if (*cursor->at++ != ',') return -1;
        }
    }
    if ((size_t)(cursor->end - cursor->at) >= 4u &&
        (memcmp(cursor->at, "true", 4u) == 0 || memcmp(cursor->at, "null", 4u) == 0)) {
        cursor->at += 4;
        return 0;
    }
    if ((size_t)(cursor->end - cursor->at) >= 5u &&
        memcmp(cursor->at, "false", 5u) == 0) {
        cursor->at += 5;
        return 0;
    }
    double ignored = 0.0;
    return read_number(cursor, &ignored);
}

static int object_field(json_cursor_t object, const char *name, json_cursor_t *value)
{
    skip_space(&object);
    if (object.at >= object.end || *object.at++ != '{') return -1;
    skip_space(&object);
    if (object.at < object.end && *object.at == '}') return 0;
    for (;;) {
        char key[128];
        if (read_string(&object, key, sizeof(key)) != 0) return -1;
        skip_space(&object);
        if (object.at >= object.end || *object.at++ != ':') return -1;
        skip_space(&object);
        json_cursor_t candidate = object;
        if (strcmp(key, name) == 0) {
            *value = candidate;
            return 1;
        }
        if (skip_value(&object, 0u) != 0) return -1;
        skip_space(&object);
        if (object.at >= object.end) return -1;
        if (*object.at == '}') return 0;
        if (*object.at++ != ',') return -1;
    }
}

static int read_numeric_field(json_cursor_t object, const char *name,
                              double *number, bool required)
{
    json_cursor_t value;
    int found = object_field(object, name, &value);
    if (found <= 0) return required ? -1 : found;
    return read_number(&value, number);
}

static int read_id(json_cursor_t *cursor, int *id)
{
    double number = 0.0;
    if (read_number(cursor, &number) != 0 || number < 0.0 ||
        number > 2147483647.0 || floor(number) != number) {
        return -1;
    }
    *id = (int)number;
    return 0;
}

static int append_node(node_list_t *list, map_node_t node)
{
    if (list->count == list->capacity) {
        size_t capacity = list->capacity == 0u ? 16u : list->capacity * 2u;
        if (capacity > 8192u) return -1;
        map_node_t *items = realloc(list->items, capacity * sizeof(*items));
        if (items == NULL) return -1;
        list->items = items;
        list->capacity = capacity;
    }
    list->items[list->count++] = node;
    return 0;
}

static int append_edge(edge_list_t *list, map_edge_t edge)
{
    if (list->count == list->capacity) {
        size_t capacity = list->capacity == 0u ? 16u : list->capacity * 2u;
        if (capacity > 4096u) return -1;
        map_edge_t *items = realloc(list->items, capacity * sizeof(*items));
        if (items == NULL) return -1;
        list->items = items;
        list->capacity = capacity;
    }
    list->items[list->count++] = edge;
    return 0;
}

static int parse_nodes(json_cursor_t array, double scale, node_list_t *nodes)
{
    skip_space(&array);
    if (array.at >= array.end || *array.at++ != '[') return -1;
    skip_space(&array);
    while (array.at < array.end && *array.at != ']') {
        if (*array.at++ != '[') return -1;
        map_node_t node;
        double x = 0.0, y = 0.0;
        if (read_id(&array, &node.id) != 0) return -1;
        skip_space(&array);
        if (array.at >= array.end || *array.at++ != ',') return -1;
        if (read_number(&array, &x) != 0) return -1;
        skip_space(&array);
        if (array.at >= array.end || *array.at++ != ',') return -1;
        if (read_number(&array, &y) != 0) return -1;
        skip_space(&array);
        if (array.at >= array.end || *array.at++ != ']') return -1;
        node.x = (float)(x * scale);
        node.y = (float)(y * scale);
        if (!isfinite(node.x) || !isfinite(node.y) || append_node(nodes, node) != 0) {
            return -1;
        }
        skip_space(&array);
        if (array.at < array.end && *array.at == ',') {
            ++array.at;
            skip_space(&array);
            if (array.at < array.end && *array.at == ']') return -1;
        } else if (array.at >= array.end || *array.at != ']') {
            return -1;
        }
    }
    if (array.at >= array.end || *array.at++ != ']' || nodes->count == 0u) return -1;
    return 0;
}

static int parse_edges(json_cursor_t array, edge_list_t *edges)
{
    skip_space(&array);
    if (array.at >= array.end || *array.at++ != '[') return -1;
    skip_space(&array);
    while (array.at < array.end && *array.at != ']') {
        if (*array.at++ != '[') return -1;
        map_edge_t edge;
        if (read_id(&array, &edge.a) != 0) return -1;
        skip_space(&array);
        if (array.at >= array.end || *array.at++ != ',') return -1;
        if (read_id(&array, &edge.b) != 0 || edge.a == edge.b) return -1;
        skip_space(&array);
        if (array.at >= array.end || *array.at++ != ']') return -1;
        if (append_edge(edges, edge) != 0) return -1;
        skip_space(&array);
        if (array.at < array.end && *array.at == ',') {
            ++array.at;
            skip_space(&array);
            if (array.at < array.end && *array.at == ']') return -1;
        } else if (array.at >= array.end || *array.at != ']') {
            return -1;
        }
    }
    if (array.at >= array.end || *array.at++ != ']' || edges->count == 0u) return -1;
    return 0;
}

static const map_node_t *find_node(const node_list_t *nodes, int id)
{
    for (size_t i = 0u; i < nodes->count; ++i) {
        if (nodes->items[i].id == id) return &nodes->items[i];
    }
    return NULL;
}

static int read_map_string(json_cursor_t root, const char *key,
                           char *output, size_t capacity)
{
    json_cursor_t value;
    return object_field(root, key, &value) == 1 ? read_string(&value, output, capacity) : -1;
}

static int safe_identifier(const char *text)
{
    if (text[0] == '\0' || (!isalpha((unsigned char)text[0]) && text[0] != '_')) return 0;
    for (const unsigned char *at = (const unsigned char *)text; *at != '\0'; ++at) {
        if (!isalnum(*at) && *at != '_' && *at != '-') return 0;
    }
    return 1;
}

static int safe_version(const char *text)
{
    int dots = 0;
    bool digit_in_component = false;
    if (text[0] == '\0') return 0;
    for (const unsigned char *at = (const unsigned char *)text; *at != '\0'; ++at) {
        if (isdigit(*at)) {
            digit_in_component = true;
        } else if (*at == '.' && digit_in_component && dots < 2) {
            ++dots;
            digit_in_component = false;
        } else {
            return 0;
        }
    }
    return dots == 2 && digit_in_component;
}

static int fail(char *error, size_t capacity, const char *message)
{
    if (error != NULL && capacity > 0u) (void)snprintf(error, capacity, "%s", message);
    return -1;
}

static int parse_json_source(char *source, size_t length, nncar_map_t *map,
                             char *error, size_t error_capacity)
{
    memset(map, 0, sizeof(*map));

    json_cursor_t root = {source, source + length};
    json_cursor_t document = root;
    json_cursor_t value;
    double units = 0.0;
    double line_width = 0.02;
    node_list_t nodes = {0};
    edge_list_t edges = {0};
    int result = -1;

    if (skip_value(&document, 0u) != 0) {
        (void)fail(error, error_capacity, "map file is not valid JSON");
        goto done;
    }
    skip_space(&document);
    if (document.at != document.end) {
        (void)fail(error, error_capacity, "map file has trailing data");
        goto done;
    }

    if (read_map_string(root, "name", map->name, sizeof(map->name)) != 0 ||
        read_map_string(root, "map_version", map->version, sizeof(map->version)) != 0 ||
        !safe_identifier(map->name) || !safe_version(map->version) ||
        read_numeric_field(root, "units_per_meter", &units, true) != 0 || units <= 0.0 ||
        read_numeric_field(root, "line_width_m", &line_width, false) < 0 ||
        line_width <= 0.0 || line_width > 1.0) {
        (void)fail(error, error_capacity, "map name/version/scale/line width is invalid");
        goto done;
    }

    if (object_field(root, "nodes", &value) != 1 ||
        parse_nodes(value, 1.0 / units, &nodes) != 0 ||
        object_field(root, "edges", &value) != 1 ||
        parse_edges(value, &edges) != 0) {
        (void)fail(error, error_capacity, "map nodes or edges are invalid");
        goto done;
    }

    for (size_t i = 0u; i < nodes.count; ++i) {
        for (size_t j = i + 1u; j < nodes.count; ++j) {
            if (nodes.items[i].id == nodes.items[j].id) {
                (void)fail(error, error_capacity, "map node IDs must be unique");
                goto done;
            }
        }
    }

    map->segments = calloc(edges.count, sizeof(*map->segments));
    if (map->segments == NULL) {
        (void)fail(error, error_capacity, "cannot allocate map segments");
        goto done;
    }
    map->segment_count = edges.count;
    map->line_width_m = (float)line_width;
    map->x_min = map->y_min = INFINITY;
    map->x_max = map->y_max = -INFINITY;
    for (size_t i = 0u; i < nodes.count; ++i) {
        const map_node_t *node = &nodes.items[i];
        map->x_min = fminf(map->x_min, node->x);
        map->x_max = fmaxf(map->x_max, node->x);
        map->y_min = fminf(map->y_min, node->y);
        map->y_max = fmaxf(map->y_max, node->y);
    }
    for (size_t i = 0u; i < edges.count; ++i) {
        const map_node_t *a = find_node(&nodes, edges.items[i].a);
        const map_node_t *b = find_node(&nodes, edges.items[i].b);
        if (a == NULL || b == NULL || (a->x == b->x && a->y == b->y)) {
            (void)fail(error, error_capacity, "map edge references missing or duplicate nodes");
            goto done;
        }
        map->segments[i] = (nncar_map_seg_t){a->x, a->y, b->x, b->y};
    }

    if (object_field(root, "start", &value) != 1) {
        (void)fail(error, error_capacity, "map start is missing");
        goto done;
    }
    double start_id_number = 0.0;
    if (read_numeric_field(value, "node", &start_id_number, true) != 0 ||
        start_id_number < 0.0 || floor(start_id_number) != start_id_number ||
        start_id_number > 2147483647.0) {
        (void)fail(error, error_capacity, "map start node is invalid");
        goto done;
    }
    const map_node_t *start = find_node(&nodes, (int)start_id_number);
    if (start == NULL) {
        (void)fail(error, error_capacity, "map start node does not exist");
        goto done;
    }
    map->start_x = start->x;
    map->start_y = start->y;
    double toward_number = 0.0;
    json_cursor_t toward_value;
    int has_toward = object_field(value, "toward", &toward_value);
    if (has_toward == 1 && read_number(&toward_value, &toward_number) != 0) {
        has_toward = -1;
    }
    if (has_toward < 0) {
        (void)fail(error, error_capacity, "map start.toward is invalid");
        goto done;
    }
    if (has_toward == 1) {
        if (toward_number < 0.0 || floor(toward_number) != toward_number ||
            toward_number > 2147483647.0) {
            (void)fail(error, error_capacity, "map start.toward is invalid");
            goto done;
        }
        const map_node_t *toward = find_node(&nodes, (int)toward_number);
        if (toward == NULL) {
            (void)fail(error, error_capacity, "map start.toward node does not exist");
            goto done;
        }
        bool adjacent = false;
        for (size_t i = 0u; i < edges.count; ++i) {
            if ((edges.items[i].a == (int)start_id_number &&
                 edges.items[i].b == (int)toward_number) ||
                (edges.items[i].b == (int)start_id_number &&
                 edges.items[i].a == (int)toward_number)) {
                adjacent = true;
                break;
            }
        }
        if (!adjacent) {
            (void)fail(error, error_capacity, "map start.toward is not an adjacent node");
            goto done;
        }
        map->start_theta = atan2f(toward->y - start->y, toward->x - start->x);
    } else {
        double heading_deg = 0.0;
        json_cursor_t heading_value;
        int has_heading = object_field(value, "heading_deg", &heading_value);
        if (has_heading < 0 ||
            (has_heading == 1 && read_number(&heading_value, &heading_deg) != 0)) {
            (void)fail(error, error_capacity, "map start.heading_deg is invalid");
            goto done;
        }
        map->start_theta = (float)(heading_deg * NNCAR_PI / 180.0);
    }

    result = 0;

done:
    free(nodes.items);
    free(edges.items);
    free(source);
    if (result != 0) nncar_map_free(map);
    else if (error != NULL && error_capacity > 0u) error[0] = '\0';
    return result;
}

int nncar_map_load_json_text(const char *json, size_t json_length,
                             nncar_map_t *map, char *error,
                             size_t error_capacity)
{
    if (json == NULL || map == NULL) {
        return fail(error, error_capacity, "invalid map arguments");
    }
    memset(map, 0, sizeof(*map));
    if (json_length == 0u || json_length > 16u * 1024u * 1024u) {
        return fail(error, error_capacity, "map file size is invalid");
    }
    char *source = malloc(json_length + 1u);
    if (source == NULL) return fail(error, error_capacity, "cannot allocate map JSON");
    memcpy(source, json, json_length);
    source[json_length] = '\0';
    return parse_json_source(source, json_length, map, error, error_capacity);
}

int nncar_map_load_json(const char *path, nncar_map_t *map,
                        char *error, size_t error_capacity)
{
    if (path == NULL || map == NULL) return fail(error, error_capacity, "invalid map arguments");
    memset(map, 0, sizeof(*map));

    FILE *file = fopen(path, "rb");
    if (file == NULL) return fail(error, error_capacity, "cannot open map file");
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return fail(error, error_capacity, "cannot seek map file");
    }
    long length = ftell(file);
    if (length <= 0 || length > 16 * 1024 * 1024 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return fail(error, error_capacity, "map file size is invalid");
    }
    char *source = malloc((size_t)length + 1u);
    if (source == NULL || fread(source, 1u, (size_t)length, file) != (size_t)length) {
        free(source);
        fclose(file);
        return fail(error, error_capacity, "cannot read map file");
    }
    fclose(file);
    source[length] = '\0';
    return parse_json_source(source, (size_t)length, map, error, error_capacity);
}

void nncar_map_free(nncar_map_t *map)
{
    if (map == NULL) return;
    free(map->segments);
    memset(map, 0, sizeof(*map));
}
