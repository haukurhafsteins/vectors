#pragma once
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float x;
    float y;
    float z;
} vector3_t;

typedef enum {
    EULER_ORDER_XYZ,
    EULER_ORDER_XZY,
    EULER_ORDER_YXZ,
    EULER_ORDER_YZX,
    EULER_ORDER_ZXY,
    EULER_ORDER_ZYX,
} euler_order_t;

typedef struct {
    float x, y, z; // euler angles
    euler_order_t order;
} euler_t;

typedef struct vector2_t {
    float x;
    float y;
} vector2_t;

typedef struct vector2_array_t {
    vector2_t *array;
    size_t size;
    size_t idx;
} vector2_array_t;

float vector3_magnitude(const vector3_t *v);
float vector3_length(const vector3_t a);
float vector3_distance(const vector3_t a, const vector3_t b);
float vector3_dot(const vector3_t *v1, const vector3_t *v2);
float vector3_get_angle(vector3_t *euler_v1, vector3_t *euler_v2);
vector3_t vector3_sub(const vector3_t a, const vector3_t b);
// vector3_t vector3_apply_euler(vector3_t v, const euler_t e);
// vector3_t vector3_apply_quaternion( vector3_t v, const Quaternion q );
vector3_t vector3_multiply_scalar(vector3_t v, float s);
vector3_t vector3_add(vector3_t a, vector3_t b);

// Quaternion quaternion_set_from_euler( Quaternion q, const euler_t euler);

void vector2_array_new(vector2_array_t *v2, vector2_t *buf, size_t size);
void vector2_array_zero(vector2_array_t *v2);
void vector2_array_set_from_end(vector2_array_t *v2, vector2_t value, size_t idx);
void vector2_array_set(vector2_array_t *v2, vector2_t value, size_t idx);
vector2_t vector2_array_get_from_end(vector2_array_t *v2, size_t idx);
int vector2_array_push(vector2_array_t *v2, vector2_t v);
vector2_t vector2_array_get(vector2_array_t *v2, size_t idx);
vector2_t vector2_array_get_last(vector2_array_t *v2);
int vector2_get_idx_from_value(vector2_array_t *v2, vector2_t value);
int vector2_get_closest_idx_from_value(vector2_array_t *v2, vector2_t value);

#ifdef __cplusplus
}
#endif