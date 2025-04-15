#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vectors.h"

#define PI 3.14159265358979323846
#define DEG2RAD(deg) ((deg) * (PI / 180.0))
#define RAD2DEG(rad) ((rad) * (180.0 / PI))

float vector3_dot(const vector3_t *v1, const vector3_t *v2)
{
	return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

float vector3_magnitude(const vector3_t *v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

vector3_t vector3_sub(const vector3_t a, const vector3_t b)
{
	vector3_t result = {a.x - b.x, a.y - b.y, a.z - b.z};
	return result;
}

float vector3_length(const vector3_t a)
{
	return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

float vector3_distance(const vector3_t a, const vector3_t b)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf(dx * dx + dy * dy + dz * dz);
}

/// @brief Applies a quaternion rotation to a 3D vector v.
///	It calculates the result of rotating the vector v by the quaternion q.
/// This is done by performing the quaternion-vector multiplication:
/// v' = q * v * q^-1
/// @param v A 3D vector.
/// @param q The quaternion to rotate the vector by.
/// @return Rotated vector v'.
vector3_t vector3_apply_quaternion(vector3_t v, const quaternion_t q)
{

	const float x = v.x, y = v.y, z = v.z;
	const float qx = q.x, qy = q.y, qz = q.z, qw = q.w;

	// calculate quat * vector

	const float ix = qw * x + qy * z - qz * y;
	const float iy = qw * y + qz * x - qx * z;
	const float iz = qw * z + qx * y - qy * x;
	const float iw = -qx * x - qy * y - qz * z;

	// calculate result * inverse quat

	v.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
	v.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
	v.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

	return v;
}

vector3_t vector3_apply_euler(vector3_t v, euler_t e)
{
	quaternion_t q = quaternion_set_from_euler((quaternion_t){0}, e);
	v = vector3_apply_quaternion(v, q);
	return v;

	// float x = v.x, y = v.y, z = v.z;
	// float a = e.v.x, b = e.v.y, c = e.v.z;
	// float cosA = cosf(a), sinA = sinf(a);
	// float cosB = cosf(b), sinB = sinf(b);
	// float cosC = cosf(c), sinC = sinf(c);
	// float cosAcosB = cosA * cosB;
	// float sinAsinB = sinA * sinB;

	// v.x = cosC * cosAcosB * x - sinC * sinA * y + cosA * sinB * z;
	// v.y = sinC * cosAcosB * x + cosC * sinA * y + sinB * sinC * z;
	// v.z = -sinA * cosB * x + sinB * y + cosA * cosB * z;

	// return v;
}

vector3_t vector3_multiply_scalar(vector3_t v, float s)
{
	v.x *= s;
	v.y *= s;
	v.z *= s;
	return v;
}

vector3_t vector3_add(vector3_t a, vector3_t b)
{
	vector3_t result = {a.x + b.x, a.y + b.y, a.z + b.z};
	return result;
}

quaternion_t quaternion_set_from_euler(quaternion_t q, euler_t euler)
{

	const float x = euler.x, y = euler.y, z = euler.z;
	const euler_order_t order = euler.order;

	// http://www.mathworks.com/matlabcentral/fileexchange/
	// 	20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/
	//	content/SpinCalc.m

	// const cos = Math.cos;
	// const sin = Math.sin;

	const float c1 = cosf(x * 0.5);
	const float c2 = cosf(y * 0.5);
	const float c3 = cosf(z * 0.5);

	const float s1 = sinf(x * 0.5);
	const float s2 = sinf(y * 0.5);
	const float s3 = sinf(z * 0.5);

	switch (order)
	{

	case EULER_ORDER_XYZ: // 'XYZ'
		q.x = s1 * c2 * c3 + c1 * s2 * s3;
		q.y = c1 * s2 * c3 - s1 * c2 * s3;
		q.z = c1 * c2 * s3 + s1 * s2 * c3;
		q.w = c1 * c2 * c3 - s1 * s2 * s3;
		break;

	case EULER_ORDER_YXZ: //'YXZ'
		q.x = s1 * c2 * c3 + c1 * s2 * s3;
		q.y = c1 * s2 * c3 - s1 * c2 * s3;
		q.z = c1 * c2 * s3 - s1 * s2 * c3;
		q.w = c1 * c2 * c3 + s1 * s2 * s3;
		break;

	case EULER_ORDER_ZXY: //'ZXY'
		q.x = s1 * c2 * c3 - c1 * s2 * s3;
		q.y = c1 * s2 * c3 + s1 * c2 * s3;
		q.z = c1 * c2 * s3 + s1 * s2 * c3;
		q.w = c1 * c2 * c3 - s1 * s2 * s3;
		break;

	case EULER_ORDER_ZYX: //'ZYX'
		q.x = s1 * c2 * c3 - c1 * s2 * s3;
		q.y = c1 * s2 * c3 + s1 * c2 * s3;
		q.z = c1 * c2 * s3 - s1 * s2 * c3;
		q.w = c1 * c2 * c3 + s1 * s2 * s3;
		break;

	case EULER_ORDER_YZX: //'YZX'
		q.x = s1 * c2 * c3 + c1 * s2 * s3;
		q.y = c1 * s2 * c3 + s1 * c2 * s3;
		q.z = c1 * c2 * s3 - s1 * s2 * c3;
		q.w = c1 * c2 * c3 - s1 * s2 * s3;
		break;

	case EULER_ORDER_XZY: //'XZY'
		q.x = s1 * c2 * c3 - c1 * s2 * s3;
		q.y = c1 * s2 * c3 - s1 * c2 * s3;
		q.z = c1 * c2 * s3 + s1 * s2 * c3;
		q.w = c1 * c2 * c3 + s1 * s2 * s3;
		break;

	default:
		printf("quaternion_set_from_euler encountered an unknown order: %d", order);
		break;
	}

	return q;
}

// Function to convert Euler angles to a direction vector
vector3_t euler_to_direction_vector(vector3_t *euler_angles) {
    vector3_t dir;
    float cos_pitch, sin_pitch, cos_yaw, sin_yaw;
    
    // Assuming the angles are in degrees, convert them to radians
    float yaw = DEG2RAD(euler_angles->z);   // Rotation around the z-axis
    float pitch = DEG2RAD(euler_angles->y); // Rotation around the y-axis
    // float roll = DEG2RAD(euler_angles.x); // Roll is not used for a direction vector
    
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);
    cos_yaw = cos(yaw);
    sin_yaw = sin(yaw);

    dir.x = cos_pitch * cos_yaw;
    dir.y = cos_pitch * sin_yaw;
    dir.z = sin_pitch;

    return dir;
}

// Function to calculate the angle between two vectors given in Euler angles
float vector3_get_angle(vector3_t *euler_v1, vector3_t *euler_v2) {
    vector3_t v1 = euler_to_direction_vector(euler_v1);
    vector3_t v2 = euler_to_direction_vector(euler_v2);

    float dot = vector3_dot(&v1, &v2);
    float magnitude_v1 = vector3_magnitude(&v1);
    float magnitude_v2 = vector3_magnitude(&v2);

    // Calculate the cosine of the angle
    float cos_angle = dot / (magnitude_v1 * magnitude_v2);
    // Ensure the cosine value is in the range [-1, 1]
    cos_angle = fmax(fmin(cos_angle, 1.0f), -1.0f);

    // Calculate the angle in radians and then convert to degrees
    float angle_rad = acos(cos_angle);
    float angle_deg = RAD2DEG(angle_rad);

    return angle_deg;
}

// setFromRotationMatrix( m, order = this._order, update = true ) {

// 		// assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)

// 		const te = m.elements;
// 		const m11 = te[ 0 ], m12 = te[ 4 ], m13 = te[ 8 ];
// 		const m21 = te[ 1 ], m22 = te[ 5 ], m23 = te[ 9 ];
// 		const m31 = te[ 2 ], m32 = te[ 6 ], m33 = te[ 10 ];

// 		switch ( order ) {

// 			case 'XYZ':

// 				q.y = Math.asin( clamp( m13, - 1, 1 ) );

// 				if ( Math.abs( m13 ) < 0.9999999 ) {

// 					q.x = Math.atan2( - m23, m33 );
// 					q.z = Math.atan2( - m12, m11 );

// 				} else {

// 					q.x = Math.atan2( m32, m22 );
// 					q.z = 0;

// 				}

// 				break;

// 			case 'YXZ':

// 				q.x = Math.asin( - clamp( m23, - 1, 1 ) );

// 				if ( Math.abs( m23 ) < 0.9999999 ) {

// 					q.y = Math.atan2( m13, m33 );
// 					q.z = Math.atan2( m21, m22 );

// 				} else {

// 					q.y = Math.atan2( - m31, m11 );
// 					q.z = 0;

// 				}

// 				break;

// 			case 'ZXY':

// 				q.x = Math.asin( clamp( m32, - 1, 1 ) );

// 				if ( Math.abs( m32 ) < 0.9999999 ) {

// 					q.y = Math.atan2( - m31, m33 );
// 					q.z = Math.atan2( - m12, m22 );

// 				} else {

// 					q.y = 0;
// 					q.z = Math.atan2( m21, m11 );

// 				}

// 				break;

// 			case 'ZYX':

// 				q.y = Math.asin( - clamp( m31, - 1, 1 ) );

// 				if ( Math.abs( m31 ) < 0.9999999 ) {

// 					q.x = Math.atan2( m32, m33 );
// 					q.z = Math.atan2( m21, m11 );

// 				} else {

// 					q.x = 0;
// 					q.z = Math.atan2( - m12, m22 );

// 				}

// 				break;

// 			case 'YZX':

// 				q.z = Math.asin( clamp( m21, - 1, 1 ) );

// 				if ( Math.abs( m21 ) < 0.9999999 ) {

// 					q.x = Math.atan2( - m23, m22 );
// 					q.y = Math.atan2( - m31, m11 );

// 				} else {

// 					q.x = 0;
// 					q.y = Math.atan2( m13, m33 );

// 				}

// 				break;

// 			case 'XZY':

// 				q.z = Math.asin( - clamp( m12, - 1, 1 ) );

// 				if ( Math.abs( m12 ) < 0.9999999 ) {

// 					q.x = Math.atan2( m32, m22 );
// 					q.y = Math.atan2( m13, m11 );

// 				} else {

// 					q.x = Math.atan2( - m23, m33 );
// 					q.y = 0;

// 				}

// 				break;

// 			default:

// 				console.warn( 'THREE.Euler: .setFromRotationMatrix() encountered an unknown order: ' + order );

// 		}

// 		this._order = order;

// 		if ( update === true ) this._onChangeCallback();

// 		return this;

// 	}


void vector2_array_new(vector2_array_t *v2, vector2_t *buf, size_t size)
{
    v2->array = buf;
    v2->size = size;
    v2->idx = 0;
}

void vector2_array_zero(vector2_array_t *v2)
{
	memset(v2->array, 0, v2->size * sizeof(vector2_t));
    v2->idx = 0;
}

void vector2_array_set_from_end(vector2_array_t *v2, vector2_t value, size_t idx)
{
    v2->array[(v2->idx - 1 - idx) % v2->size] = value;
}

vector2_t vector2_array_get_from_end(vector2_array_t *v2, size_t idx)
{
    return v2->array[(v2->idx - 1 - idx) % v2->size];
}

int vector2_array_push(vector2_array_t *v2, vector2_t v)
{
	int idx = v2->idx;
    v2->array[v2->idx] = v;
    v2->idx = (v2->idx + 1) % v2->size;
	return idx;
}

vector2_t vector2_array_get(vector2_array_t *v2, size_t idx)
{
    return v2->array[idx % v2->size];
} 

void vector2_array_set(vector2_array_t *v2, vector2_t value, size_t idx)
{
	v2->array[idx % v2->size] = value;
}

vector2_t vector2_array_get_last(vector2_array_t *v2)
{
    return vector2_array_get(v2, v2->idx - 1);
}

int vector2_get_idx_from_value(vector2_array_t *v2, vector2_t value)
{
	for (int i = 0; i < v2->size; i++)
	{
		if (v2->array[i].x == value.x && v2->array[i].y == value.y)
		{
			return i;
		}
	}
	return -1;
}

int vector2_get_closest_idx_from_value(vector2_array_t *v2, vector2_t value)
{
	float min_dist = 1000000;
	int min_idx = -1;
	for (int i = 0; i < v2->size; i++)
	{
		float dist = vector3_distance((vector3_t){v2->array[i].x, v2->array[i].y, 0}, (vector3_t){value.x, value.y, 0});
		if (dist < min_dist)
		{
			min_dist = dist;
			min_idx = i;
		}
	}
	return min_idx;
}
