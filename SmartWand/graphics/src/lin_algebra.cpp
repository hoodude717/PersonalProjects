#include "lin_algebra.h"

mat4 create_matrix_transform(vec3 translation) {
    mat4 matrix;

    matrix.entries[0]  = 1.0f;
    matrix.entries[1]  = 0.0f;
    matrix.entries[2]  = 0.0f;
    matrix.entries[3]  = 0.0f;

    matrix.entries[4]  = 0.0f;
    matrix.entries[5]  = 1.0f;
    matrix.entries[6]  = 0.0f;
    matrix.entries[7]  = 0.0f;

    matrix.entries[8]  = 0.0f;
    matrix.entries[9]  = 0.0f;
    matrix.entries[10] = 1.0f;
    matrix.entries[11] = 0.0f;

    matrix.entries[12] = translation.entries[0];
    matrix.entries[13] = translation.entries[1];
    matrix.entries[14] = translation.entries[2];
    matrix.entries[15] = 1.0f;

    return matrix;
}

mat4 create_z_rot(float angle) {

    angle = angle * PI / 180.0f;

    float c = cosf(angle);
    float s = sinf(angle);

    mat4 matrix;

    matrix.entries[0]  = c;
    matrix.entries[1]  = s;
    matrix.entries[2]  = 0.0f;
    matrix.entries[3]  = 0.0f;

    matrix.entries[4]  = -s;
    matrix.entries[5]  = c;
    matrix.entries[6]  = 0.0f;
    matrix.entries[7]  = 0.0f;

    matrix.entries[8]  = 0.0f;
    matrix.entries[9]  = 0.0f;
    matrix.entries[10] = 1.0f;
    matrix.entries[11] = 0.0f;

    matrix.entries[12] = 0.0f;
    matrix.entries[13] = 0.0f;
    matrix.entries[14] = 0.0f;
    matrix.entries[15] = 1.0f;

    return matrix;

}