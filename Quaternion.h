#pragma once
#include <GL/glut.h>
#include <cmath>

struct Vec3 {
    double x, y, z;
};

struct Quaternion {
    double w, x, y, z;
};

Quaternion quatNormalize(Quaternion q);
Quaternion quatFromAxisAngle(Vec3 axis, double angle);
Quaternion quatMultiply(const Quaternion& q1, const Quaternion& q2);
Quaternion quatIntegrate(Quaternion q, Vec3 w, double dt);

// Convert a quaternion to a 4x4 rotation matrix (no translation)
void quaternionToMatrix(const Quaternion& q, float* M);

// Build a complete 4x4 transform matrix from quaternion (rotation) and a position vector (translation).
void buildTransformMatrix(const Quaternion& q, const Vec3& t, float* M);


