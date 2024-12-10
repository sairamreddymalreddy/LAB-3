#include "stdafx.h"
#include<math.h>
#include<cmath>
#include "Quaternion.h"

// Normalize a quaternion
Quaternion quatNormalize(const Quaternion& q) {
    double mag = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return Quaternion{ q.w / mag, q.x / mag, q.y / mag, q.z / mag };
    
}

// Create a quaternion from Euler angles (in radians)
Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return quatNormalize(q);
}

// Multiply two quaternions
Quaternion quatMultiply(const Quaternion& q1, const Quaternion& q2) {
    return q1 * q2;
}

// Integrate quaternion orientation given an angular velocity over a time step
Quaternion quatIntegrate(const Quaternion& q, const Vec3& w, double dt) {
    // Convert angular velocity to quaternion (0, wx, wy, wz)
    Quaternion Wq = { 0.0, w.x, w.y, w.z };

    // dq/dt = 0.5 * q * Wq
    Quaternion dq;
    dq.w = 0.5 * (q.x * Wq.x + q.y * Wq.y + q.z * Wq.z);
    dq.x = 0.5 * (q.w * Wq.x + q.y * Wq.z - q.z * Wq.y);
    dq.y = 0.5 * (q.w * Wq.y - q.x * Wq.z + q.z * Wq.x);
    dq.z = 0.5 * (q.w * Wq.z + q.x * Wq.y - q.y * Wq.x);

    // Update quaternion
    Quaternion newQ;
    newQ.w = q.w + dq.w * dt;
    newQ.x = q.x + dq.x * dt;
    newQ.y = q.y + dq.y * dt;
    newQ.z = q.z + dq.z * dt;

    return quatNormalize(newQ);
}

// Convert a quaternion to a 4x4 rotation matrix (column-major order)
void quaternionToMatrix(const Quaternion& q, GLfloat* rotation) {
    double x2 = q.x + q.x;
    double y2 = q.y + q.y;
    double z2 = q.z + q.z;

    double xx = q.x * x2;
    double xy = q.x * y2;
    double xz = q.x * z2;
    double yy = q.y * y2;
    double yz = q.y * z2;
    double zz = q.z * z2;
    double wx = q.w * x2;
    double wy = q.w * y2;
    double wz = q.w * z2;

    rotation[0] = (1.0 - (yy + zz));
    rotation[1] = (xy + wz);
    rotation[2] = (xz - wy);
    rotation[3] = 0.0f;

    rotation[4] = (xy - wz);
    rotation[5] = (1.0 - (xx + zz));
    rotation[6] = (yz + wx);
    rotation[7] = 0.0f;

    rotation[8] = (xz + wy);
    rotation[9] = (yz - wx);
    rotation[10] = (1.0 - (xx + yy));
    rotation[11] = 0.0f;

    rotation[12] = 0.0f;
    rotation[13] = 0.0f;
    rotation[14] = 0.0f;
    rotation[15] = 1.0f;
}

// Build a complete 4x4 transform matrix from quaternion (rotation) and a position vector (translation)
void buildTransformMatrix(const Quaternion& q, const Vec3& t, GLfloat* rotation) {
    quaternionToMatrix(q, rotation);
    // Insert translation
    rotation[12] = (t.x);
    rotation[13] = (t.y);
    rotation[14] = (t.z);
}

