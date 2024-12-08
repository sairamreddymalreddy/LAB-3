#include "stdafx.h"
#include<math.h>
#include<cmath>
#include "Quaternion.h"

Quaternion quatNormalize(Quaternion q) {
    double mag = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= mag; q.x /= mag; q.y /= mag; q.z /= mag;
    return q;
}

Quaternion quatFromAxisAngle(Vec3 axis, double angle) {
    double half = angle * 0.5;
    double s = std::sin(half);
    Quaternion q;
    q.w = std::cos(half);
    q.x = axis.x * s;
    q.y = axis.y * s;
    q.z = axis.z * s;
    return quatNormalize(q);
}

Quaternion quatMultiply(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q;
    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return q;
}

Quaternion quatIntegrate(Quaternion q, Vec3 w, double dt) {
    Quaternion Wq = { 0.0, w.x, w.y, w.z };

    Quaternion dq;
    dq.w = 0.5 * (Wq.w * q.w - Wq.x * q.x - Wq.y * q.y - Wq.z * q.z);
    dq.x = 0.5 * (Wq.w * q.x + Wq.x * q.w + Wq.y * q.z - Wq.z * q.y);
    dq.y = 0.5 * (Wq.w * q.y - Wq.x * q.z + Wq.y * q.w + Wq.z * q.x);
    dq.z = 0.5 * (Wq.w * q.z + Wq.x * q.y - Wq.y * q.x + Wq.z * q.w);

    q.w += dq.w * dt;
    q.x += dq.x * dt;
    q.y += dq.y * dt;
    q.z += dq.z * dt;
    return quatNormalize(q);
}

void quaternionToMatrix(const Quaternion& q, float* M) {
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

    M[0] = (float)(1.0 - (yy + zz));
    M[1] = (float)(xy + wz);
    M[2] = (float)(xz - wy);
    M[3] = 0.0f;

    M[4] = (float)(xy - wz);
    M[5] = (float)(1.0 - (xx + zz));
    M[6] = (float)(yz + wx);
    M[7] = 0.0f;

    M[8] = (float)(xz + wy);
    M[9] = (float)(yz - wx);
    M[10] = (float)(1.0 - (xx + yy));
    M[11] = 0.0f;

    M[12] = 0.0f;
    M[13] = 0.0f;
    M[14] = 0.0f;
    M[15] = 1.0f;
}

void buildTransformMatrix(const Quaternion& q, const Vec3& t, float* M) {
    quaternionToMatrix(q, M);
    // Insert translation
    M[12] = (float)t.x;
    M[13] = (float)t.y;
    M[14] = (float)t.z;
}
