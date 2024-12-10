#pragma once
#include <GL/glut.h>
#include <cmath>

// 3D Vector Structure
struct Vec3 {
    double x, y, z;

    // Vector subtraction
    Vec3 operator-(const Vec3& other) const {
        return Vec3{ x - other.x, y - other.y, z - other.z };
    }

    // Vector addition
    Vec3 operator+(const Vec3& other) const {
        return Vec3{ x + other.x, y + other.y, z + other.z };
    }

    // Scalar multiplication
    Vec3 operator*(double scalar) const {
        return Vec3{ x * scalar, y * scalar, z * scalar };
    }

    // Dot product
    double Dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Magnitude
    double Magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Normalization
    Vec3 Normalized() const {
        double mag = Magnitude();
        return Vec3{ x / mag, y / mag, z / mag };
    }
   
};

// Quaternion Structure
struct Quaternion {
    double w, x, y, z;

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion{
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }
};

// Normalize a quaternion
Quaternion quatNormalize(const Quaternion& q);

// Create a quaternion from Euler angles (in radians)
Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

// Multiply two quaternions
Quaternion quatMultiply(const Quaternion& q1, const Quaternion& q2);

// Integrate quaternion orientation given an angular velocity over a time step
Quaternion quatIntegrate(const Quaternion& q, const Vec3& w, double dt);

// Convert a quaternion to a 4x4 rotation matrix (column-major order)
void quaternionToMatrix(const Quaternion& q, float* M);

// Build a complete 4x4 transform matrix from quaternion (rotation) and a position vector (translation)
void buildTransformMatrix(const Quaternion& q, const Vec3& t, float* M);



