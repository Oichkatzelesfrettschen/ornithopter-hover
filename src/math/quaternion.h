#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

/**
 * @brief Quaternion class for rotation representation
 * 
 * Implements quaternion mathematics for singularity-free 3D rotations.
 * Provides better numerical stability than Euler angles and avoids gimbal lock.
 */
class Quaternion {
public:
    float w, x, y, z;
    
    // Constructors
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    
    /**
     * @brief Create quaternion from axis-angle representation
     * @param angle Rotation angle in radians
     * @param axis_x X component of rotation axis (must be normalized)
     * @param axis_y Y component of rotation axis
     * @param axis_z Z component of rotation axis
     */
    static Quaternion fromAxisAngle(float angle, float axis_x, float axis_y, float axis_z) {
        float half_angle = angle * 0.5f;
        float s = sin(half_angle);
        return Quaternion(cos(half_angle), axis_x * s, axis_y * s, axis_z * s);
    }
    
    /**
     * @brief Create quaternion from Euler angles (ZYX convention)
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     */
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        
        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
    
    /**
     * @brief Quaternion multiplication (composition of rotations)
     * @param q Other quaternion
     * @return Composed rotation q1 * q2
     */
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
    
    /**
     * @brief Quaternion conjugate (inverse rotation for unit quaternion)
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    /**
     * @brief Compute quaternion magnitude
     */
    float norm() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }
    
    /**
     * @brief Normalize quaternion to unit length
     * CRITICAL: Must be called after integration to prevent drift
     */
    void normalize() {
        float n = norm();
        if (n > 1e-6f) {  // Avoid division by zero
            float inv_n = 1.0f / n;
            w *= inv_n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;
        } else {
            // Reset to identity if degenerate
            w = 1.0f;
            x = y = z = 0.0f;
        }
    }
    
    /**
     * @brief Rotate a 3D vector using this quaternion
     * @param vx, vy, vz Components of vector to rotate
     * @param rx, ry, rz Output: rotated vector components
     */
    void rotateVector(float vx, float vy, float vz, float& rx, float& ry, float& rz) const {
        // v' = q * [0, v] * q*
        // Optimized formula avoiding quaternion multiplication
        float qw2 = w * w;
        float qx2 = x * x;
        float qy2 = y * y;
        float qz2 = z * z;
        
        rx = vx * (qw2 + qx2 - qy2 - qz2) + 
             vy * 2.0f * (x * y - w * z) + 
             vz * 2.0f * (x * z + w * y);
        
        ry = vx * 2.0f * (x * y + w * z) + 
             vy * (qw2 - qx2 + qy2 - qz2) + 
             vz * 2.0f * (y * z - w * x);
        
        rz = vx * 2.0f * (x * z - w * y) + 
             vy * 2.0f * (y * z + w * x) + 
             vz * (qw2 - qx2 - qy2 + qz2);
    }
    
    /**
     * @brief Convert quaternion to rotation matrix
     * @param R Output 3x3 rotation matrix (row-major order)
     */
    void toRotationMatrix(float R[3][3]) const {
        float qw2 = w * w;
        float qx2 = x * x;
        float qy2 = y * y;
        float qz2 = z * z;
        
        R[0][0] = qw2 + qx2 - qy2 - qz2;
        R[0][1] = 2.0f * (x * y - w * z);
        R[0][2] = 2.0f * (x * z + w * y);
        
        R[1][0] = 2.0f * (x * y + w * z);
        R[1][1] = qw2 - qx2 + qy2 - qz2;
        R[1][2] = 2.0f * (y * z - w * x);
        
        R[2][0] = 2.0f * (x * z - w * y);
        R[2][1] = 2.0f * (y * z + w * x);
        R[2][2] = qw2 - qx2 - qy2 + qz2;
    }
    
    /**
     * @brief Convert quaternion to Euler angles (ZYX convention)
     * @param roll Output: roll angle in radians
     * @param pitch Output: pitch angle in radians
     * @param yaw Output: yaw angle in radians
     */
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (fabs(sinp) >= 1.0f)
            pitch = copysign(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
        else
            pitch = asin(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
    }
    
    /**
     * @brief Integrate angular velocity using 4th order Runge-Kutta
     * @param wx, wy, wz Angular velocity in body frame (rad/s)
     * @param dt Time step (seconds)
     */
    void integrate(float wx, float wy, float wz, float dt) {
        // q_dot = 0.5 * q * [0, w]
        // RK4 integration for accuracy
        
        // k1
        float k1w = 0.5f * (-x * wx - y * wy - z * wz);
        float k1x = 0.5f * (w * wx + y * wz - z * wy);
        float k1y = 0.5f * (w * wy - x * wz + z * wx);
        float k1z = 0.5f * (w * wz + x * wy - y * wx);
        
        // k2 (midpoint)
        float q2w = w + 0.5f * dt * k1w;
        float q2x = x + 0.5f * dt * k1x;
        float q2y = y + 0.5f * dt * k1y;
        float q2z = z + 0.5f * dt * k1z;
        
        float k2w = 0.5f * (-q2x * wx - q2y * wy - q2z * wz);
        float k2x = 0.5f * (q2w * wx + q2y * wz - q2z * wy);
        float k2y = 0.5f * (q2w * wy - q2x * wz + q2z * wx);
        float k2z = 0.5f * (q2w * wz + q2x * wy - q2y * wx);
        
        // k3 (midpoint)
        float q3w = w + 0.5f * dt * k2w;
        float q3x = x + 0.5f * dt * k2x;
        float q3y = y + 0.5f * dt * k2y;
        float q3z = z + 0.5f * dt * k2z;
        
        float k3w = 0.5f * (-q3x * wx - q3y * wy - q3z * wz);
        float k3x = 0.5f * (q3w * wx + q3y * wz - q3z * wy);
        float k3y = 0.5f * (q3w * wy - q3x * wz + q3z * wx);
        float k3z = 0.5f * (q3w * wz + q3x * wy - q3y * wx);
        
        // k4 (endpoint)
        float q4w = w + dt * k3w;
        float q4x = x + dt * k3x;
        float q4y = y + dt * k3y;
        float q4z = z + dt * k3z;
        
        float k4w = 0.5f * (-q4x * wx - q4y * wy - q4z * wz);
        float k4x = 0.5f * (q4w * wx + q4y * wz - q4z * wy);
        float k4y = 0.5f * (q4w * wy - q4x * wz + q4z * wx);
        float k4z = 0.5f * (q4w * wz + q4x * wy - q4y * wx);
        
        // Final update
        w += (dt / 6.0f) * (k1w + 2.0f * k2w + 2.0f * k3w + k4w);
        x += (dt / 6.0f) * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
        y += (dt / 6.0f) * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
        z += (dt / 6.0f) * (k1z + 2.0f * k2z + 2.0f * k3z + k4z);
        
        // Normalize to prevent drift
        normalize();
    }
};

#endif // QUATERNION_H
