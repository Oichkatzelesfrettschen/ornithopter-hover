#include <unity.h>
#include "../src/math/quaternion.h"
#include <math.h>

// Helper function to compare floats with tolerance
void assertFloatEquals(float expected, float actual, float tolerance = 0.001f) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
}

void setUp(void) {
    // Set up before each test
}

void tearDown(void) {
    // Clean up after each test
}

void test_quaternion_identity(void) {
    Quaternion q;
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.z);
}

void test_quaternion_multiplication(void) {
    // Identity * any = any
    Quaternion identity(1, 0, 0, 0);
    Quaternion q(0.707f, 0.707f, 0, 0); // 90° roll
    
    Quaternion result = identity * q;
    
    assertFloatEquals(0.707f, result.w);
    assertFloatEquals(0.707f, result.x);
    assertFloatEquals(0.0f, result.y);
    assertFloatEquals(0.0f, result.z);
}

void test_quaternion_conjugate(void) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion conj = q.conjugate();
    
    TEST_ASSERT_EQUAL_FLOAT(0.5f, conj.w);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, conj.x);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, conj.y);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, conj.z);
}

void test_quaternion_norm(void) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    float n = q.norm();
    assertFloatEquals(1.0f, n);
}

void test_quaternion_normalization(void) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    q.normalize();
    
    float norm = q.norm();
    assertFloatEquals(1.0f, norm);
}

void test_quaternion_from_axis_angle(void) {
    // 90° rotation around X axis
    Quaternion q = Quaternion::fromAxisAngle(M_PI / 2.0f, 1.0f, 0.0f, 0.0f);
    
    assertFloatEquals(0.707f, q.w, 0.01f);
    assertFloatEquals(0.707f, q.x, 0.01f);
    assertFloatEquals(0.0f, q.y, 0.01f);
    assertFloatEquals(0.0f, q.z, 0.01f);
}

void test_quaternion_from_euler(void) {
    // 90° roll
    Quaternion q = Quaternion::fromEuler(M_PI / 2.0f, 0.0f, 0.0f);
    
    assertFloatEquals(0.707f, q.w, 0.01f);
    assertFloatEquals(0.707f, q.x, 0.01f);
    assertFloatEquals(0.0f, q.y, 0.01f);
    assertFloatEquals(0.0f, q.z, 0.01f);
}

void test_quaternion_to_euler(void) {
    float roll = M_PI / 4.0f;   // 45°
    float pitch = M_PI / 6.0f;  // 30°
    float yaw = M_PI / 3.0f;    // 60°
    
    Quaternion q = Quaternion::fromEuler(roll, pitch, yaw);
    
    float r, p, y;
    q.toEuler(r, p, y);
    
    assertFloatEquals(roll, r, 0.01f);
    assertFloatEquals(pitch, p, 0.01f);
    assertFloatEquals(yaw, y, 0.01f);
}

void test_quaternion_rotate_vector(void) {
    // 90° rotation around Z axis
    Quaternion q = Quaternion::fromAxisAngle(M_PI / 2.0f, 0.0f, 0.0f, 1.0f);
    
    // Rotate vector [1, 0, 0] should give [0, 1, 0]
    float rx, ry, rz;
    q.rotateVector(1.0f, 0.0f, 0.0f, rx, ry, rz);
    
    assertFloatEquals(0.0f, rx, 0.01f);
    assertFloatEquals(1.0f, ry, 0.01f);
    assertFloatEquals(0.0f, rz, 0.01f);
}

void test_quaternion_integration(void) {
    Quaternion q; // Identity
    
    // Constant angular velocity around X: 1 rad/s
    float wx = 1.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    float dt = 0.01f; // 10ms
    
    // Integrate for 1 second
    for (int i = 0; i < 100; i++) {
        q.integrate(wx, wy, wz, dt);
    }
    
    // After 1 second at 1 rad/s, should have rotated ~57.3°
    float roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);
    
    assertFloatEquals(1.0f, roll, 0.1f); // 1 radian
    assertFloatEquals(0.0f, pitch, 0.01f);
    assertFloatEquals(0.0f, yaw, 0.01f);
    
    // Quaternion should still be normalized
    assertFloatEquals(1.0f, q.norm(), 0.01f);
}

void test_quaternion_inverse_property(void) {
    // q * q^-1 = identity
    Quaternion q = Quaternion::fromEuler(0.5f, 0.3f, 0.7f);
    Quaternion q_inv = q.conjugate();
    
    Quaternion identity = q * q_inv;
    
    assertFloatEquals(1.0f, identity.w, 0.01f);
    assertFloatEquals(0.0f, identity.x, 0.01f);
    assertFloatEquals(0.0f, identity.y, 0.01f);
    assertFloatEquals(0.0f, identity.z, 0.01f);
}

void test_quaternion_rotation_matrix(void) {
    // 90° rotation around Z axis
    Quaternion q = Quaternion::fromAxisAngle(M_PI / 2.0f, 0.0f, 0.0f, 1.0f);
    
    float R[3][3];
    q.toRotationMatrix(R);
    
    // Expected rotation matrix for 90° around Z:
    // [0 -1  0]
    // [1  0  0]
    // [0  0  1]
    
    assertFloatEquals(0.0f, R[0][0], 0.01f);
    assertFloatEquals(-1.0f, R[0][1], 0.01f);
    assertFloatEquals(0.0f, R[0][2], 0.01f);
    
    assertFloatEquals(1.0f, R[1][0], 0.01f);
    assertFloatEquals(0.0f, R[1][1], 0.01f);
    assertFloatEquals(0.0f, R[1][2], 0.01f);
    
    assertFloatEquals(0.0f, R[2][0], 0.01f);
    assertFloatEquals(0.0f, R[2][1], 0.01f);
    assertFloatEquals(1.0f, R[2][2], 0.01f);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    RUN_TEST(test_quaternion_identity);
    RUN_TEST(test_quaternion_multiplication);
    RUN_TEST(test_quaternion_conjugate);
    RUN_TEST(test_quaternion_norm);
    RUN_TEST(test_quaternion_normalization);
    RUN_TEST(test_quaternion_from_axis_angle);
    RUN_TEST(test_quaternion_from_euler);
    RUN_TEST(test_quaternion_to_euler);
    RUN_TEST(test_quaternion_rotate_vector);
    RUN_TEST(test_quaternion_integration);
    RUN_TEST(test_quaternion_inverse_property);
    RUN_TEST(test_quaternion_rotation_matrix);
    
    return UNITY_END();
}
