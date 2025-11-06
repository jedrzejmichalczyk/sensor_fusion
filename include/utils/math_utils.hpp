#ifndef SENSOR_FUSION_MATH_UTILS_HPP
#define SENSOR_FUSION_MATH_UTILS_HPP

#include <array>
#include <cmath>

namespace sensor_fusion {

/**
 * @brief Mathematical utility functions
 */
class MathUtils {
public:
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double wrapToPi(double angle);

    /**
     * @brief Normalize angle to [0, 2*pi]
     */
    static double wrapTo2Pi(double angle);

    /**
     * @brief Convert degrees to radians
     */
    static double deg2rad(double deg) { return deg * M_PI / 180.0; }

    /**
     * @brief Convert radians to degrees
     */
    static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

    /**
     * @brief Compute vector norm
     */
    static double norm(const std::array<double, 3>& v);

    /**
     * @brief Normalize vector to unit length
     */
    static std::array<double, 3> normalize(const std::array<double, 3>& v);

    /**
     * @brief Dot product
     */
    static double dot(const std::array<double, 3>& a, const std::array<double, 3>& b);

    /**
     * @brief Cross product
     */
    static std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b);

    /**
     * @brief 3x3 matrix multiplication: C = A * B
     */
    static void matrixMultiply3x3(const double A[3][3], const double B[3][3], double C[3][3]);

    /**
     * @brief 3x3 matrix transpose
     */
    static void matrixTranspose3x3(const double A[3][3], double At[3][3]);

    /**
     * @brief 3x3 matrix times vector: y = A * x
     */
    static std::array<double, 3> matrixVectorMultiply3x3(const double A[3][3], const std::array<double, 3>& x);
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_MATH_UTILS_HPP
