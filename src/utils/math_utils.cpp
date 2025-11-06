#include "utils/math_utils.hpp"
#include <cmath>

namespace sensor_fusion {

double MathUtils::wrapToPi(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

double MathUtils::wrapTo2Pi(double angle)
{
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle < 0.0)
        angle += 2.0 * M_PI;
    return angle;
}

double MathUtils::norm(const std::array<double, 3>& v)
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

std::array<double, 3> MathUtils::normalize(const std::array<double, 3>& v)
{
    double n = norm(v);
    if (n < 1e-9) {
        return {0.0, 0.0, 0.0};
    }
    return {v[0]/n, v[1]/n, v[2]/n};
}

double MathUtils::dot(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

std::array<double, 3> MathUtils::cross(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

void MathUtils::matrixMultiply3x3(const double A[3][3], const double B[3][3], double C[3][3])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void MathUtils::matrixTranspose3x3(const double A[3][3], double At[3][3])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            At[i][j] = A[j][i];
        }
    }
}

std::array<double, 3> MathUtils::matrixVectorMultiply3x3(const double A[3][3], const std::array<double, 3>& x)
{
    std::array<double, 3> y;
    for (int i = 0; i < 3; ++i) {
        y[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
    return y;
}

} // namespace sensor_fusion
