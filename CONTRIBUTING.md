# Contributing to Sensor Fusion Navigation

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Development Workflow

### 1. Fork and Clone

```bash
git clone https://github.com/yourusername/sensor_fusion.git
cd sensor_fusion
git remote add upstream https://github.com/jedrzejmichalczyk/sensor_fusion.git
```

### 2. Create a Feature Branch

```bash
git checkout -b feature/your-feature-name
```

### 3. Make Changes

- Follow the code style (use `clang-format`)
- Write unit tests for new functionality
- Update documentation as needed

### 4. Test Your Changes

```bash
mkdir build && cd build
cmake ..
make
ctest
```

### 5. Commit

Use clear, descriptive commit messages:

```
Add ZUPT implementation for stationary detection

- Implement zero-velocity update algorithm
- Add unit tests for ZUPT detector
- Update documentation with ZUPT usage
```

### 6. Push and Create Pull Request

```bash
git push origin feature/your-feature-name
```

Then create a Pull Request on GitHub.

## Code Style

We use `clang-format` for consistent code formatting:

```bash
# Format all C++ files
find src include tests examples -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
```

### Guidelines

- **C++ Standard**: C++17
- **Indentation**: 4 spaces (no tabs)
- **Line Length**: 100 characters max
- **Naming**:
  - Classes: `PascalCase` (e.g., `ExtendedKalmanFilter`)
  - Functions: `camelCase` (e.g., `updateGPS()`)
  - Variables: `snake_case` (e.g., `gps_pos_std_`)
  - Constants: `UPPER_CASE` (e.g., `GRAVITY`)
- **Comments**: Use Doxygen-style comments for public APIs

## Testing

All new features must include tests:

```cpp
// tests/test_new_feature.cpp
#include <iostream>
#include "your_feature.hpp"

int main() {
    // Test your feature
    bool test_ok = true;

    // Add assertions

    std::cout << "Test " << (test_ok ? "PASSED" : "FAILED") << "\n";
    return test_ok ? 0 : 1;
}
```

Add to `tests/CMakeLists.txt`:

```cmake
add_executable(test_new_feature test_new_feature.cpp)
target_link_libraries(test_new_feature sensor_fusion)
add_test(NAME test_new_feature COMMAND test_new_feature)
```

## Documentation

### Code Documentation

Use Doxygen comments for all public APIs:

```cpp
/**
 * @brief Brief description of function
 *
 * Detailed description of what the function does,
 * including any important notes.
 *
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 */
void myFunction(int param1, double param2);
```

### LaTeX Documentation

If modifying the technical documentation:

```bash
cd doc
make
# Review sensor_fusion_documentation.pdf
```

## Pull Request Process

1. **Update tests**: Ensure all tests pass
2. **Update docs**: Add/update documentation
3. **CI checks**: Wait for all CI checks to pass
4. **Review**: Address any feedback from reviewers
5. **Merge**: Maintainer will merge once approved

### CI Checks

Your PR will be automatically tested for:

- ✅ Builds on Ubuntu 20.04 and 22.04
- ✅ Builds with GCC and Clang
- ✅ All unit tests pass
- ✅ Code formatting (clang-format)
- ✅ Static analysis (cppcheck)
- ✅ Documentation builds successfully

## What to Contribute

### High Priority

See [MISSING_FEATURES.md](MISSING_FEATURES.md) for details:

1. **Fix EKF edge cases**
   - Better covariance conditioning
   - Innovation gating
   - Outlier rejection

2. **Add ZUPT**
   - Zero-velocity detection
   - Stationary update implementation

3. **Improve documentation**
   - More examples
   - Tuning guide
   - Frame convention diagrams

### Medium Priority

4. **Adaptive filtering**
   - Auto-tune Q/R matrices
   - Innovation-based adaptation

5. **Real sensor drivers**
   - I2C/SPI/Serial interfaces
   - Specific IMU support (MPU6050, BMI088, etc.)

6. **ROS integration**
   - ROS1 and ROS2 nodes
   - Message conversions

### Low Priority

7. **Python bindings** (pybind11)
8. **Additional filters** (UKF, particle filter)
9. **Advanced features** (non-holonomic constraints, fault detection)

## Questions?

- Open an issue for bugs or feature requests
- Start a discussion for questions
- Email: [maintainer email if available]

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing! 🚀
