# Sensor Fusion for Navigation Systems

A comprehensive C++ implementation of sensor fusion for navigation, combining IMU, GPS, barometric, and magnetometer sensors using Extended Kalman Filter (EKF) algorithms. Designed for cross-platform deployment including embedded systems.

## Features

- **Multi-Sensor Integration**
  - Inertial Measurement Unit (IMU) with realistic error models
  - GPS receiver simulation
  - Barometric altimeter
  - 3-axis magnetometer

- **Advanced Algorithms**
  - Extended Kalman Filter (EKF) implementation
  - Error-state formulation for numerical stability
  - Quaternion-based attitude representation

- **Cross-Platform Support**
  - C++17 standard for portability
  - CMake build system
  - ARM cross-compilation toolchain
  - Minimal dependencies (header-only Eigen3 optional)

- **Comprehensive Documentation**
  - LaTeX technical documentation
  - BibTeX bibliography with academic references
  - Inline code documentation
  - Example programs

## Project Structure

```
sensor_fusion/
├── include/              # Header files
│   ├── sensors/         # Sensor models
│   ├── fusion/          # Kalman filter
│   └── utils/           # Utilities
├── src/                 # Implementation files
│   ├── sensors/
│   ├── fusion/
│   └── utils/
├── tests/               # Unit tests
├── examples/            # Example programs
├── doc/                 # LaTeX documentation
│   ├── sensor_fusion_documentation.tex
│   ├── references.bib
│   └── Makefile
├── build/               # Build directory
└── CMakeLists.txt       # Build configuration
```

## Building

### Prerequisites

- CMake 3.15 or higher
- C++17 compatible compiler (GCC, Clang, MSVC)
- Optional: Eigen3 library (bundled version available)
- Optional: LaTeX distribution for documentation (pdflatex, bibtex)

### Standard Build (x64)

```bash
mkdir build && cd build
cmake ..
make
make test
```

### ARM Cross-Compilation

```bash
mkdir build-arm && cd build-arm
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-arm.cmake ..
make
```

### Building Documentation

```bash
cd doc
make
# Produces sensor_fusion_documentation.pdf
```

## Running Examples

### Complete Simulation

```bash
./build/simulation_example
```

Demonstrates full sensor fusion with circular trajectory, showing:
- IMU-based dead reckoning
- GPS position/velocity updates
- Barometric altitude corrections
- Magnetometer heading updates

### IMU/GPS Fusion with Outage

```bash
./build/imu_gps_fusion
```

Shows GPS outage handling and error growth during dead-reckoning.

## Sensor Models

### IMU Error Model

Implements realistic MEMS IMU errors:
- White Gaussian noise
- Bias instability (Gauss-Markov process)
- Scale factor errors
- Temperature effects (placeholder)

Typical parameters (consumer-grade):
- Accelerometer noise: 150 μg/√Hz
- Gyroscope noise: 0.01 °/s/√Hz
- Bias stability: 40 μg (accel), 10 °/h (gyro)

### GPS Model

- Position accuracy: 2.5 m (horizontal), 5.0 m (vertical)
- Velocity accuracy: 0.1 m/s
- Update rate: 1-10 Hz
- Outage simulation capability

### Barometer Model

- Altitude accuracy: 1 m
- ISA atmosphere model
- Update rate: 10-50 Hz

### Magnetometer Model

- Noise: 0.3 μT/√Hz
- Hard iron bias: 5 μT
- Soft iron errors: 1%
- Simplified Earth magnetic field model

## Extended Kalman Filter

### State Vector (15D Error State)

- Position error (3D)
- Velocity error (3D)
- Attitude error (3D, rotation vector)
- Accelerometer bias error (3D)
- Gyroscope bias error (3D)

### Key Features

- Error-state formulation for stability
- Quaternion normalization
- Configurable process/measurement noise
- Modular measurement update interface

## Testing

Run all tests:

```bash
cd build
ctest
```

Or run individual tests:

```bash
./build/test_imu
./build/test_gps
./build/test_ekf
./build/test_sensor_fusion
```

## API Example

```cpp
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "fusion/ekf.hpp"

// Create sensors
sensor_fusion::IMU imu(sensor_fusion::IMUErrorModel(), 100.0);
sensor_fusion::GPS gps(sensor_fusion::GPSErrorModel(), 1.0);

// Initialize EKF
sensor_fusion::NavigationState initial_state;
sensor_fusion::ExtendedKalmanFilter ekf(initial_state);

// Simulation loop
for (double t = 0.0; t < sim_time; t += dt) {
    // Get IMU measurement
    auto imu_meas = imu.measure(true_accel, true_gyro, t);

    // EKF prediction
    ekf.predict(imu_meas, dt);

    // GPS update (when available)
    if (gps_update_available) {
        auto gps_meas = gps.measure(true_pos, true_vel, t);
        ekf.updateGPS(gps_meas);
    }

    // Get state estimate
    auto state = ekf.getState();
}
```

## Performance

### Computational Complexity

- Prediction step: O(n²) where n=15 (error state size)
- GPS update: O(n·m) where m=6 (measurement size)
- Barometer update: O(n) (scalar measurement)

### Memory Requirements

- State: ~200 bytes
- Covariance matrix: ~1 KB
- Total: <5 KB (suitable for embedded systems)

## References

The implementation is based on established literature in sensor fusion and navigation. See `doc/references.bib` for complete bibliography, including:

- Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"
- Titterton & Weston, "Strapdown Inertial Navigation Technology"
- Woodman, "An Introduction to Inertial Navigation"
- Bar-Shalom et al., "Estimation with Applications to Tracking and Navigation"

Full technical documentation is available in `doc/sensor_fusion_documentation.pdf`.

## Future Enhancements

- [ ] Unscented Kalman Filter (UKF) implementation
- [ ] Adaptive covariance tuning
- [ ] Visual odometry integration
- [ ] Outlier rejection and fault detection
- [ ] Real-time data logging
- [ ] Python bindings
- [ ] ROS integration

## License

This project is provided for educational and research purposes.

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## Authors

Sensor Fusion Project Team

## Acknowledgments

Based on research and algorithms from the navigation and estimation community. Special thanks to academic researchers who have published their work openly.
