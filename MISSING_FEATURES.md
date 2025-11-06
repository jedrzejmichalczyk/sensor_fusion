# Missing Features and Known Issues

This document tracks missing features, known bugs, and future enhancements for the sensor fusion navigation system.

**Status**: Initial implementation complete, but numerical stability issues exist.

## Critical Issues ❌

### 1. **EKF Numerical Instability**
- **Problem**: Filter diverges exponentially during runtime
- **Impact**: System unusable for real applications
- **Root Causes**:
  - Gravity compensation may be incorrect in body frame
  - Process noise (Q matrix) may be mis-tuned
  - State transition matrix F may have errors
  - Covariance matrix P becomes ill-conditioned
- **Fix Priority**: **CRITICAL**
- **Proposed Solutions**:
  - Debug gravity handling in mechanization equations
  - Implement Joseph form covariance update
  - Add covariance limiting/conditioning
  - Use square-root filtering
  - Add innovation gating

### 2. **No Measurement Validation**
- **Problem**: Filter accepts any measurement without checking validity
- **Impact**: Outliers cause divergence
- **Fix Priority**: **HIGH**
- **Solutions**:
  - Chi-square test for innovations
  - Mahalanobis distance gating
  - NEES (Normalized Estimation Error Squared) monitoring

### 3. **No State/Covariance Bounds**
- **Problem**: No limits on state estimates or uncertainty
- **Impact**: Allows numerical overflow
- **Fix Priority**: **HIGH**
- **Solutions**:
  - Implement state saturation limits
  - Covariance matrix conditioning
  - NaN/Inf detection and recovery

## Essential Missing Features ⚠️

### 4. **Data Logging** ✅ (Partially Complete)
- **Status**: Basic CSV logger implemented but not integrated
- **Missing**:
  - Real-time plotting
  - Binary logging for large datasets
  - ROS bag export
- **Priority**: **HIGH**

### 5. **Configuration File Support**
- **Status**: Not implemented
- **Current**: All parameters hard-coded
- **Needed**:
  - YAML/JSON configuration files
  - Runtime parameter adjustment
  - Multiple configuration profiles
- **Priority**: **MEDIUM**

### 6. **Initial Alignment Procedures**
- **Status**: Not implemented
- **Current**: Assumes perfect initial state
- **Needed**:
  - Static alignment (stationary calibration)
  - Dynamic alignment (motion-based)
  - Automatic heading initialization
  - GPS-aided initialization
- **Priority**: **HIGH**

### 7. **Visualization Tools**
- **Status**: Not implemented
- **Needed**:
  - Python plotting scripts
  - Real-time trajectory display
  - Covariance ellipse plotting
  - Innovation sequence plots
- **Priority**: **MEDIUM**

### 8. **Comprehensive Error Handling**
- **Status**: Minimal
- **Missing**:
  - Exception handling
  - Graceful degradation
  - Error reporting/logging
  - Recovery mechanisms
- **Priority**: **MEDIUM**

## Advanced Features (Future Work)

### 9. **Zero Velocity Updates (ZUPT)**
- **Description**: Detect stationary periods and apply velocity=0 constraint
- **Benefit**: Dramatically reduces drift during stops
- **Applications**: Pedestrian navigation, indoor positioning
- **Priority**: **LOW**

### 10. **Adaptive Noise Estimation**
- **Description**: Automatically tune Q and R matrices based on innovation statistics
- **Benefit**: More robust to changing conditions
- **Methods**: Innovation-based adaptive filtering, fading memory
- **Priority**: **LOW**

### 11. **Outlier Detection and Rejection**
- **Description**: Identify and reject bad measurements
- **Methods**:
  - Chi-square test (innovation^2 < threshold)
  - NEES (Normalized Estimation Error Squared)
  - NIS (Normalized Innovation Squared)
  - RAIM (Receiver Autonomous Integrity Monitoring) for GPS
- **Priority**: **MEDIUM**

### 12. **Multi-Rate Sensor Handling**
- **Description**: Better support for sensors with different update rates
- **Current**: Basic counter-based approach
- **Needed**:
  - Proper timestamp-based synchronization
  - Asynchronous measurement updates
  - Delayed state augmentation
- **Priority**: **LOW**

### 13. **Non-Holonomic Constraints**
- **Description**: Constrain vehicle motion (e.g., no lateral slip for car)
- **Benefit**: Improves accuracy for ground vehicles
- **Priority**: **LOW**

### 14. **Fault Detection and Isolation**
- **Description**: Detect and isolate sensor failures
- **Methods**:
  - Redundancy-based voting
  - Model-based fault detection
  - Parity space approach
- **Priority**: **LOW**

### 15. **Advanced Covariance Management**
- **Description**: Prevent covariance matrix degradation
- **Methods**:
  - Joseph form update (numerically stable)
  - Square-root filtering (UD factorization)
  - Covariance inflation/limiting
- **Priority**: **HIGH** (related to stability issue)

### 16. **Real Sensor Interfaces**
- **Description**: Interface with actual hardware
- **Current**: Simulation only
- **Needed**:
  - Serial/USB IMU drivers
  - GPS NMEA parsing
  - I2C/SPI sensor drivers
  - ROS topic interfaces
- **Priority**: **MEDIUM**

### 17. **Advanced Trajectory Generators**
- **Description**: More realistic test scenarios
- **Current**: Simple circular/linear motion
- **Needed**:
  - Figure-8 patterns
  - Random walk
  - Real GPS track replay
  - 3D aerobatic maneuvers
- **Priority**: **LOW**

### 18. **Performance Profiling**
- **Description**: Measure computational efficiency
- **Needed**:
  - Execution time per update
  - Memory usage monitoring
  - CPU profiling
  - Embedded platform benchmarks
- **Priority**: **LOW**

## Software Engineering Improvements

### 19. **Continuous Integration**
- **Description**: Automated testing and deployment
- **Needed**:
  - GitHub Actions / GitLab CI
  - Automated builds for x64 and ARM
  - Test coverage reports
  - Static analysis integration
- **Priority**: **LOW**

### 20. **Code Coverage**
- **Description**: Measure test coverage
- **Current**: Unknown coverage
- **Tools**: gcov, lcov
- **Priority**: **LOW**

### 21. **API Documentation**
- **Description**: Generate Doxygen documentation
- **Current**: Inline comments only
- **Needed**:
  - Doxygen configuration
  - Class diagrams
  - API reference manual
- **Priority**: **LOW**

### 22. **Python Bindings**
- **Description**: Use from Python for rapid experimentation
- **Tools**: pybind11, SWIG
- **Benefits**: Easier prototyping, Jupyter notebook support
- **Priority**: **LOW**

### 23. **ROS/ROS2 Integration**
- **Description**: Robot Operating System compatibility
- **Needed**:
  - ROS node wrapper
  - Message type conversions
  - Launch files
- **Priority**: **MEDIUM** (for robotics applications)

### 24. **Extended Unit Tests**
- **Description**: More comprehensive test coverage
- **Missing**:
  - Edge case testing
  - Stress tests
  - Monte Carlo validation
  - Hardware-in-the-loop tests
- **Priority**: **MEDIUM**

## Documentation Gaps

### 25. **Frame Convention Documentation**
- **Description**: Clear definition of coordinate frames
- **Needed**:
  - Body frame definition (FRD vs FLU)
  - Navigation frame (NED vs ENU)
  - Earth frame conventions
  - Transformation diagrams
- **Priority**: **HIGH** (causes confusion)

### 26. **Filter Tuning Guide**
- **Description**: How to adjust Q and R matrices
- **Needed**:
  - Step-by-step tuning procedure
  - Rule-of-thumb guidelines
  - Example tuning scenarios
  - Sensitivity analysis
- **Priority**: **MEDIUM**

### 27. **Performance Plots**
- **Description**: Add figures to LaTeX documentation
- **Needed**:
  - Error vs. time plots
  - Allan variance curves
  - Monte Carlo results
  - Covariance consistency plots
- **Priority**: **LOW**

### 28. **Example Datasets**
- **Description**: Provide realistic test data
- **Needed**:
  - Recorded sensor data
  - Ground truth trajectories
  - Various motion profiles
  - Public dataset references
- **Priority**: **LOW**

## Known Bugs 🐛

1. **EKF Divergence** (Critical)
   - Test: `test_sensor_fusion` fails
   - Example: `imu_gps_fusion` diverges after ~3 seconds
   - Error: Position estimate grows exponentially

2. **VLA Warnings** (Fixed)
   - ~~Variable-length arrays not C++ standard~~
   - Status: ✅ Fixed using std::vector

3. **Missing GRAVITY constant** (Fixed)
   - ~~Examples and tests didn't define GRAVITY~~
   - Status: ✅ Fixed

## Implementation Priority

### Phase 1: Critical Fixes (Week 1)
1. Debug EKF divergence issue
2. Implement measurement validation
3. Add state/covariance bounds
4. Document frame conventions

### Phase 2: Essential Features (Week 2-3)
1. Integrate data logging into examples
2. Add visualization scripts
3. Implement initial alignment
4. Create configuration file support

### Phase 3: Advanced Features (Month 2)
1. ZUPT implementation
2. Adaptive filtering
3. Fault detection
4. Real sensor interfaces

### Phase 4: Polish (Month 3+)
1. ROS integration
2. Python bindings
3. Complete documentation
4. Performance optimization

## Contributing

If you want to help fix these issues:
1. Pick an item from the list
2. Create a feature branch
3. Add tests for your changes
4. Submit a pull request

Priority labels: **CRITICAL** > **HIGH** > **MEDIUM** > **LOW**

## References

- Groves Ch. 14: INS/GNSS Integration
- Bar-Shalom Ch. 11: Practical Aspects of Kalman Filtering
- Farrell Ch. 8: Fault Detection and Integrity Monitoring
