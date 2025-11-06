cmake_minimum_required(VERSION 3.15)

# Test executables
add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu sensor_fusion)
add_test(NAME test_imu COMMAND test_imu)

add_executable(test_gps test_gps.cpp)
target_link_libraries(test_gps sensor_fusion)
add_test(NAME test_gps COMMAND test_gps)

add_executable(test_ekf test_ekf.cpp)
target_link_libraries(test_ekf sensor_fusion)
add_test(NAME test_ekf COMMAND test_ekf)

add_executable(test_sensor_fusion test_sensor_fusion.cpp)
target_link_libraries(test_sensor_fusion sensor_fusion)
add_test(NAME test_sensor_fusion COMMAND test_sensor_fusion)
