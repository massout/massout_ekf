cmake_minimum_required(VERSION 3.12)
project(massout_ekf VERSION 1.0 LANGUAGES CXX)

add_library(massout_ekf INTERFACE)

target_include_directories(massout_ekf
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(massout_ekf INTERFACE cxx_std_11)

install(TARGETS massout_ekf
        EXPORT massout_ekfTargets
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION include)

install(EXPORT massout_ekfTargets
        FILE massout_ekfTargets.cmake
        DESTINATION lib/cmake/massout_ekf)

option(MASSOUT_EKF_BUILD_TESTS "Build massout_ekf tests" ON)
if (MASSOUT_EKF_BUILD_TESTS)
    enable_testing()

    add_executable(ekf_constant_velocity tests/constant_velocity.cpp)
    target_link_libraries(ekf_constant_velocity PRIVATE massout_ekf)
    add_test(NAME ekf_constant_velocity COMMAND ekf_constant_velocity)

    add_executable(ekf_variable_acceleration tests/variable_acceleration.cpp)
    target_link_libraries(ekf_variable_acceleration PRIVATE massout_ekf)
    add_test(NAME ekf_variable_acceleration COMMAND ekf_variable_acceleration)

    add_executable(ekf_multiple_filters tests/multiple_filters.cpp)
    target_link_libraries(ekf_multiple_filters PRIVATE massout_ekf)
    add_test(NAME ekf_multiple_filters COMMAND ekf_multiple_filters)
endif()
