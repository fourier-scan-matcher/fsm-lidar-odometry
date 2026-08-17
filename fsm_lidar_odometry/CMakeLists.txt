cmake_minimum_required(VERSION 3.16)
project(fsm_lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FSM_LIDAR_ODOMETRY_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(FSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

if(FSM_LIDAR_ODOMETRY_EXACT_FP)
  add_compile_options(-O2 -fno-fast-math -ffp-contract=off)
else()
  add_compile_options(-Ofast -fno-finite-math-only)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)

add_library(fsm_lidar_odometry_core SHARED
  src/fsm_lidar_odometry/fsm_lidar_odometry.cpp)

target_include_directories(fsm_lidar_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(fsm_lidar_odometry_core PUBLIC
  Eigen3::Eigen
  CGAL::CGAL
  PkgConfig::FFTW3)

if(FSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT)
  target_compile_definitions(fsm_lidar_odometry_core PUBLIC FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE)
endif()

add_library(fsm_lidar_odometry_interface SHARED
  src/fsm_lidar_odometry/fsm_lidar_odometry_interface.cpp)

target_link_libraries(fsm_lidar_odometry_interface PUBLIC
  fsm_lidar_odometry_core
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS})

add_executable(fsm_lidar_odometry_interface_node
  src/fsm_lidar_odometry/fsm_lidar_odometry_interface_node.cpp)

target_link_libraries(fsm_lidar_odometry_interface_node fsm_lidar_odometry_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_library(fsm_lidar_odometry_core_exact OBJECT
    src/fsm_lidar_odometry/fsm_lidar_odometry.cpp)

  target_include_directories(fsm_lidar_odometry_core_exact PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(fsm_lidar_odometry_core_exact PUBLIC
    Eigen3::Eigen
    CGAL::CGAL
    PkgConfig::FFTW3)

  target_compile_options(fsm_lidar_odometry_core_exact PUBLIC
    -O2 -fno-fast-math -ffp-contract=off)

  target_compile_definitions(fsm_lidar_odometry_core_exact PUBLIC
    FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE)

  ament_add_gtest(test_fsm_lidar_odometry_core
    test/test_defects_core.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_core_golden
    test/test_core_golden.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_dft
    test/test_dft.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_finite_guard
    test/test_finite_guard.cpp)
  target_include_directories(test_fsm_lidar_odometry_finite_guard PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_fsm_lidar_odometry_finite_guard
    Eigen3::Eigen
    CGAL::CGAL
    PkgConfig::FFTW3)
  target_compile_options(test_fsm_lidar_odometry_finite_guard PRIVATE
    "SHELL:-Ofast -ffast-math -fno-finite-math-only -ffp-contract=fast")
  target_compile_definitions(test_fsm_lidar_odometry_finite_guard PRIVATE
    FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE)

  ament_add_gtest(test_fsm_lidar_odometry_geometry
    test/test_geometry.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_invalid_ranges
    test/test_invalid_ranges.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_params
    test/test_params.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_rotation_criterion_carryover
    test/test_rotation_criterion_carryover.cpp)
  target_include_directories(test_fsm_lidar_odometry_rotation_criterion_carryover PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_fsm_lidar_odometry_rotation_criterion_carryover
    Eigen3::Eigen
    CGAL::CGAL
    PkgConfig::FFTW3)
  target_compile_options(test_fsm_lidar_odometry_rotation_criterion_carryover PRIVATE
    -O2 -fno-fast-math -ffp-contract=off)
  target_compile_definitions(test_fsm_lidar_odometry_rotation_criterion_carryover PRIVATE
    _GLIBCXX_ASSERTIONS
    FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE
    FSM_LIDAR_ODOMETRY_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  ament_add_gtest(test_fsm_lidar_odometry_scan_handling
    test/test_scan_handling.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_stages
    test/test_stages.cpp)

  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_node_startup.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=81
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/test_node_behaviour.py TIMEOUT 400
    ENV ROS_DOMAIN_ID=82
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_initial_pose.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=83
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  foreach(test_target test_fsm_lidar_odometry_core test_fsm_lidar_odometry_core_golden
                      test_fsm_lidar_odometry_dft test_fsm_lidar_odometry_geometry
                      test_fsm_lidar_odometry_invalid_ranges test_fsm_lidar_odometry_params
                      test_fsm_lidar_odometry_scan_handling test_fsm_lidar_odometry_stages)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} fsm_lidar_odometry_core_exact)
      target_compile_definitions(${test_target} PRIVATE
        FSM_LIDAR_ODOMETRY_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
  endforeach()
endif()

install(TARGETS
  fsm_lidar_odometry_core
  fsm_lidar_odometry_interface
  EXPORT export_fsm_lidar_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS
  fsm_lidar_odometry_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_fsm_lidar_odometry HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  CGAL)

ament_package(CONFIG_EXTRAS cmake/fsm_lidar_odometry-extras.cmake)
