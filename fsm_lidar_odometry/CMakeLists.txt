cmake_minimum_required(VERSION 3.16)
project(fsm_lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# FSM_LIDAR_ODOMETRY_EXACT_FP=ON replaces -Ofast with optimisation settings that forbid the
# compiler from reassociating floating point arithmetic or contracting
# multiply-adds. Two compilers reassociate differently, so a reference build and
# a build under test only produce comparable numbers when this is on.
#
# FSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT=ON makes FFTW select its plan by heuristic rather
# than by timing itself. Plan choice affects rounding, so without this the same
# binary can produce different numbers on two consecutive runs.
#
# Both default to OFF, which is the configuration this package ships with. The
# tests below force both ON regardless, since their tolerances are meaningless
# otherwise.
# ------------------------------------------------------------------------------
option(FSM_LIDAR_ODOMETRY_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(FSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

if(FSM_LIDAR_ODOMETRY_EXACT_FP)
  add_compile_options(-O2 -fno-fast-math -ffp-contract=off)
else()
  # -Ofast turns on -ffast-math, which turns on -ffinite-math-only. That sub
  # flag lets the compiler assume no infinity or not-a-number value ever
  # occurs, which folds a check written to catch one into a no-op and can
  # make an equality comparison against a literal report true for a value
  # that is not a number. Disabling it costs a parity check on the rare
  # double equality comparison this package's hot loops make, and keeps
  # every other relaxation -Ofast brings: the higher optimisation level,
  # reassociation, reciprocal approximations, signed zero and trapping
  # assumptions, and flush to zero.
  add_compile_options(-Ofast -fno-finite-math-only)
endif()

# ------------------------------------------------------------------------------
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

# ------------------------------------------------------------------------------
# The core: the matcher and the odometry state it drives. No ROS anywhere in
# here, so it can be tested without a ROS installation present.
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

# ------------------------------------------------------------------------------
# The ROS interface, and the executable that runs it.
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

# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The two formatting linters impose a brace and wrapping style that this
  # package has never used, and the core header is 3800 lines of inherited code
  # whose arithmetic must not be disturbed. Reformatting it to satisfy a linter
  # would be the single riskiest change in the package for no behavioural gain.
  # Everything that checks something other than formatting stays on.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The tests assert agreement to 1e-9 and tighter. Under fast arithmetic the
  # compiler is free to reassociate, which moves results well past that, so a
  # passing test would say nothing. Force the exact configuration here whatever
  # the library itself was configured with.
  ament_add_gtest(test_fsm_lidar_odometry_core
    test/test_defects_core.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_core_golden
    test/test_core_golden.cpp)

  ament_add_gtest(test_fsm_lidar_odometry_dft
    test/test_dft.cpp)

  # Characterises the four finite checks in the transform stage that
  # -Ofast's -ffinite-math-only folds to a no-op. Deliberately not part of the
  # foreach loop below: that loop forces the exact floating point
  # configuration precisely so this one target does not get it, since the
  # whole point is to run this file under the configuration the package
  # actually ships. It is also not linked against fsm_lidar_odometry_core, so
  # its own compile flags, not the library's, are what govern the code that
  # runs.
  #
  # The shipping configuration is pinned on the target rather than inherited
  # from directory scope, because FSM_LIDAR_ODOMETRY_EXACT_FP=ON rewrites
  # directory scope to the exact settings and would otherwise turn the one file
  # written to defeat the optimiser into another copy of the same safe build.
  # Three things make the pin fussier than it looks, and getting any of them
  # wrong yields a target that reads as pinned and is not:
  #
  #   * FSM_LIDAR_ODOMETRY_EXACT_FP=ON rewrites directory scope to
  #     -fno-fast-math and -ffp-contract=off, and directory options precede
  #     target options on the command line, so both have to be restated to be
  #     undone.
  #   * gcc applies the flags implied by an -O level before it applies any
  #     explicit flag, so a later -Ofast on its own does not undo an earlier
  #     explicit -fno-fast-math. -ffast-math has to be named outright.
  #   * cmake de-duplicates compile options, so a flag restated here that also
  #     appears at directory scope is dropped from this list rather than moved
  #     to the end of it. That silently reorders the rest, and -ffast-math
  #     landing after -fno-finite-math-only turns finite math back on. The
  #     SHELL: prefix is what keeps the four flags together and in order.
  #
  # The file fails to compile if this is ever weakened.
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

  # Characterises the out of bounds read in Match::fmtdbh's candidate angle
  # sift. Deliberately not linked against fsm_lidar_odometry_core, and given
  # its own -D_GLIBCXX_ASSERTIONS on top of the exact floating point settings
  # every other suite already forces, so libstdc++'s own bounds check on
  # std::vector::operator[] is what catches the defect in this file's own
  # compiled code, rather than in whatever fsm_lidar_odometry_core happened to
  # be built with.
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

  # The launch runner imports a test file directly rather than through pytest,
  # so the directory holding the shared helpers has to be on the import path
  # for it. Setting it here keeps the test files free of path manipulation.
  #
  # The startup contract needs a node nothing has touched, so it launches its
  # own rather than sharing the one the behaviour tests drive.
  add_launch_test(test/test_node_startup.py TIMEOUT 300
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/test_node_behaviour.py TIMEOUT 400
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  # The initial pose cases launch their own node for a different reason: the
  # service they drive holds the node until a message arrives on the initial
  # pose topic, which is not a state to hand on to a case expecting an
  # answer.
  add_launch_test(test/test_initial_pose.py TIMEOUT 300
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  foreach(test_target test_fsm_lidar_odometry_core test_fsm_lidar_odometry_core_golden
                      test_fsm_lidar_odometry_dft test_fsm_lidar_odometry_geometry
                      test_fsm_lidar_odometry_invalid_ranges test_fsm_lidar_odometry_params
                      test_fsm_lidar_odometry_scan_handling test_fsm_lidar_odometry_stages)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} fsm_lidar_odometry_core)
      target_compile_options(${test_target} PRIVATE
        -O2 -fno-fast-math -ffp-contract=off)
      target_compile_definitions(${test_target} PRIVATE
        FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE
        FSM_LIDAR_ODOMETRY_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
  endforeach()
endif()

# ------------------------------------------------------------------------------
# The two libraries go into an export set rather than being installed loose, so
# that a consumer's find_package(fsm_lidar_odometry) yields imported targets
# carrying this package's include directories and its link dependencies.
# Announcing include directories and library names alone, as this file did,
# leaves a consumer to rediscover Eigen, CGAL and FFTW3 for itself and to guess
# the link order, which is the same as announcing nothing usable.
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

# What the two libraries expose in their own headers, and therefore what a
# consumer has to have found before it can compile against them. FFTW3 is the
# exception: it has no CMake package to find, so the extras file below builds
# its imported target the same way this file does.
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
