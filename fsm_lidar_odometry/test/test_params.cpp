// Copyright 2022 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <string>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{
constexpr std::uint64_t kPositiveInfinityBits = 0x7FF0000000000000ULL;
constexpr std::uint64_t kQuietNotANumberBits = 0x7FF8000000000000ULL;

bool mentions(const std::string& message, const std::string& setting)
{
  return message.find(setting) != std::string::npos;
}

}

TEST(ParameterValidation, TheDefaultsAreAccepted)
{
  EXPECT_EQ(fsm_lidar_odometry::validate(fsm_lidar_odometry::Parameters{}), std::string{});
}

TEST(ParameterValidation, AScanSizeOfZeroIsAcceptedAndMeansMatchWhole)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

TEST(ParameterValidation, ZeroIterationsAreRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.num_iterations = 0;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "num_iterations")) << problem;
}

TEST(ParameterValidation, ANegativePositionBoundIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.xy_bound = -0.2;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "xy_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "-0.2")) << problem;
}

TEST(ParameterValidation, ANegativeOrientationBoundIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.t_bound = -0.5;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "t_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "-0.5")) << problem;
}

TEST(ParameterValidation, APositionBoundThatIsNotANumberIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.xy_bound = std::bit_cast<double>(kQuietNotANumberBits);

  ASSERT_EQ(std::bit_cast<std::uint64_t>(parameters.xy_bound),
    kQuietNotANumberBits);

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "xy_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "nan")) << problem;
}

TEST(ParameterValidation, AnOrientationBoundThatIsNotANumberIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.t_bound = std::bit_cast<double>(kQuietNotANumberBits);

  ASSERT_EQ(std::bit_cast<std::uint64_t>(parameters.t_bound),
    kQuietNotANumberBits);

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "t_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "nan")) << problem;
}

TEST(ParameterValidation, AnInfinitePositionBoundIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.xy_bound = std::bit_cast<double>(kPositiveInfinityBits);

  ASSERT_EQ(std::bit_cast<std::uint64_t>(parameters.xy_bound),
    kPositiveInfinityBits);

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "xy_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "inf")) << problem;
}

TEST(ParameterValidation, AnInfiniteOrientationBoundIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.t_bound = std::bit_cast<double>(kPositiveInfinityBits);

  ASSERT_EQ(std::bit_cast<std::uint64_t>(parameters.t_bound),
    kPositiveInfinityBits);

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "t_bound")) << problem;
  EXPECT_TRUE(mentions(problem, "inf")) << problem;
}

TEST(ParameterValidation, BoundsOfExactlyZeroAreAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.xy_bound = 0.0;
  parameters.t_bound = 0.0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

TEST(ParameterValidation, ACounterLimitOfZeroIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.max_counter = 0;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "max_counter")) << problem;
}

TEST(ParameterValidation, AnInvertedMagnificationRangeIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.min_magnification_size = 3;
  parameters.max_magnification_size = 1;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "max_magnification_size")) << problem;
  EXPECT_TRUE(mentions(problem, "min_magnification_size")) << problem;
}

TEST(ParameterValidation, AMagnificationRangeOfOneLevelIsAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.min_magnification_size = 2;
  parameters.max_magnification_size = 2;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

TEST(ParameterValidation, ForbiddingRecoveryIsAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.max_recoveries = 0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

TEST(ParameterValidation, EitherRaySearchIsAcceptedByName)
{
  fsm_lidar_odometry::Parameters parameters;

  parameters.ray_search = "angular";
  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});

  parameters.ray_search = "windowed";
  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

TEST(ParameterValidation, AnUnknownRaySearchIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.ray_search = "Angular";

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_TRUE(mentions(problem, "ray_search")) << problem;
  EXPECT_TRUE(mentions(problem, "Angular")) << problem;
}

TEST(ParameterValidation, TheFirstProblemIsTheOneReported)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.num_iterations = 0;
  parameters.max_counter = 0;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_TRUE(mentions(problem, "num_iterations")) << problem;
  EXPECT_FALSE(mentions(problem, "max_counter")) << problem;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
