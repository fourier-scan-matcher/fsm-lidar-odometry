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

/*
 * Settings that cannot work, and what the package says about them.
 *
 * The published version checked these with assertions, which do nothing at all
 * in a release build and abort the process in any other. Neither is any use to
 * somebody who has mistyped a number in a configuration file. The check now
 * returns a sentence naming the setting, and the node refuses to start and
 * prints it.
 *
 * Every case below asserts that the sentence names the setting that is wrong,
 * because a refusal that does not say which of sixteen numbers is at fault is
 * barely better than the assertion it replaced.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <string>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{

/*
 * The two values that are not numbers are written as bit patterns rather than
 * taken from std::numeric_limits. A pattern is what it says it is whatever the
 * compiler has been told about floating point, whereas a library constant read
 * under fast arithmetic can be folded into something else before it is ever
 * handed over. What these tests exercise is a library built with settings this
 * file does not share, so the value has to be pinned on this side of the call.
 */
constexpr std::uint64_t kPositiveInfinityBits = 0x7FF0000000000000ULL;
constexpr std::uint64_t kQuietNotANumberBits = 0x7FF8000000000000ULL;

/* Case-insensitive, since the message is prose and the setting is not. */
bool mentions(const std::string& message, const std::string& setting)
{
  return message.find(setting) != std::string::npos;
}

}  // namespace

/*
 * The defaults are the values documented in the readme, and they must pass.
 * If this fails then the package cannot start at all without a configuration
 * file, and every other case here is meaningless.
 */
TEST(ParameterValidation, TheDefaultsAreAccepted)
{
  EXPECT_EQ(fsm_lidar_odometry::validate(fsm_lidar_odometry::Parameters{}), std::string{});
}

/*
 * Zero is not a mistake here. It means match every ray the scan carries rather
 * than reduce it to a fixed number, which is the default and the reason there
 * is nothing to refuse.
 */
TEST(ParameterValidation, AScanSizeOfZeroIsAcceptedAndMeansMatchWhole)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

/*
 * Zero iterations means the translation stage never runs, so the matcher would
 * report the pose it was given as the pose it found.
 */
TEST(ParameterValidation, ZeroIterationsAreRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.num_iterations = 0;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "num_iterations")) << problem;
}

/*
 * The bounds are distances and angles, so a negative one is not a smaller
 * bound, it is a bound nothing can satisfy. The message repeats the value back,
 * since a sign is easy to miss in a configuration file.
 */
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

/*
 * A bound that is not a number is worse than a negative one. It survives every
 * ordering comparison the recovery search makes, so the search draws pose
 * after pose and none of them is ever inside the bound. The published version
 * caught this with an assertion, and assertions are compiled out of the build
 * that ships, so the node simply stops answering. A parameter file or a
 * command line can carry the value, which makes this reachable without writing
 * any code at all.
 *
 * Each case asserts the bits of the value it is about to hand over, so a run
 * that silently turned it into something ordinary fails here rather than
 * further down where it would look like the refusal working.
 */
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

/*
 * An infinite bound is the other half of the same refusal. It passes the
 * negative check, and it describes a search area no draw can be outside of,
 * which is not a bound.
 */
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

/*
 * A bound of exactly zero is allowed. It pins the search to the pose it starts
 * from, which is a strange thing to want but not a contradiction, and refusing
 * it would be the check overreaching.
 */
TEST(ParameterValidation, BoundsOfExactlyZeroAreAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.xy_bound = 0.0;
  parameters.t_bound = 0.0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

/*
 * The counter limits how many attempts each magnification level gets. Zero
 * means the level is over before it starts.
 */
TEST(ParameterValidation, ACounterLimitOfZeroIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.max_counter = 0;

  const std::string problem = fsm_lidar_odometry::validate(parameters);

  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(mentions(problem, "max_counter")) << problem;
}

/*
 * The magnification ladder runs from the smallest to the largest, so a largest
 * below the smallest describes a ladder with no rungs.
 */
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

/*
 * A single rung is a ladder. The two being equal is the configuration that
 * turns magnification off, which is legitimate.
 */
TEST(ParameterValidation, AMagnificationRangeOfOneLevelIsAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.min_magnification_size = 2;
  parameters.max_magnification_size = 2;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

/*
 * No recoveries at all is allowed: it means give up rather than guess, which
 * is what somebody comparing two builds wants, since a guess cannot be
 * reproduced.
 */
TEST(ParameterValidation, ForbiddingRecoveryIsAllowed)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.max_recoveries = 0;

  EXPECT_EQ(fsm_lidar_odometry::validate(parameters), std::string{});
}

/*
 * Both ray searches are accepted by name, and nothing else is. A misspelling
 * must refuse startup rather than fall back to a default, because the two
 * searches disagree about what a room with a re-entrant corner looks like and
 * a run made with the wrong one would look ordinary.
 */
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

/*
 * Only the first problem is reported. Somebody fixing a configuration file
 * wants one thing to fix at a time, and the check stops at the first, so a
 * file with two mistakes names the earlier one.
 */
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
