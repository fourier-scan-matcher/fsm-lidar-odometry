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
 * Scans carrying rays the sensor could not measure.
 *
 * A driver reports such a ray as zero, as infinity, or as not-a-number,
 * depending on its conventions. Only zero was recognised before; the other two
 * went into the frequency transform as though they were distances, and a
 * single one of them contaminates every coefficient the transform produces.
 *
 * The property these tests pin is that all three forms mean the same thing.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{

const std::size_t kSize = 360;
const double kInfinity = std::numeric_limits<double>::infinity();
const double kNotANumber = std::numeric_limits<double>::quiet_NaN();

/*
 * Whether `value` is neither infinite nor not-a-number, read off its
 * exponent bits rather than asked of `std::isfinite`. This target currently
 * sits in the CMake group that forces `-fno-fast-math`, and under that
 * setting `std::isfinite` genuinely answers the question. But `-Ofast` is
 * what this package ships, and `-Ofast` implies `-ffinite-math-only`, under
 * which the compiler is entitled to assume no infinity or not-a-number ever
 * exists and folds `std::isfinite` to a constant `true`. Moving this target
 * out of the exact arithmetic group, for any reason, would turn the three
 * assertions below into assertions that always pass, with nothing failing to
 * say so. `isValidRange` in `fsm_lidar_odometry.cpp` and `isFinite` in
 * `fsm_core.hpp`'s `DFTUtils` already met this trap and read the bits
 * instead; this is the same remedy, kept independent of both since a test
 * has no business depending on the production code it is not exercising
 * here.
 */
bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

std::vector<double> roomScan(const double x, const double y)
{
  std::vector<double> ranges(kSize);
  for (std::size_t i = 0; i < kSize; i++)
  {
    const double angle = i * 2.0 * M_PI / kSize - M_PI;
    double nearest = 1e9;
    const double walls[4][3] = {
      {x, -1.0, 0.0}, {8.0 - x, 1.0, 0.0},
      {y, 0.0, -1.0}, {5.0 - y, 0.0, 1.0}};

    for (const auto& wall : walls)
    {
      const double projection =
        std::cos(angle) * wall[1] + std::sin(angle) * wall[2];
      if (projection > 1e-9)
        nearest = std::min(nearest, wall[0] / projection);
    }
    ranges[i] = nearest;
  }
  return ranges;
}

fsm_lidar_odometry::Parameters parameters()
{
  fsm_lidar_odometry::Parameters p;
  p.size_scan = kSize;
  return p;
}

fsm_lidar_odometry::Pose matchWith(std::vector<double> second)
{
  fsm_lidar_odometry::Matcher matcher(parameters());
  const auto first = matcher.process(roomScan(3.0, 2.5));
  EXPECT_FALSE(first.has_value());

  const auto result = matcher.process(second);
  EXPECT_TRUE(result.has_value());
  return result.has_value() ? result->increment : fsm_lidar_odometry::Pose{};
}

}  // namespace

TEST(InvalidRanges, ZeroInfinityAndNotANumberAreAllTreatedAsNoReading)
{
  std::vector<double> zeros = roomScan(3.05, 2.5);
  std::vector<double> infinities = zeros;
  std::vector<double> nans = zeros;

  for (std::size_t i = 100; i < 112; i++)
  {
    zeros[i] = 0.0;
    infinities[i] = kInfinity;
    nans[i] = kNotANumber;
  }

  const fsm_lidar_odometry::Pose from_zeros = matchWith(zeros);
  const fsm_lidar_odometry::Pose from_infinities = matchWith(infinities);
  const fsm_lidar_odometry::Pose from_nans = matchWith(nans);

  EXPECT_DOUBLE_EQ(from_infinities.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_infinities.y, from_zeros.y);
  EXPECT_DOUBLE_EQ(from_infinities.t, from_zeros.t);

  EXPECT_DOUBLE_EQ(from_nans.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_nans.y, from_zeros.y);
  EXPECT_DOUBLE_EQ(from_nans.t, from_zeros.t);
}

TEST(InvalidRanges, AnInfiniteRayDoesNotWreckTheMatch)
{
  const std::vector<double> clean = roomScan(3.05, 2.5);

  std::vector<double> with_infinity = clean;
  with_infinity[200] = kInfinity;

  const fsm_lidar_odometry::Pose expected = matchWith(clean);
  const fsm_lidar_odometry::Pose actual = matchWith(with_infinity);

  EXPECT_TRUE(isFinite(actual.x));
  EXPECT_TRUE(isFinite(actual.y));
  EXPECT_TRUE(isFinite(actual.t));

  EXPECT_NEAR(actual.x, expected.x, 1e-3);
  EXPECT_NEAR(actual.y, expected.y, 1e-3);
  EXPECT_NEAR(actual.t, expected.t, 1e-3);
}

TEST(InvalidRanges, ANegativeRangeIsTreatedAsNoReading)
{
  std::vector<double> zeros = roomScan(3.05, 2.5);
  std::vector<double> negatives = zeros;

  for (std::size_t i = 40; i < 48; i++)
  {
    zeros[i] = 0.0;
    negatives[i] = -1.0;
  }

  const fsm_lidar_odometry::Pose from_zeros = matchWith(zeros);
  const fsm_lidar_odometry::Pose from_negatives = matchWith(negatives);

  EXPECT_DOUBLE_EQ(from_negatives.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_negatives.y, from_zeros.y);
}

TEST(InvalidRanges, AnEntirelyInvalidScanIsRefused)
{
  fsm_lidar_odometry::Matcher matcher(parameters());

  const std::vector<double> all_infinite(kSize, kInfinity);
  const auto result = matcher.process(all_infinite);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lidar_odometry::MatchError::scan_entirely_invalid);
}

TEST(InvalidRanges, AnEntirelyInvalidScanDoesNotBecomeTheReference)
{
  fsm_lidar_odometry::Matcher matcher(parameters());

  ASSERT_FALSE(matcher.process(roomScan(3.0, 2.5)).has_value());

  const std::vector<double> all_zero(kSize, 0.0);
  ASSERT_FALSE(matcher.process(all_zero).has_value());

  const auto result = matcher.process(roomScan(3.05, 2.5));
  ASSERT_TRUE(result.has_value())
    << "the good scan before the invalid one should still be the reference";
}

TEST(InvalidRanges, IsValidRangeAgreesWithItsDocumentation)
{
  EXPECT_TRUE(fsm_lidar_odometry::isValidRange(1.0));
  EXPECT_FALSE(fsm_lidar_odometry::isValidRange(0.0));
  EXPECT_FALSE(fsm_lidar_odometry::isValidRange(-1.0));
  EXPECT_FALSE(fsm_lidar_odometry::isValidRange(kInfinity));
  EXPECT_FALSE(fsm_lidar_odometry::isValidRange(-kInfinity));
  EXPECT_FALSE(fsm_lidar_odometry::isValidRange(kNotANumber));
}

TEST(RecoverySeed, TheSameSeedProducesTheSameSequence)
{
  const FSM::Pose base;
  const std::vector<std::pair<double, double>> map{
    {-4.0, -4.0}, {4.0, -4.0}, {4.0, 4.0}, {-4.0, 4.0}};

  /*
   * A seed takes effect once and the stream runs on from there, which is what
   * makes a whole session replayable. Returning to a seed therefore has to go
   * by way of a different one, exactly as restarting the process would.
   */
  const auto draw = [&](const unsigned int seed)
  {
    std::vector<double> drawn;
    for (int i = 0; i < 8; i++)
    {
      const FSM::Pose pose =
        FSM::Utils::generatePose(base, map, 0.2, 0.3, 0.0,
          FSM::RaySearch::angular,
          i == 0 ? seed : 0).value();
      drawn.push_back(pose.x);
      drawn.push_back(pose.y);
      drawn.push_back(pose.t);
    }
    return drawn;
  };

  const std::vector<double> first = draw(20260813);
  const std::vector<double> other = draw(11111111);
  const std::vector<double> again = draw(20260813);

  ASSERT_EQ(first.size(), again.size());
  for (std::size_t i = 0; i < first.size(); i++)
    EXPECT_DOUBLE_EQ(first[i], again[i]) << "element " << i;

  EXPECT_NE(first, other) << "two different seeds produced the same sequence";
}

TEST(RecoverySeed, TheParametersCarryTheSeedIntoTheMatcher)
{
  fsm_lidar_odometry::Parameters p;
  p.rng_seed = 4242;

  EXPECT_EQ(p.rng_seed, 4242u);
  EXPECT_EQ(fsm_lidar_odometry::Parameters{}.rng_seed, 0u)
    << "the default must keep drawing from hardware entropy";
}
