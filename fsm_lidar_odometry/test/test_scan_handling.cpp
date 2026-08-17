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

#include <cmath>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{
const double kExact = 1e-12;

const double kA = 5.317;
const double kB = 3.041;
const double kC = 7.628;

std::vector<double> uniformScan(const std::size_t size, const double range)
{
  return std::vector<double>(size, range);
}

}

TEST(ScanHandling, AnInteriorRunTakesTheMeanOfItsNeighbours)
{
  const std::vector<double> scan{kA, 0.0, 0.0, kB, kC, kC};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());

  const double expected = (kA + kB) / 2;
  EXPECT_NEAR(filled[1], expected, kExact);
  EXPECT_NEAR(filled[2], expected, kExact);

  EXPECT_NEAR(filled[0], kA, kExact);
  EXPECT_NEAR(filled[3], kB, kExact);
  EXPECT_NEAR(filled[4], kC, kExact);
  EXPECT_NEAR(filled[5], kC, kExact);
}

TEST(ScanHandling, ARunThatWrapsTheEndOfTheArrayIsOneRun)
{
  const std::vector<double> scan{0.0, 0.0, kB, kC, kC, kC, kA, 0.0};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());

  const double expected = (kA + kB) / 2;
  EXPECT_NEAR(filled[7], expected, kExact) << "before the wrap";
  EXPECT_NEAR(filled[0], expected, kExact) << "after the wrap";
  EXPECT_NEAR(filled[1], expected, kExact) << "after the wrap";

  EXPECT_NEAR(filled[2], kB, kExact);
  EXPECT_NEAR(filled[6], kA, kExact);
}

TEST(ScanHandling, TwoRunsAreFilledIndependently)
{
  const std::vector<double> scan{kA, 0.0, kB, kC, 0.0, 0.0, kA, kB};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  EXPECT_NEAR(filled[1], (kA + kB) / 2, kExact) << "first run";
  EXPECT_NEAR(filled[4], (kC + kA) / 2, kExact) << "second run";
  EXPECT_NEAR(filled[5], (kC + kA) / 2, kExact) << "second run";
}

TEST(ScanHandling, AScanWithNothingMissingIsUnchanged)
{
  const std::vector<double> scan{kA, kB, kC, kA, kB, kC};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  for (std::size_t i = 0; i < scan.size(); i++)
    EXPECT_NEAR(filled[i], scan[i], kExact) << "ray " << i;
}

TEST(ScanHandling, ASingleMissingRayIsFilled)
{
  const std::vector<double> scan{kA, kB, 0.0, kC, kA};
  const std::vector<double> filled =
    FSM::DatasetUtils::interpolateRanges(scan);

  ASSERT_EQ(filled.size(), scan.size());
  EXPECT_NEAR(filled[2], (kB + kC) / 2, kExact);
}

TEST(ScanHandling, AScanWithNothingMeasuredNeverReachesGapFilling)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 8;

  fsm_lidar_odometry::Matcher matcher(parameters);

  const std::vector<double> nothing(8, 0.0);
  const auto result = matcher.process(nothing);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lidar_odometry::MatchError::scan_entirely_invalid);
}

TEST(ScanHandling, AScanIsMatchedAtTheSizeItArrivesWith)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  fsm_lidar_odometry::Matcher matcher(parameters);

  EXPECT_EQ(matcher.matchSize(), 0u) << "nothing has arrived to settle it";

  const std::vector<double> first = uniformScan(90, kC);
  const auto reference = matcher.process(first);

  ASSERT_FALSE(reference.has_value());
  EXPECT_EQ(reference.error(), fsm_lidar_odometry::MatchError::no_reference_yet);
  EXPECT_EQ(matcher.matchSize(), 90u);

  const auto matched = matcher.process(uniformScan(90, kC));
  EXPECT_TRUE(matched.has_value());
}

TEST(ScanHandling, ALaterScanOfADifferentLengthIsResampledRatherThanRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 0;

  fsm_lidar_odometry::Matcher matcher(parameters);

  matcher.process(uniformScan(180, kC));
  ASSERT_EQ(matcher.matchSize(), 180u);

  const auto longer = matcher.process(uniformScan(720, kC));
  EXPECT_TRUE(longer.has_value());
  EXPECT_EQ(matcher.matchSize(), 180u) << "the size must not follow the scan";

  const auto shorter = matcher.process(uniformScan(45, kC));
  EXPECT_TRUE(shorter.has_value());
  EXPECT_EQ(matcher.matchSize(), 180u);
}

TEST(ScanHandling, AScanShorterThanTheSizeAskedForIsRefused)
{
  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = 360;

  fsm_lidar_odometry::Matcher matcher(parameters);

  const auto result = matcher.process(uniformScan(90, kC));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lidar_odometry::MatchError::scan_too_short);
  EXPECT_EQ(matcher.matchSize(), 360u) << "asked for, not settled by a scan";
}

TEST(ScanHandling, SubsamplingProducesTheRequestedNumberOfRays)
{
  const std::vector<double> scan = uniformScan(360, kC);

  const FSM::RaySearch angular = FSM::RaySearch::angular;

  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 360, angular).size(), 360u);
  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 180, angular).size(), 180u);
  EXPECT_EQ(FSM::Utils::subsampleScan(scan, 90, angular).size(), 90u);
}

TEST(ScanHandling, SubsamplingByAWholeFactorKeepsTheRangesExactly)
{
  const std::vector<double> scan = uniformScan(360, kC);
  const std::vector<double> smaller =
    FSM::Utils::subsampleScan(scan, 90, FSM::RaySearch::angular);

  ASSERT_EQ(smaller.size(), 90u);
  for (std::size_t i = 0; i < smaller.size(); i++)
    EXPECT_NEAR(smaller[i], kC, 1e-8) << "ray " << i;
}

TEST(ScanHandling, SubsamplingByAFractionFallsShortAndNeverLong)
{
  const std::vector<double> scan = uniformScan(360, kC);
  const std::vector<double> smaller =
    FSM::Utils::subsampleScan(scan, 100, FSM::RaySearch::angular);

  ASSERT_EQ(smaller.size(), 100u);

  const double step = 2 * M_PI / 360;
  const double deepest = kC * (1 - std::cos(step / 2));

  for (std::size_t i = 0; i < smaller.size(); i++)
  {
    EXPECT_LE(smaller[i], kC + 1e-8) << "ray " << i << " reaches too far";
    EXPECT_GE(smaller[i], kC - deepest - 1e-8) << "ray " << i << " falls short";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
