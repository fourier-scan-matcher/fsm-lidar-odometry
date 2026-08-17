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
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{
const std::size_t kSize = 360;

const double kStep = 2 * M_PI / kSize;

std::string fixture(const std::string& name)
{
  return std::string(FSM_LIDAR_ODOMETRY_TEST_FIXTURES) + "/" + name;
}

std::vector<std::vector<double>> readScans(const std::string& name)
{
  std::ifstream file(fixture(name));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(name);

  std::size_t count = 0;
  std::size_t size = 0;
  file >> count >> size;

  std::vector<std::vector<double>> scans(count, std::vector<double>(size));
  for (std::size_t s = 0; s < count; s++)
    for (std::size_t i = 0; i < size; i++)
      file >> scans[s][i];

  return scans;
}

std::vector<FSM::Pose> readTruth(const std::string& name)
{
  std::ifstream file(fixture(name));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(name);

  std::string line;
  std::getline(file, line);

  std::vector<FSM::Pose> poses;
  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    for (char& c : line)
      if (c == ',')
        c = ' ';

    std::istringstream fields(line);
    double index = 0.0;
    double stamp = 0.0;
    FSM::Pose pose;
    fields >> index >> stamp >> pose.x >> pose.y >> pose.t;
    poses.push_back(pose);
  }
  return poses;
}

std::vector<std::pair<double, double>> mapOf(const std::vector<double>& scan)
{
  return FSM::Utils::scan2points(scan, FSM::Pose{});
}

double closestTo(const std::vector<double>& candidates, const double target)
{
  double best = candidates.at(0);
  for (const double candidate : candidates)
    if (std::fabs(candidate - target) < std::fabs(best - target))
      best = candidate;
  return best;
}

}

TEST(RotationStage, RecoversATurnToTheResolutionItHas)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_rotation_scans.csv");
  const std::vector<FSM::Pose> truth = readTruth("pure_rotation_truth.csv");

  ASSERT_EQ(scans.size(), truth.size());
  ASSERT_GT(scans.size(), 1u);

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);
  const fftw_plan inverse = FSM::DFTUtils::inversePlan(kSize);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const double turn = truth[s].t - truth[s - 1].t;
    ASSERT_GT(std::fabs(turn), 0.0) << "pair " << s << " does not turn";

    for (unsigned int magnification = 0; magnification <= 3; magnification++)
    {
      const FSM::RotationOutput output = FSM::Rotation::fmt(scans[s],
        FSM::Pose{}, mapOf(scans[s - 1]), magnification, "batch", forward,
        inverse, FSM::RaySearch::angular);

      ASSERT_FALSE(output.angles.empty())
        << "pair " << s << " at magnification " << magnification;

      const double resolution = kStep / (1u << magnification);
      const double error = std::fabs(closestTo(output.angles, turn) - turn);

      EXPECT_LE(error, resolution)
        << "pair " << s << " at magnification " << magnification
        << ": closest candidate is further than one step from the truth";
    }
  }
}

TEST(RotationStage, MagnificationDoesNotMakeTheAnswerWorse)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_rotation_scans.csv");
  const std::vector<FSM::Pose> truth = readTruth("pure_rotation_truth.csv");

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);
  const fftw_plan inverse = FSM::DFTUtils::inversePlan(kSize);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const double turn = truth[s].t - truth[s - 1].t;
    const std::vector<std::pair<double, double>> map = mapOf(scans[s - 1]);

    const FSM::RotationOutput coarse = FSM::Rotation::fmt(scans[s], FSM::Pose{},
      map, 0, "batch", forward, inverse, FSM::RaySearch::angular);
    const FSM::RotationOutput fine = FSM::Rotation::fmt(scans[s], FSM::Pose{},
      map, 3, "batch", forward, inverse, FSM::RaySearch::angular);

    ASSERT_FALSE(coarse.angles.empty());
    ASSERT_FALSE(fine.angles.empty());

    EXPECT_LE(std::fabs(closestTo(fine.angles, turn) - turn),
      std::fabs(closestTo(coarse.angles, turn) - turn) + 1e-12)
      << "pair " << s;
  }
}

TEST(RotationStage, FindsNoTurnWhereThereIsNone)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_translation_scans.csv");

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);
  const fftw_plan inverse = FSM::DFTUtils::inversePlan(kSize);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    for (unsigned int magnification = 0; magnification <= 2; magnification++)
    {
      const FSM::RotationOutput output = FSM::Rotation::fmt(scans[s],
        FSM::Pose{}, mapOf(scans[s - 1]), magnification, "batch", forward,
        inverse, FSM::RaySearch::angular);

      ASSERT_FALSE(output.angles.empty());
      EXPECT_NEAR(closestTo(output.angles, 0.0), 0.0, 1e-12)
        << "pair " << s << " at magnification " << magnification;
    }
  }
}

TEST(TranslationStage, RecoversAMoveGivenEnoughIterations)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_translation_scans.csv");
  const std::vector<FSM::Pose> truth = readTruth("pure_translation_truth.csv");

  ASSERT_EQ(scans.size(), truth.size());

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const FSM::TranslationOutput output = FSM::Translation::tff(scans[s],
      FSM::Pose{}, mapOf(scans[s - 1]), 32, 0.2, true, forward,
      FSM::RaySearch::angular);

    EXPECT_NEAR(output.pose.x, truth[s].x - truth[s - 1].x, 1e-3)
      << "pair " << s << ", along the direction of travel";
    EXPECT_NEAR(output.pose.y, truth[s].y - truth[s - 1].y, 1e-3)
      << "pair " << s << ", across it";
  }
}

TEST(TranslationStage, MoreIterationsDoNotMakeTheAnswerWorse)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_translation_scans.csv");
  const std::vector<FSM::Pose> truth = readTruth("pure_translation_truth.csv");

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const std::vector<std::pair<double, double>> map = mapOf(scans[s - 1]);
    const double expected = truth[s].x - truth[s - 1].x;

    const FSM::TranslationOutput few = FSM::Translation::tff(scans[s],
      FSM::Pose{}, map, 4, 0.2, true, forward, FSM::RaySearch::angular);
    const FSM::TranslationOutput many = FSM::Translation::tff(scans[s],
      FSM::Pose{}, map, 32, 0.2, true, forward, FSM::RaySearch::angular);

    EXPECT_LE(std::fabs(many.pose.x - expected),
      std::fabs(few.pose.x - expected)) << "pair " << s;

    EXPECT_LE(many.iterations, 33) << "pair " << s << ": ran past its limit";
  }
}

TEST(TranslationStage, LeavesTheOrientationAsItFoundIt)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_translation_scans.csv");

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);
  const std::vector<std::pair<double, double>> map = mapOf(scans[0]);

  for (const double orientation : {0.0, 0.3, -1.25, 3.0})
  {
    const FSM::Pose given{0.0, 0.0, orientation};
    const FSM::TranslationOutput output =
      FSM::Translation::tff(scans[1], given, map, 8, 0.2, true, forward,
        FSM::RaySearch::angular);

    EXPECT_DOUBLE_EQ(output.pose.t, orientation);
  }
}

TEST(TranslationStage, MistakesATurnForAMove)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_rotation_scans.csv");
  const std::vector<FSM::Pose> truth = readTruth("pure_rotation_truth.csv");

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);

  const FSM::TranslationOutput output = FSM::Translation::tff(scans[1],
    FSM::Pose{}, mapOf(scans[0]), 32, 0.2, true, forward,
    FSM::RaySearch::angular);

  EXPECT_DOUBLE_EQ(truth[1].x, truth[0].x) << "the trajectory does not move";
  EXPECT_DOUBLE_EQ(truth[1].y, truth[0].y) << "the trajectory does not move";

  const double reported = std::hypot(output.pose.x, output.pose.y);
  EXPECT_GT(reported, 1e-3)
    << "a turn used to be visible to this stage as a move; if it no longer is, "
       "the input or the stage has changed";
}

TEST(Diagnostics, TheCoreReportsThroughWhateverTheHostInstalls)
{
  const std::vector<std::vector<double>> scans =
    readScans("pure_rotation_scans.csv");
  ASSERT_GE(scans.size(), 2u);

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(kSize);
  const fftw_plan inverse = FSM::DFTUtils::inversePlan(kSize);
  const std::vector<std::pair<double, double>> map = mapOf(scans[0]);

  FSM::Rotation::fmt(scans[1], FSM::Pose{}, map, 0, "spiral", forward, inverse,
    FSM::RaySearch::angular);

  std::vector<std::string> reported;
  fsm_lidar_odometry::setDiagnosticSink(
    [&reported](const std::string& message) { reported.push_back(message); });

  FSM::Rotation::fmt(scans[1], FSM::Pose{}, map, 0, "spiral", forward, inverse,
    FSM::RaySearch::angular);

  fsm_lidar_odometry::setDiagnosticSink(nullptr);

  ASSERT_EQ(reported.size(), 1u);
  EXPECT_NE(reported[0].find("batch"), std::string::npos) << reported[0];
  EXPECT_EQ(reported[0].back() != '\n', true)
    << "the sink decides how a line ends, not the core";

  FSM::Rotation::fmt(scans[1], FSM::Pose{}, map, 0, "spiral", forward, inverse,
    FSM::RaySearch::angular);
  EXPECT_EQ(reported.size(), 1u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
