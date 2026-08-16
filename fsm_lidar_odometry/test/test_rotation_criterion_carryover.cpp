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

/**
 * @brief Characterisation test for an out of bounds read in the matcher's
 * main loop.
 *
 * Match::fmtdbh keeps a running best candidate angle across the while loop's
 * iterations. Each iteration appends that angle to the rotation stage's own
 * candidates when it is not already among them, sifts the whole list by
 * translation criterion, and then reads the winning candidate's pair of
 * rotation criteria straight out of the rotation stage's two output vectors
 * by index. Those two vectors carry one entry per candidate the rotation
 * stage itself returned, with no entry for the carried-over angle appended
 * afterwards, so the read runs one past the end of both vectors whenever that
 * appended angle wins the sift, which happens whenever the previous best
 * beats every angle the rotation stage found this time round.
 *
 * This is built with -D_GLIBCXX_ASSERTIONS so libstdc++'s own bounds check on
 * std::vector::operator[] is what catches the defect, rather than relying on
 * a sanitiser or on the read happening to land somewhere that changes the
 * result. Before the correction this aborts partway through one of the
 * scenarios below; after it, every scenario completes and every reported pose
 * is finite. The library under test is linked directly against no other
 * compiled object, so this file's own compile flags are what govern the code
 * actually run, unlike a build that pulls the matcher in from a shared
 * library built with different settings.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "fsm_lidar_odometry/fsm_core.hpp"

namespace
{

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

FSM::input_params defaultParams()
{
  FSM::input_params ip;
  ip.num_iterations = 2;
  ip.xy_bound = 0.2;
  ip.t_bound = M_PI / 4;
  ip.max_counter = 200;
  ip.min_magnification_size = 0;
  ip.max_magnification_size = 3;
  ip.max_recoveries = 10;
  ip.rng_seed = 1;
  ip.ray_search = FSM::RaySearch::angular;
  return ip;
}

/*
 * Matches every later scan in a fixture against the first one, rather than
 * against its immediate predecessor. The first scan of each fixture stands as
 * a fixed map, and every scan after it is asked for directly, so the
 * displacement the matcher has to recover grows with each one rather than
 * staying at one small step. Chaining consecutive scans instead, the way the
 * odometry wrapper does, never reproduced the defect: each step is close
 * enough to the last that the rotation stage finds a genuine candidate ahead
 * of the carried-over one at every magnification, so the carried-over entry
 * never wins the sift. Asking for a larger displacement in one go stresses
 * exactly the path that does.
 */
void driveScenario(const std::string& scenario)
{
  const std::vector<std::vector<double>> scans =
    readScans(scenario + "_scans.csv");
  ASSERT_GE(scans.size(), 2u) << scenario;

  const fftw_plan forward = FSM::DFTUtils::forwardPlan(scans[0].size());
  const fftw_plan inverse = FSM::DFTUtils::inversePlan(scans[0].size());
  const FSM::input_params ip = defaultParams();

  const std::vector<std::pair<double, double>> map =
    FSM::Utils::scan2points(scans[0], FSM::Pose{});

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const FSM::MatchOutput match =
      FSM::Match::fmtdbh(scans[s], FSM::Pose{}, map, forward, inverse, ip);

    EXPECT_TRUE(std::isfinite(match.pose.x)) << scenario << " step " << s;
    EXPECT_TRUE(std::isfinite(match.pose.y)) << scenario << " step " << s;
    EXPECT_TRUE(std::isfinite(match.pose.t)) << scenario << " step " << s;
  }
}

}  // namespace

TEST(RotationCriterionCarryover, PureRotationStaysWithinBounds)
{
  driveScenario("pure_rotation");
}

TEST(RotationCriterionCarryover, PureTranslationStaysWithinBounds)
{
  driveScenario("pure_translation");
}

TEST(RotationCriterionCarryover, RectangularRoomStaysWithinBounds)
{
  driveScenario("rect_room_short");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
