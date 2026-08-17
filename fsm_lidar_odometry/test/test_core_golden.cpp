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
const double kTolerance = 1e-9;

struct Reference
{
  std::vector<fsm_lidar_odometry::Pose> increments;
  std::vector<fsm_lidar_odometry::Pose> accumulated;
  std::vector<unsigned int> recoveries;
};

std::string fixture(const std::string& name)
{
  return std::string(FSM_LIDAR_ODOMETRY_TEST_FIXTURES) + "/" + name;
}

std::vector<std::vector<double>> readScans(const std::string& path)
{
  std::ifstream file(fixture(path));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(path);

  std::size_t count = 0;
  std::size_t size = 0;
  file >> count >> size;

  std::vector<std::vector<double>> scans(count, std::vector<double>(size));
  for (std::size_t s = 0; s < count; s++)
    for (std::size_t i = 0; i < size; i++)
      file >> scans[s][i];

  return scans;
}

Reference readReference(const std::string& path)
{
  std::ifstream file(fixture(path));
  EXPECT_TRUE(file.good()) << "cannot open " << fixture(path);

  Reference reference;

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);

    unsigned int index = 0;
    fsm_lidar_odometry::Pose increment;
    fsm_lidar_odometry::Pose accumulated;
    unsigned int recoveries = 0;

    values >> index
           >> increment.x >> increment.y >> increment.t
           >> accumulated.x >> accumulated.y >> accumulated.t
           >> recoveries;

    reference.increments.push_back(increment);
    reference.accumulated.push_back(accumulated);
    reference.recoveries.push_back(recoveries);
  }

  return reference;
}

void expectAgreement(const std::string& scenario)
{
  const std::vector<std::vector<double>> scans =
    readScans(scenario + "_scans.csv");
  const Reference reference = readReference(scenario + "_core.csv");

  ASSERT_GE(scans.size(), 2u) << scenario;
  ASSERT_EQ(reference.increments.size(), scans.size() - 1) << scenario;

  fsm_lidar_odometry::Parameters parameters;
  parameters.size_scan = scans[0].size();

  fsm_lidar_odometry::Matcher matcher(parameters);

  const std::expected<fsm_lidar_odometry::MatchResult, fsm_lidar_odometry::MatchError> first =
    matcher.process(scans[0]);
  ASSERT_FALSE(first.has_value()) << "the first scan must yield no match";
  EXPECT_EQ(first.error(), fsm_lidar_odometry::MatchError::no_reference_yet);

  for (std::size_t s = 1; s < scans.size(); s++)
  {
    const std::expected<fsm_lidar_odometry::MatchResult, fsm_lidar_odometry::MatchError> result =
      matcher.process(scans[s]);

    ASSERT_TRUE(result.has_value()) << scenario << " step " << s;

    const std::size_t step = s - 1;

    EXPECT_NEAR(result->increment.x, reference.increments[step].x, kTolerance)
      << scenario << " step " << s << ", increment x";
    EXPECT_NEAR(result->increment.y, reference.increments[step].y, kTolerance)
      << scenario << " step " << s << ", increment y";
    EXPECT_NEAR(result->increment.t, reference.increments[step].t, kTolerance)
      << scenario << " step " << s << ", increment orientation";

    EXPECT_NEAR(result->accumulated.x, reference.accumulated[step].x, kTolerance)
      << scenario << " step " << s << ", accumulated x";
    EXPECT_NEAR(result->accumulated.y, reference.accumulated[step].y, kTolerance)
      << scenario << " step " << s << ", accumulated y";
    EXPECT_NEAR(result->accumulated.t, reference.accumulated[step].t, kTolerance)
      << scenario << " step " << s << ", accumulated orientation";

    EXPECT_EQ(result->num_recoveries, reference.recoveries[step])
      << scenario << " step " << s << ", recovery count";
    EXPECT_EQ(result->num_recoveries, 0u)
      << scenario << " step " << s
      << ": the recovery path fired, which makes this scenario unusable for "
         "cross version comparison";
  }
}

}

TEST(CoreGolden, PureRotation)
{
  expectAgreement("pure_rotation");
}

TEST(CoreGolden, PureTranslation)
{
  expectAgreement("pure_translation");
}

TEST(CoreGolden, RectangularRoom)
{
  expectAgreement("rect_room_short");
}
