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

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{
const double kExact = 1e-12;

const double kFarPointCancellation = 1e-8;

using Point = std::pair<double, double>;
using Room = std::vector<Point>;

const FSM::RaySearch kAngular = FSM::RaySearch::angular;
const FSM::RaySearch kWindowed = FSM::RaySearch::windowed;

Room rectangle(const double half_width, const double half_height)
{
  return Room{
    {-half_width, -half_height},
    {half_width, -half_height},
    {half_width, half_height},
    {-half_width, half_height}};
}

void expectPoint(const Point& got, const double x, const double y,
  const std::string& which)
{
  EXPECT_NEAR(got.first, x, kExact) << which << ", x";
  EXPECT_NEAR(got.second, y, kExact) << which << ", y";
}

double distance(const Point& from, const Point& to)
{
  const double dx = to.first - from.first;
  const double dy = to.second - from.second;
  return std::sqrt(dx * dx + dy * dy);
}

}

TEST(Geometry, AxisAlignedRaysFromTheCentreOfASquare)
{
  const Room room = rectangle(4.0, 4.0);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 4, kAngular);

  ASSERT_EQ(hits.size(), 4u);
  expectPoint(hits[0], -4.0, 0.0, "ray at -pi");
  expectPoint(hits[1], 0.0, -4.0, "ray at -pi/2");
  expectPoint(hits[2], 4.0, 0.0, "ray at 0");
  expectPoint(hits[3], 0.0, 4.0, "ray at pi/2");
}

TEST(Geometry, APoseCloseToAWall)
{
  const Room room = rectangle(4.0, 4.0);
  const FSM::Pose pose{3.5, 0.0, 0.0};
  const std::vector<Point> hits = FSM::X::find(pose, room, 4, kAngular);

  ASSERT_EQ(hits.size(), 4u);
  expectPoint(hits[0], -4.0, 0.0, "away from the near wall");
  expectPoint(hits[2], 4.0, 0.0, "into the near wall");

  EXPECT_NEAR(distance({pose.x, pose.y}, hits[2]), 0.5, kExact);
  EXPECT_NEAR(distance({pose.x, pose.y}, hits[0]), 7.5, kExact);
}

TEST(Geometry, ARayThroughACorner)
{
  const Room room = rectangle(4.0, 4.0);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 8, kAngular);

  ASSERT_EQ(hits.size(), 8u);
  expectPoint(hits[1], -4.0, -4.0, "ray at -3pi/4");
  expectPoint(hits[3], 4.0, -4.0, "ray at -pi/4");
  expectPoint(hits[5], 4.0, 4.0, "ray at pi/4");
  expectPoint(hits[7], -4.0, 4.0, "ray at 3pi/4");
}

TEST(Geometry, AnObliqueRayInARectangularRoom)
{
  const Room room = rectangle(4.0, 2.5);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 12, kAngular);

  ASSERT_EQ(hits.size(), 12u);

  const double expected_y = 4.0 * std::tan(M_PI / 6);
  ASSERT_LT(expected_y, 2.5) << "the case is only oblique if it clears the top";

  EXPECT_NEAR(hits[7].first, 4.0, kExact) << "ray at pi/6, x";
  EXPECT_NEAR(hits[7].second, expected_y, kFarPointCancellation)
    << "ray at pi/6, y";
}

TEST(Geometry, TurningThePoseByOneRayShiftsTheIntersections)
{
  const Room room = rectangle(4.0, 2.5);
  const unsigned int rays = 16;

  const std::vector<Point> straight =
    FSM::X::find(FSM::Pose{}, room, rays, kAngular);
  const std::vector<Point> turned =
    FSM::X::find(FSM::Pose{0.0, 0.0, 2 * M_PI / rays}, room, rays,
      kAngular);

  ASSERT_EQ(straight.size(), rays);
  ASSERT_EQ(turned.size(), rays);

  for (unsigned int i = 0; i < rays - 1; i++)
    expectPoint(turned[i], straight[i + 1].first, straight[i + 1].second,
      "ray " + std::to_string(i));
}

TEST(Geometry, BothRaySearchesAgreeWithTheExhaustiveOneInAConvexRoom)
{
  const Room room = rectangle(4.0, 2.5);

  for (const FSM::RaySearch ray_search : {kAngular, kWindowed})
  {
    for (const FSM::Pose& pose : {FSM::Pose{}, FSM::Pose{1.5, -0.5, 0.7},
        FSM::Pose{-2.0, 1.0, -2.2}, FSM::Pose{3.9, 2.4, 3.0}})
    {
      const std::vector<Point> hits =
        FSM::X::find(pose, room, 90, ray_search);
      const std::vector<Point> exhaustive = FSM::X::findExact(pose, room, 90);

      ASSERT_EQ(hits.size(), exhaustive.size());
      for (std::size_t i = 0; i < hits.size(); i++)
        expectPoint(hits[i], exhaustive[i].first, exhaustive[i].second,
          "ray " + std::to_string(i));
    }
  }
}

Room wedgeRoom()
{
  return Room{
    {-4.0, -2.5}, {0.0, -3.5}, {4.0, -2.5}, {4.0, 2.5},
    {1.0, 1.0}, {-1.0, 2.5}, {-4.0, 2.5}};
}

TEST(Geometry, TheAngularRaySearchFindsTheNearestWallInAConcaveRoom)
{
  const Room room = wedgeRoom();

  for (const FSM::Pose& pose : {FSM::Pose{-2.0, 1.0, -2.2},
      FSM::Pose{}, FSM::Pose{2.5, -1.0, 1.3}, FSM::Pose{-3.0, -1.5, 2.9}})
  {
    const std::vector<Point> hits = FSM::X::find(pose, room, 90, kAngular);
    const std::vector<Point> exhaustive = FSM::X::findExact(pose, room, 90);

    ASSERT_EQ(hits.size(), exhaustive.size());
    for (std::size_t i = 0; i < hits.size(); i++)
      expectPoint(hits[i], exhaustive[i].first, exhaustive[i].second,
        "ray " + std::to_string(i));
  }
}

TEST(Geometry, TheWindowedRaySearchSettlesForAWallBehindTheNearestOne)
{
  const Room room = wedgeRoom();
  const FSM::Pose pose{-2.0, 1.0, -2.2};

  const std::vector<Point> windowed = FSM::X::find(pose, room, 90, kWindowed);
  const std::vector<Point> exhaustive = FSM::X::findExact(pose, room, 90);

  ASSERT_EQ(windowed.size(), exhaustive.size());

  std::size_t wrong = 0;
  double worst = 0.0;

  for (std::size_t i = 0; i < windowed.size(); i++)
  {
    const double off = distance(windowed[i], exhaustive[i]);
    if (off > kExact)
    {
      wrong++;
      worst = std::max(worst, off);
    }
  }

  EXPECT_EQ(wrong, 4u);
  EXPECT_NEAR(worst, 3.86, 0.01);
}

TEST(Geometry, TheTwoRaySearchesDisagreeWhereTheRoomTurnsBackOnItself)
{
  const Room room = wedgeRoom();
  const FSM::Pose pose{-2.0, 1.0, -2.2};

  const std::vector<Point> angular = FSM::X::find(pose, room, 90, kAngular);
  const std::vector<Point> windowed = FSM::X::find(pose, room, 90, kWindowed);

  ASSERT_EQ(angular.size(), windowed.size());
  EXPECT_NE(angular, windowed);
}

TEST(Geometry, TheRaySearchMatchesTheExhaustiveOneOverManyRandomRooms)
{
  std::mt19937 generator(20260814);
  std::uniform_real_distribution<double> radius(0.8, 6.0);
  std::uniform_real_distribution<double> offset(-0.4, 0.4);
  std::uniform_real_distribution<double> orientation(-M_PI, M_PI);

  std::size_t compared = 0;

  for (int room_number = 0; room_number < 100; room_number++)
  {
    Room room;
    for (int corner = 0; corner < 60; corner++)
    {
      const double angle = corner * 2 * M_PI / 60;
      const double r = radius(generator);
      room.push_back({r * std::cos(angle), r * std::sin(angle)});
    }

    for (int attempt = 0; attempt < 10; attempt++)
    {
      const FSM::Pose pose{offset(generator), offset(generator),
        orientation(generator)};

      const std::vector<Point> hits = FSM::X::find(pose, room, 90, kAngular);
      const std::vector<Point> exhaustive = FSM::X::findExact(pose, room, 90);

      ASSERT_EQ(hits.size(), exhaustive.size());
      for (std::size_t i = 0; i < hits.size(); i++)
      {
        ASSERT_EQ(hits[i].first, exhaustive[i].first)
          << "room " << room_number << ", pose " << attempt << ", ray " << i;
        ASSERT_EQ(hits[i].second, exhaustive[i].second)
          << "room " << room_number << ", pose " << attempt << ", ray " << i;
      }

      compared += hits.size();
    }
  }

  EXPECT_EQ(compared, 90000u);
}

TEST(Geometry, PointsAndRangesRoundTrip)
{
  const Room room = rectangle(4.0, 2.5);
  const FSM::Pose pose{1.25, -0.75, 0.4};

  const std::vector<double> ranges =
    FSM::Utils::points2scan(FSM::X::find(pose, room, 180, kAngular), pose);

  ASSERT_EQ(ranges.size(), 180u);

  const std::vector<Point> back = FSM::Utils::scan2points(ranges, pose);
  const std::vector<double> again = FSM::Utils::points2scan(back, pose);

  ASSERT_EQ(again.size(), ranges.size());
  for (std::size_t i = 0; i < ranges.size(); i++)
    EXPECT_NEAR(again[i], ranges[i], kExact) << "ray " << i;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
