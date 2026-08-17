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

#include "fsm_lidar_odometry/fsm_core.hpp"

namespace
{
const double kAngle = 0.7853981633974483;
const double kTightTolerance = 1e-12;

Eigen::Matrix3d rotationOnly(const double& angle)
{
  return FSM::Utils::computeTransform(
    FSM::Pose{0.0, 0.0, angle}, Eigen::Matrix3d::Identity());
}

}

TEST(ComputeTransform, TransformIsAProperRotation)
{
  const Eigen::Matrix3d m = rotationOnly(kAngle);
  const Eigen::Matrix2d r = m.topLeftCorner<2, 2>();
  const Eigen::Matrix2d should_be_identity = r.transpose() * r;

  EXPECT_NEAR(should_be_identity(0, 0), 1.0, kTightTolerance);
  EXPECT_NEAR(should_be_identity(1, 1), 1.0, kTightTolerance);
  EXPECT_NEAR(should_be_identity(0, 1), 0.0, kTightTolerance);
  EXPECT_NEAR(should_be_identity(1, 0), 0.0, kTightTolerance);
  EXPECT_NEAR(r.determinant(), 1.0, kTightTolerance);
}

TEST(ComputeTransform, TransformMatchesDoublePrecisionTrig)
{
  const Eigen::Matrix3d m = rotationOnly(kAngle);

  EXPECT_NEAR(m(0, 0), std::cos(kAngle), kTightTolerance);
  EXPECT_NEAR(m(1, 1), std::cos(kAngle), kTightTolerance);
  EXPECT_NEAR(m(1, 0), std::sin(kAngle), kTightTolerance);
  EXPECT_NEAR(m(0, 1), -std::sin(kAngle), kTightTolerance);
}

TEST(ComputeTransform, RecoveredAngleSurvivesRepeatedComposition)
{
  const double step = 0.01;
  const unsigned int steps = 200;

  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  for (unsigned int i = 0; i < steps; i++)
    m = FSM::Utils::computeTransform(FSM::Pose{0.0, 0.0, step}, m);

  EXPECT_NEAR(std::atan2(m(1, 0), m(0, 0)), steps * step, kTightTolerance);
}

TEST(ComputeTransform, TranslationIsUnaffectedByOrientation)
{
  const Eigen::Matrix3d m = FSM::Utils::computeTransform(
    FSM::Pose{1.25, -0.5, kAngle}, Eigen::Matrix3d::Identity());

  EXPECT_NEAR(m(0, 2), 1.25, kTightTolerance);
  EXPECT_NEAR(m(1, 2), -0.5, kTightTolerance);
}

TEST(ScanHandling, GapFillingAcceptsAScanWithNoInvalidReturns)
{
  const std::vector<double> clean(360, 3.0);
  const std::vector<double> filled = FSM::DatasetUtils::interpolateRanges(clean);

  ASSERT_EQ(filled.size(), clean.size());
  for (unsigned int i = 0; i < clean.size(); i++)
    EXPECT_EQ(filled[i], clean[i]);
}

TEST(ScanHandling, GapFillingStillFillsAnInteriorRun)
{
  std::vector<double> ranges(360, 3.0);
  for (unsigned int i = 100; i < 110; i++)
    ranges[i] = 0.0;

  const std::vector<double> filled = FSM::DatasetUtils::interpolateRanges(ranges);

  ASSERT_EQ(filled.size(), ranges.size());
  for (unsigned int i = 100; i < 110; i++)
    EXPECT_EQ(filled[i], 3.0);
}

TEST(TranslationStage, FirstCoefficientNormIsDoublePrecision)
{
  const unsigned int size = 360;

  std::vector<double> real_scan(size, 0.0);
  std::vector<double> virtual_scan(size, 0.0);
  for (unsigned int i = 0; i < size; i++)
  {
    const double angle = -M_PI + i * 2 * M_PI / size;
    real_scan[i] = 3.0
      + 0.372131 * std::cos(angle)
      + 0.153907 * std::sin(angle)
      + 0.041113 * std::sin(2 * angle);
    virtual_scan[i] = 3.0
      + 0.319717 * std::cos(angle)
      + 0.122803 * std::sin(angle)
      + 0.037619 * std::sin(2 * angle);
  }

  double* in = static_cast<double*>(fftw_malloc(size * sizeof(double)));
  double* out = static_cast<double*>(fftw_malloc(size * sizeof(double)));
  const fftw_plan r2rp =
    fftw_plan_r2r_1d(size, in, out, FFTW_R2HC, FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);
  fftw_free(in);
  fftw_free(out);

  const FSM::TranslationCorrection correction =
    FSM::Translation::tffCore(real_scan, virtual_scan, 0.0, 1000.0, r2rp);
  const double norm_x1 = correction.norm_x1;

  const auto [diff, d_v_expected] =
    FSM::Utils::diffScansPerRay(real_scan, virtual_scan, 1000.0);
  const std::vector<double> x1 =
    FSM::DFTUtils::getFirstDFTCoefficient(diff, r2rp);
  const double expected = std::sqrt(x1[0] * x1[0] + x1[1] * x1[1]);

  fftw_destroy_plan(r2rp);

  ASSERT_GT(expected, 0.0);
  EXPECT_NEAR(norm_x1 / expected, 1.0, kTightTolerance);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
