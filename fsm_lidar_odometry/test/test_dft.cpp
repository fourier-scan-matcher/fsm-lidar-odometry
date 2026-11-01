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
#include <complex>
#include <utility>
#include <vector>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace
{
const double kTransform = 1e-11;

const double kRoundTrip = 1e-12;

std::vector<double> signal(const std::size_t n)
{
  std::vector<double> x(n);
  for (std::size_t i = 0; i < n; i++)
  {
    const double a = 2.0 * M_PI * static_cast<double>(i) / n;
    x[i] = 1.379 + 0.713 * std::sin(a) + 0.311 * std::cos(3 * a)
      - 0.157 * std::sin(5 * a);
  }
  return x;
}

std::complex<double> coefficient(const std::vector<double>& x,
  const std::size_t k)
{
  std::complex<double> sum{0.0, 0.0};
  for (std::size_t j = 0; j < x.size(); j++)
  {
    const double a =
      2.0 * M_PI * static_cast<double>((j * k) % x.size()) / x.size();
    sum += std::complex<double>(x[j] * std::cos(a), -x[j] * std::sin(a));
  }
  return sum;
}

std::vector<double> signalWithTopHarmonic(const std::size_t n)
{
  std::vector<double> x = signal(n);
  for (std::size_t i = 0; i < n; i++)
  {
    const double a = 2.0 * M_PI * static_cast<double>(i) / n;
    x[i] += 0.229 * std::sin(static_cast<double>(n / 2) * a + 0.4);
  }
  return x;
}

}

TEST(FrequencyTransform, TheForwardTransformMatchesTheDefinition)
{
  const std::size_t n = 16;
  const std::vector<double> x = signal(n);
  const std::vector<double> packed = FSM::DFTUtils::dft(x);

  ASSERT_EQ(packed.size(), n);

  for (std::size_t k = 0; k <= n / 2; k++)
  {
    const std::complex<double> expected = coefficient(x, k);

    EXPECT_NEAR(packed[k], expected.real(), kTransform)
      << "real part of coefficient " << k;

    if (k == 0 || k == n / 2)
      continue;

    EXPECT_NEAR(packed[n - k], expected.imag(), kTransform)
      << "imaginary part of coefficient " << k;
  }
}

TEST(FrequencyTransform, TheUnpackedSpectrumMirrorsItself)
{
  const std::size_t n = 16;
  const std::vector<double> x = signal(n);
  const std::vector<std::pair<double, double>> spectrum =
    FSM::DFTUtils::getDFTCoefficientsPairs(FSM::DFTUtils::dft(x));

  ASSERT_EQ(spectrum.size(), n);

  for (std::size_t k = 0; k <= n / 2; k++)
  {
    const std::complex<double> expected = coefficient(x, k);
    EXPECT_NEAR(spectrum[k].first, expected.real(), kTransform)
      << "real part of coefficient " << k;
    EXPECT_NEAR(spectrum[k].second, expected.imag(), kTransform)
      << "imaginary part of coefficient " << k;
  }

  for (std::size_t k = 1; k < n / 2; k++)
  {
    EXPECT_NEAR(spectrum[n - k].first, spectrum[k].first, kRoundTrip)
      << "mirror of coefficient " << k << ", real part";
    EXPECT_NEAR(spectrum[n - k].second, -spectrum[k].second, kRoundTrip)
      << "mirror of coefficient " << k << ", imaginary part";
  }
}

/**
 * @brief The unpacked spectrum is the one the definition gives, coefficient by
 * coefficient, up to the highest.
 * @details At an odd length the highest coefficient has an imaginary part and
 * there is no coefficient at half the length, so the spectrum cannot be
 * unpacked as at an even one. The signal carries the highest coefficient so
 * that it is not nought: at 16 it is the real one at half the length. The odd
 * lengths are those of a scan reduced to 269 rays and of the rb1 recording's
 * 1073. Measured, the spectrum misses the definition by at most 3.6e-15,
 * 2.7e-15, 2.3e-13 and 2.5e-12 at lengths 15, 16, 269 and 1073. Unpacked as
 * at an even length it came out a coefficient short at every odd length, and
 * it would drop an imaginary part of 0.229 * n * cos(0.4) / 2, 1.58 at 15,
 * derived. The bound is between the two.
 */
TEST(FrequencyTransform, TheUnpackedSpectrumMatchesTheDefinitionToItsTop)
{
  for (const std::size_t n : {15u, 16u, 269u, 1073u})
  {
    const std::vector<double> x = signalWithTopHarmonic(n);
    const std::vector<std::pair<double, double>> spectrum =
      FSM::DFTUtils::getDFTCoefficientsPairs(FSM::DFTUtils::dft(x));

    ASSERT_EQ(spectrum.size(), n) << "length " << n;

    for (std::size_t k = 0; k < n; k++)
    {
      const std::complex<double> expected = coefficient(x, k);
      EXPECT_NEAR(spectrum[k].first, expected.real(), kTransform)
        << "length " << n << ", real part of coefficient " << k;
      EXPECT_NEAR(spectrum[k].second, expected.imag(), kTransform)
        << "length " << n << ", imaginary part of coefficient " << k;
    }
  }
}

TEST(FrequencyTransform, ForwardAndInverseRoundTrip)
{
  for (const std::size_t n : {8u, 15u, 16u, 64u, 269u, 360u, 1073u})
  {
    const std::vector<double> x = signalWithTopHarmonic(n);
    const std::vector<double> back = FSM::DFTUtils::idft(
      FSM::DFTUtils::getDFTCoefficientsPairs(FSM::DFTUtils::dft(x)));

    ASSERT_EQ(back.size(), n) << "length " << n;
    for (std::size_t i = 0; i < n; i++)
      EXPECT_NEAR(back[i], x[i], kRoundTrip) << "length " << n << ", entry " << i;
  }
}

TEST(FrequencyTransform, TheFirstCoefficientShortcutAgreesWithTheTransform)
{
  const std::size_t n = 64;
  const std::vector<double> x = signal(n);

  const std::vector<double> first = FSM::DFTUtils::getFirstDFTCoefficient(x);
  const std::complex<double> expected = coefficient(x, 1);

  ASSERT_EQ(first.size(), 2u);
  EXPECT_NEAR(first[0], expected.real(), kTransform) << "real part";
  EXPECT_NEAR(first[1], expected.imag(), kTransform) << "imaginary part";
}

TEST(FrequencyTransform, ACallerSuppliedPlanMatchesTheCachedOne)
{
  const std::size_t n = 64;
  const std::vector<double> x = signal(n);

  const fftw_plan plan = FSM::DFTUtils::forwardPlan(n);

  const std::vector<double> cached = FSM::DFTUtils::dft(x);
  const std::vector<double> supplied = FSM::DFTUtils::dft(x, plan);

  ASSERT_EQ(cached.size(), supplied.size());
  for (std::size_t i = 0; i < cached.size(); i++)
    EXPECT_DOUBLE_EQ(cached[i], supplied[i]) << "entry " << i;

  const std::vector<double> first_cached =
    FSM::DFTUtils::getFirstDFTCoefficient(x);
  const std::vector<double> first_supplied =
    FSM::DFTUtils::getFirstDFTCoefficient(x, plan);

  ASSERT_EQ(first_supplied.size(), 2u);
  EXPECT_DOUBLE_EQ(first_cached[0], first_supplied[0]);
  EXPECT_DOUBLE_EQ(first_cached[1], first_supplied[1]);
}

TEST(FrequencyTransform, TheBatchTransformMatchesOneAtATime)
{
  const std::size_t n = 32;
  std::vector<std::vector<double>> scans;
  for (std::size_t s = 0; s < 4; s++)
  {
    std::vector<double> x = signal(n);
    for (double& v : x)
      v += 0.083 * static_cast<double>(s);
    scans.push_back(x);
  }

  const std::vector<std::vector<double>> batch =
    FSM::DFTUtils::dftBatch(scans);

  ASSERT_EQ(batch.size(), scans.size());
  for (std::size_t s = 0; s < scans.size(); s++)
  {
    const std::vector<double> one = FSM::DFTUtils::dft(scans[s]);
    ASSERT_EQ(batch[s].size(), one.size()) << "scan " << s;
    for (std::size_t i = 0; i < one.size(); i++)
      EXPECT_DOUBLE_EQ(batch[s][i], one[i]) << "scan " << s << ", entry " << i;
  }
}

TEST(FrequencyTransform, ShiftingTwiceRestoresTheOriginal)
{
  const std::vector<double> original{1.3, 2.7, 3.1, 4.9, 5.5, 6.2};

  std::vector<double> shifted = original;
  FSM::DFTUtils::fftshift(shifted);

  ASSERT_EQ(shifted.size(), original.size());
  EXPECT_DOUBLE_EQ(shifted[0], original[3]);

  FSM::DFTUtils::fftshift(shifted);
  for (std::size_t i = 0; i < original.size(); i++)
    EXPECT_DOUBLE_EQ(shifted[i], original[i]) << "entry " << i;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
