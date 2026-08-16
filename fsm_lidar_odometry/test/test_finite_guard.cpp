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
 * @brief Characterisation test for the finite checks the transform stage
 * relies on to keep a non finite Fourier coefficient out of the rotation
 * estimate.
 *
 * DFTUtils::getFirstDFTCoefficient replaces a non finite real or imaginary
 * part of the first coefficient with zero, guarded by a check on the value.
 * This package ships compiled with -Ofast, which implies -ffinite-math-only,
 * under which the compiler is entitled to assume no infinity or not-a-number
 * value ever occurs and is therefore free to fold a call to std::isfinite
 * down to a constant true. The guard was written with exactly that call, so
 * in the configuration this package ships it did nothing: a non finite
 * coefficient passed straight through.
 *
 * This file is deliberately not built with the exact floating point settings
 * the rest of the suite forces, and is not linked against the shared library.
 * It is compiled with whatever the package itself is configured with, which
 * by default is the shipped -Ofast, so what runs here is the same code the
 * published binary runs rather than a safer stand-in for it. Feeding a
 * not-a-number range straight into a real scan makes every coefficient of its
 * transform not-a-number too, since a discrete Fourier transform is a linear
 * combination of every sample, so the effect on the first coefficient is
 * whatever finite check the transform stage happens to be running that build.
 *
 * The result is read back by comparing bit patterns rather than with a plain
 * double comparison. Measured directly: under this file's own -Ofast, an
 * unreplaced not-a-number compares equal to 0.0 with the ordinary == operator,
 * because -ffinite-math-only licenses the compiler to assume neither operand
 * of a floating point comparison is ever not-a-number, the same assumption
 * that folds the guard itself. A comparison written in the same optimisation
 * unit as the value under test cannot be trusted to see it honestly, so the
 * bits are compared as the integers they are instead.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "fsm_lidar_odometry/fsm_core.hpp"

namespace
{

std::uint64_t bitsOf(const double value)
{
  return std::bit_cast<std::uint64_t>(value);
}

}  // namespace

TEST(FiniteGuard, NonFiniteCoefficientIsZeroedInTheShippedBuild)
{
  const std::size_t size = 64;
  std::vector<double> rays_diff(size, 0.3);
  rays_diff[0] = std::numeric_limits<double>::quiet_NaN();

  const std::vector<double> coefficient =
    FSM::DFTUtils::getFirstDFTCoefficient(rays_diff);

  ASSERT_EQ(coefficient.size(), 2u);
  EXPECT_EQ(bitsOf(coefficient[0]), bitsOf(0.0))
    << "the real part of a non finite coefficient must be replaced by zero";
  EXPECT_EQ(bitsOf(coefficient[1]), bitsOf(0.0))
    << "the imaginary part of a non finite coefficient must be replaced by "
       "zero";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
