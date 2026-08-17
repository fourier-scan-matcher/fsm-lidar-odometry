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

#ifndef FSM_LIDAR_ODOMETRY__FSM_LIDAR_ODOMETRY_HPP_
#define FSM_LIDAR_ODOMETRY__FSM_LIDAR_ODOMETRY_HPP_

#include <cstddef>
#include <expected>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fsm_lidar_odometry/fsm_core.hpp"

namespace fsm_lidar_odometry
{
/**
 * @brief A planar pose, or a planar displacement between two poses.
 * The core carries the same type, so this is an alias rather than a second
 * declaration: the two layers hand poses to each other unconverted.
 */
using Pose = FSM::Pose;

/**
 * @brief Everything the matcher needs to know, and nothing about ROS.
 */
struct Parameters
{
  std::size_t size_scan{0};
  unsigned int num_iterations{2};
  double xy_bound{0.2};
  double t_bound{M_PI / 4};
  unsigned int max_counter{200};
  unsigned int min_magnification_size{0};
  unsigned int max_magnification_size{3};
  unsigned int max_recoveries{10};

  unsigned int rng_seed{0};

  std::string ray_search{"angular"};
};

/**
 * @brief What one successful match yields.
 */
struct MatchResult
{
  Pose increment;

  Pose accumulated;

  double execution_time{0.0};
  unsigned int num_recoveries{0};
};

/**
 * @brief Why a scan produced no match.
 */
enum class MatchError
{
  no_reference_yet,

  scan_too_short,

  scan_entirely_invalid,
};

/**
 * @brief Whether a range reading carries a usable distance.
 * A lidar reports a ray that struck nothing, or a reading it could not trust,
 * in one of three ways: zero, infinity, or not-a-number. Which of the three it
 * picks is a matter of driver convention. All three mean the same thing here,
 * and are filled in from the rays around them before matching.
 */
bool isValidRange(double range);

/**
 * @brief Validate a parameter set.
 * @return an empty string when the parameters are usable, otherwise a sentence
 * naming the offending parameter and its value.
 */
std::string validate(const Parameters& parameters);

/**
 * @brief Install a destination for the matching core's diagnostics.
 * The core reports in plain strings and does not grade them, having no way to
 * know what a host considers serious. Where no destination is installed the
 * reports are dropped, which is what a library with nobody listening should
 * do.
 * Nearly everything the core has to say is stage timing, and that is compiled
 * out unless the core is built with FSM_LIDAR_ODOMETRY_TRACE. An ordinary build reports
 * only where it cannot write a file or is handed a mode it does not know.
 * Install before matching starts. Nothing guards a host that swaps the
 * destination while a match is running.
 */
void setDiagnosticSink(std::function<void(const std::string&)> sink);

/**
 * @brief Lidar odometry by correspondenceless scan matching.
 * Holds the odometry state that is not a ROS concept: the cached transform
 * plans, the previous scan, the accumulating pose, and the trajectory. Feed it
 * scans and it yields the displacement between each pair.
 */
class Matcher
{
public:
  explicit Matcher(const Parameters& parameters);

  Matcher(const Matcher&) = delete;
  Matcher& operator=(const Matcher&) = delete;
  Matcher(Matcher&&) = delete;
  Matcher& operator=(Matcher&&) = delete;

  /**
   * @brief Match a scan against the one before it.
   * The first scan after construction or after a reset is kept as the
   * reference and yields no result.
   */
  std::expected<MatchResult, MatchError> process(std::span<const double> ranges);

  /**
   * @brief Forget the trajectory and return the accumulated pose to the origin.
   * The reference scan is kept, so the next scan still produces an increment.
   */
  void clearTrajectory();

  /**
   * @brief Place the accumulated pose at an arbitrary starting point.
   */
  void setInitialPose(const Pose& pose);

  const Parameters& parameters() const { return parameters_; }
  Pose accumulatedPose() const;
  const std::vector<Pose>& trajectory() const { return trajectory_; }
  unsigned int scansSeen() const { return scans_seen_; }

  /**
   * @brief How many rays every scan of this session is matched at.
   * Zero until the first scan settles it, where size_scan asked for no
   * particular size.
   */
  std::size_t matchSize() const { return match_size_; }

private:
  Parameters parameters_;

  std::size_t match_size_{0};

  std::vector<double> reference_scan_;
  std::vector<Pose> trajectory_;
  Eigen::Matrix3d accumulated_{Eigen::Matrix3d::Identity()};
  unsigned int scans_seen_{0};
};

}

#endif
