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
#ifndef FSM_H
#define FSM_H

#include <algorithm>
#include <bit>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <mutex>
#include <numeric>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <fftw3.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Min_ellipse_2.h>
#include <CGAL/Min_ellipse_2_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/squared_distance_2.h>

#include <Eigen/Geometry>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2                       Point_2;
typedef Kernel::Vector_2                      Vector_2;
typedef CGAL::Polygon_2<Kernel>               Polygon_2;
typedef Polygon_2::Vertex_iterator            VertexIterator;
typedef CGAL::Min_ellipse_2_traits_2<Kernel>  Traits;
typedef CGAL::Min_ellipse_2<Traits>           Min_ellipse;

/*
 * FFTW_MEASURE selects a plan by running timing trials, so the plan, and with
 * it the rounding of the transform, can differ between two runs of the same
 * binary on the same machine. FFTW_ESTIMATE selects by heuristic and is
 * therefore repeatable. Define FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE at build time to obtain
 * repeatable output at some cost in speed; the shipped configuration does not.
 */
#ifdef FSM_LIDAR_ODOMETRY_FFTW_ESTIMATE
  #define FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG FFTW_ESTIMATE
#else
  #define FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG FFTW_MEASURE
#endif

/*
 * Diagnostics. Define FSM_LIDAR_ODOMETRY_TRACE at build time to have the algorithm print
 * what it is doing and how long each part took, and to have it fill in the
 * timing and iteration fields of its output report. No build defines it.
 *
 * This replaces four separate switches, TIMES, PRINTS, DEBUG and LOGS, none of
 * which was ever defined anywhere and which between them guarded one thing:
 * work done for the benefit of somebody watching. The report fields those
 * switches filled are zero without it, which is worth knowing before reading
 * anything into them.
 */


namespace FSM {
/* ========================================================================== */
/*
 * A planar pose: position and orientation, the orientation in radians. The
 * members are zero initialised so that a default constructed pose reads as the
 * origin, which is what the triple of doubles this replaces already did.
 */
struct Pose
{
  double x{0.0};
  double y{0.0};
  double t{0.0};
};
/* ========================================================================== */
/*
 * Two scans compared ray by ray. diff_true is every difference as it stands;
 * diff is the same with any difference outside the inclusion bound replaced by
 * zero, so a ray that disagrees wildly cannot pull the match with it.
 */
struct ScanDifference
{
  std::vector<double> diff;
  std::vector<double> diff_true;
};
/* ========================================================================== */
/*
 * The inverse transform of the cross power spectrum of two scans, together
 * with the index of its peak. The peak is where the two scans line up, so its
 * index is what the orientation is read from.
 */
struct Correlation
{
  std::vector<double> q_0;
  unsigned int q_0_max_id{0};
};
/* ========================================================================== */
/*
 * Matching one scan against one virtual scan: the orientation it gives, and
 * the three quantities by which that orientation is judged against the others.
 */
struct FMTOutput
{
  double angle{0.0};
  double snr{0.0};
  double fahm{0.0};
  double pd{0.0};
};
/* ========================================================================== */
/*
 * What the rotation stage returns: the candidate orientations that survived
 * ranking and, one per candidate, the two criteria by which the caller picks
 * between them. The time spent finding ray intersections is reported alongside
 * because the caller accumulates it across both stages.
 */
struct RotationOutput
{
  std::vector<double> angles;
  std::vector<double> rc0;
  std::vector<double> rc1;
  std::chrono::duration<double> intersections_time{};
};
/* ========================================================================== */
/*
 * What one pass of the translation stage returns: the criterion by which the
 * caller judges the correction, the pose it settled on, how many iterations it
 * took, and the time spent finding ray intersections, which the caller
 * accumulates across both stages.
 *
 * out_of_map carries the fact that the pose left the map's bounds mid pass.
 * criterion is set to a sentinel, -2.0, in that case, but criterion is
 * otherwise an average of per ray differences that is itself not a number
 * whenever the pose it was computed from is, so a caller comparing criterion
 * against that sentinel can be fooled by a bad pose into believing the map
 * was left when it was not. out_of_map is set once, alongside the sentinel,
 * and is what a caller should read instead.
 */
struct TranslationOutput
{
  double criterion{0.0};
  Pose pose;
  int iterations{0};
  std::chrono::duration<double> intersections_time{};
  bool out_of_map{false};
};
/* ========================================================================== */
/*
 * One correction step: the x-wise and y-wise error, the per ray differences
 * that produced them, and the norm of the first transform coefficient.
 */
struct TranslationCorrection
{
  double x_e{0.0};
  double y_e{0.0};
  std::vector<double> d_v;
  double norm_x1{0.0};
};
/* ========================================================================== */
/*
 * Where one ray met the polygon, and the index of the segment it met. The
 * index is what the next ray starts its own search from, in the windowed
 * search, a scan being continuous.
 */
struct RayHit
{
  std::pair<double,double> intersection_point;
  int start_segment_id{0};
};
/* ========================================================================== */
/*
 * One scan of a recorded dataset: the ranges and the pose they were taken
 * from.
 */
struct DatasetScan
{
  std::vector<double> ranges;
  Pose pose;
};
/* ========================================================================== */
/*
 * A whole recorded dataset: every scan's ranges, and the pose each was taken
 * from, in the same order.
 */
struct Dataset
{
  std::vector< std::vector<double> > ranges;
  std::vector< Pose > poses;
};
/* ========================================================================== */
/*
 * A scan completed against a map: the completed ranges, the map they were
 * measured against, and the pose the map is expressed from.
 */
struct CompletedScan
{
  std::vector<double> scan;
  std::vector< std::pair<double,double> > map;
  Pose map_origin;
};
/* ========================================================================== */
/*
 * Which way each ray of a scan is matched to the wall it meets.
 *
 * `angular` offers each wall only to the rays whose angle can reach it. It
 * returns the nearest wall in front of every ray whatever shape the room is,
 * and costs time in proportion to the ray count.
 *
 * `windowed` narrows the search for each ray to the neighbourhood of the
 * segment the previous ray met, widening it until something is hit. It is what
 * this algorithm shipped with. It costs time in proportion to the square of
 * the ray count, and where a room turns back on itself it can hand back a wall
 * standing behind the nearest one. It is kept so that a run can be compared
 * against everything published before the angular search existed.
 */
enum class RaySearch
{
  angular,
  windowed
};
/* ========================================================================== */
struct input_params
{
  unsigned int num_iterations;
  double xy_bound;
  double t_bound;
  unsigned int max_counter;
  unsigned int min_magnification_size;
  unsigned int max_magnification_size;
  unsigned int max_recoveries;

  /* Zero draws the recovery search from hardware entropy, as this algorithm
   * has always done. Any other value pins it so a run can be reproduced. */
  unsigned int rng_seed;

  RaySearch ray_search{RaySearch::angular};
};
/* ========================================================================== */
struct output_params
{
  double exec_time;
  double rotation_times;
  double translation_times;
  double rotation_iterations;
  double translation_iterations;
  double intersections_times;
  unsigned int num_recoveries;
  std::vector< Pose > trajectory;

  /* Rotation criterion */
  double rc;

  /* Translation criterion */
  double tc;

  output_params()
  {
    exec_time = 0;
    rotation_times = 0;
    translation_times = 0;
    rotation_iterations = 0;
    translation_iterations = 0;
    intersections_times = 0;
    num_recoveries = 0;
    rc = 0;
    tc = 0;
  };
};
/* ========================================================================== */
/*
 * What a match returns: the pose increment between the two scans, and the
 * report on how it was arrived at.
 */
struct MatchOutput
{
  Pose pose;
  output_params op;
};
/* ========================================================================== */
/*
 * What follows are classes only in spelling: every member is static, none
 * holds state, and each is a namespace wearing a class's clothes. They keep
 * that form on purpose. Name lookup inside a class does not depend on the
 * order the members are written in, so a function may call one declared below
 * it. As namespaces the same file would need a declaration for all two hundred
 * odd functions before any definition, and that block would have to be kept in
 * step with the definitions by hand for the rest of the file's life. The
 * spelling is the smaller wrong.
 */
/* ========================================================================== */
/*
 * Where the core's diagnostics go.
 *
 * The core knows nothing of ROS, and nothing of whatever else a host might
 * want its reports to reach. It hands each line to the sink the host
 * installed, and where no sink was installed it drops the line. Dropping is
 * the right default: a library writing to a terminal nobody is reading is a
 * library making a decision that was never its to make.
 *
 * Most of what passes through here is stage timing, and that is compiled out
 * altogether unless FSM_LIDAR_ODOMETRY_TRACE is defined, because the clock readings around
 * every stage cost more than the printing ever did. What survives into an
 * ordinary build is the handful of complaints the core makes when it cannot
 * write a file or is handed a mode it does not recognise.
 *
 * The sink is installed once, before matching starts, and read from the thread
 * that matches. It is not guarded against a host that swaps it while a match
 * is running, and no host has reason to.
 */
class Diagnostics
{
  public:

  static void report(const std::string& message)
  {
    if (const std::function<void(const std::string&)>& installed = sink())
      installed(message);
  }

  static void setSink(std::function<void(const std::string&)> sink_in)
  {
    sink() = std::move(sink_in);
  }

  private:

  static std::function<void(const std::string&)>& sink()
  {
    static std::function<void(const std::string&)> installed;
    return installed;
  }
};
/* ========================================================================== */
class X
{
  public:

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > find(
    const Pose& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays,
    const RaySearch ray_search)
  {
    return ray_search == RaySearch::windowed
      ? findExactWindowed(pose, lines, num_rays)
      : findExactAngular(pose, lines, num_rays);
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > findExact(
    const Pose& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    double px = pose.x;
    const double py = pose.y;
    const double pt = pose.t;

    std::vector< std::pair<double,double> > intersections;
    const double mul = 100000000.0;

    for (std::size_t i = 0; i < num_rays; i++)
    {
      double t_ray = i * 2*M_PI / num_rays + pt - M_PI;
      t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

      const double x_far = px + mul*cos(t_ray);
      double y_far = py + mul*sin(t_ray);


      double tan_t_ray = tan(t_ray);
      bool tan_peligro = false;
      /* if (fabs(fabs(t_ray) - M_PI/2) == 0.0) */
      if (fabs(fabs(t_ray) - M_PI/2) < 0.0001)
        tan_peligro = true;


      std::vector< std::pair<double,double> > candidate_points;

      for (std::size_t l = 0; l < lines.size(); l++)
      {
        /* The index of the first sensed point */
        int idx_1 = l;

        /* The index of the second sensed point (in counter-clockwise order) */
        int idx_2 = idx_1 + 1;


        if (idx_2 >= static_cast<int>(lines.size()))
          idx_2 = fmod(idx_2, lines.size());

        if (idx_1 >= static_cast<int>(lines.size()))
          idx_1 = fmod(idx_1, lines.size());

        const double det_1 =
          (lines[idx_1].first-px)*(lines[idx_2].second-py)-
          (lines[idx_2].first-px)*(lines[idx_1].second-py);

        const double det_2 =
          (lines[idx_1].first-x_far)*(lines[idx_2].second-y_far)-
          (lines[idx_2].first-x_far)*(lines[idx_1].second-y_far);


        if (det_1 * det_2 <= 0.0)
        {
          const double det_3 =
            (px-lines[idx_1].first)*(y_far-lines[idx_1].second)-
            (x_far-lines[idx_1].first)*(py-lines[idx_1].second);

          const double det_4 =
            (px-lines[idx_2].first)*(y_far-lines[idx_2].second)-
            (x_far-lines[idx_2].first)*(py-lines[idx_2].second);

          if (det_3 * det_4 <= 0.0)
          {
            /* They intersect! */

            double x = 0.0;
            double y = 0.0;

            const double ttp_x = lines[idx_2].first - lines[idx_1].first;
            const double ttp_y = lines[idx_2].second - lines[idx_1].second;

            /* The line segment is perpendicular to the x-axis */
            if (ttp_x == 0.0)
            {
              /* The ray is parallel to the x-axis */
              if (x_far == px)
              {
                x = lines[idx_1].first;
                y = py;
              }
              else
              {
                x = lines[idx_1].first;
                y = y_far + (y_far - py)/(x_far - px) * (x - x_far);
              }
            }
            else
            {
              double tan_two_points = ttp_y / ttp_x;

              if (!tan_peligro)
              {
                x = (py - lines[idx_1].second + tan_two_points * lines[idx_1].first
                  -tan_t_ray * px) / (tan_two_points - tan_t_ray);

                y = py + tan_t_ray * (x - px);
              }
              else
              {
                x = px;
                y = lines[idx_1].second + tan_two_points * (x - lines[idx_1].first);
                /* y = (lines[idx_2].second + lines[idx_1].second)/2; */
              }
            }

            candidate_points.push_back(std::make_pair(x,y));
          }
        }
      }

      double min_r = 100000000.0;
      int idx = -1;
      for (std::size_t c = 0; c < candidate_points.size(); c++)
      {
        const double dx = candidate_points[c].first - px;
        const double dy = candidate_points[c].second - py;
        double r = dx*dx+dy*dy;

        if (r < min_r)
        {
          min_r = r;
          idx = c;
        }
      }

      assert(idx >= 0);

      intersections.push_back(
        std::make_pair(candidate_points[idx].first, candidate_points[idx].second));
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [X::findExact]",
      elapsed.count()));
#endif

    return intersections;
  }

  /*****************************************************************************
   * Where every ray of a scan taken from `pose` meets the walls `lines`.
   *
   * A ray can only meet a wall that stands across its direction, so each wall
   * is offered to the rays whose angle falls inside the arc that wall subtends
   * at the pose, and to no others. The rays are evenly spaced and their angles
   * are known before any of them is cast, so that arc converts straight into a
   * range of ray indices with no search involved. Walking the walls once then
   * costs about as many segment tests as there are crossings to be found,
   * rather than one test per wall per ray.
   *
   * The answer is the answer testing every wall against every ray gives.
   * Nothing that could be hit is excluded: the arc is widened by one ray index
   * at each end so a ray passing exactly through a corner cannot fall through
   * the gap, and a wall subtending half a turn or standing on the pose, which
   * is a pose lying on the wall itself, is offered to every ray.
   *
   * This is the default, and `findExactWindowed` is the alternative.
   */
  static std::vector< std::pair<double,double> > findExactAngular(
    const Pose& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    const double px = pose.x;
    const double py = pose.y;
    const double pt = pose.t;

    const double mul = 100000000.0;
    const int rays = static_cast<int>(num_rays);

    std::vector<double> x_far(num_rays);
    std::vector<double> y_far(num_rays);
    std::vector<double> tan_t_ray(num_rays);
    std::vector<char> tan_peligro(num_rays);

    for (std::size_t i = 0; i < num_rays; i++)
    {
      double t_ray = i * 2*M_PI / num_rays + pt - M_PI;
      t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

      x_far[i] = px + mul*cos(t_ray);
      y_far[i] = py + mul*sin(t_ray);
      tan_t_ray[i] = tan(t_ray);
      /* if (fabs(fabs(t_ray) - M_PI/2) == 0.0) */
      tan_peligro[i] = fabs(fabs(t_ray) - M_PI/2) < 0.0001;
    }

    /*
     * The nearest crossing each ray has met so far, as a squared distance. The
     * starting value is the hundred million this search has always begun from,
     * so a crossing further off than ten thousand metres is not believed and a
     * ray that meets only those counts as having met nothing.
     */
    const double unreached = 100000000.0;
    std::vector<double> min_r(num_rays, unreached);
    std::vector< std::pair<double,double> > intersections(num_rays);

    /*
     * Ray i leaves at i * 2*pi/num_rays + pose orientation - pi, so an angle
     * becomes the index of the ray carrying it by inverting that. Indices are
     * kept unwrapped through the arithmetic and folded into range at the last
     * moment, which is what lets an arc straddling the back of the scan be one
     * interval rather than two.
     */
    const double index_per_radian = num_rays / (2*M_PI);

    for (std::size_t l = 0; l < lines.size(); l++)
    {
      const std::pair<double,double>& p_1 = lines[l];
      const std::pair<double,double>& p_2 = lines[(l+1) % lines.size()];

      const double a_1 = atan2(p_1.second - py, p_1.first - px);
      const double a_2 = atan2(p_2.second - py, p_2.first - px);

      /* A wall subtends less than half a turn at any point off it, so the
       * short way round between its ends is the arc it covers. */
      const double span =
        (fmod(a_2 - a_1 + 5*M_PI, 2*M_PI) - M_PI) * index_per_radian;

      const double from = (a_1 - pt + M_PI) * index_per_radian;

      /*
       * Half a turn, or a corner sitting exactly on the pose, means the pose
       * is on the wall. The arc is then the whole scan or is not defined at
       * all, and the wall has to be put to every ray.
       */
      const bool pose_on_wall =
        fabs(span) >= 0.5*rays - 1.0
        || (p_1.first == px && p_1.second == py)
        || (p_2.first == px && p_2.second == py);

      const int first = pose_on_wall
        ? 0
        : static_cast<int>(std::floor(std::min(from, from + span))) - 1;

      const int last = pose_on_wall
        ? rays - 1
        : static_cast<int>(std::ceil(std::max(from, from + span))) + 1;

      for (int j = first; j <= last; j++)
      {
        const int i = ((j % rays) + rays) % rays;

        const std::optional< std::pair<double,double> > meeting =
          rayMeetsSegment(px,py, tan_t_ray[i], x_far[i],y_far[i],
            p_1, p_2, tan_peligro[i]);

        if (!meeting.has_value())
          continue;

        const double dx = meeting->first - px;
        const double dy = meeting->second - py;
        const double r = dx*dx+dy*dy;

        if (r < min_r[i])
        {
          min_r[i] = r;
          intersections[i] = *meeting;
        }
      }
    }

    /*
     * A ray that met nothing. A scan is continuous, so the nearest thing to
     * the truth is what the previous ray saw; for the first ray there is
     * nothing better than the pose itself. A ray can meet nothing when the
     * pose has wandered outside the polygon, or when a run of equal ranges has
     * made a stretch of it collinear.
     */
    for (std::size_t i = 0; i < num_rays; i++)
    {
      if (min_r[i] < unreached)
        continue;

      intersections[i] = i == 0
        ? std::make_pair(px, py)
        : intersections[i-1];
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [X::findExactAngular]",
      elapsed.count()));
#endif

    return intersections;
  }

  /*****************************************************************************
   * The nearest wall one ray meets among the segments [start, end), and which
   * segment that was. Nothing, if it meets none of them.
   */
  static std::optional<RayHit> findExactOneRay(
    const double& px, const double& py, const double& tan_t_ray,
    const double& x_far, const double& y_far,
    const std::vector< std::pair<double, double> >& lines,
    const int& start_search_id, const int& end_search_id,
    const bool& tan_peligro)
  {
    double min_r = 100000000.0;
    std::optional<RayHit> nearest;

    for (int l = start_search_id; l < end_search_id; l++)
    {
      /* The index of the first sensed point */
      int idx_1 = l;

      /* The index of the second sensed point (in counter-clockwise order) */
      int idx_2 = idx_1 + 1;

      if (idx_2 >= static_cast<int>(lines.size()))
        idx_2 = fmod(idx_2, lines.size());

      if (idx_1 >= static_cast<int>(lines.size()))
        idx_1 = fmod(idx_1, lines.size());

      const std::optional< std::pair<double,double> > meeting =
        rayMeetsSegment(px,py,tan_t_ray, x_far,y_far,
          lines[idx_1], lines[idx_2], tan_peligro);

      if (!meeting.has_value())
        continue;

      const double dx = meeting->first - px;
      const double dy = meeting->second - py;
      const double r = dx*dx+dy*dy;

      if (r < min_r)
      {
        min_r = r;
        nearest = RayHit{*meeting, idx_1};
      }
    }

    return nearest;
  }

  /*****************************************************************************
   * Where every ray of a scan taken from `pose` meets the walls `lines`, found
   * by narrowing the search for each ray to the neighbourhood of the segment
   * the previous ray met.
   *
   * A scan is continuous, so consecutive rays tend to meet neighbouring
   * segments, and a window a sixteenth of the room wide usually holds the
   * answer. Where it does not the window is widened until something is hit,
   * and failing that every segment is looked at once.
   *
   * That reasoning holds only where the walls turn one way. Across a corner
   * that turns back on itself the segment a ray meets stops advancing with the
   * ray, the window stops following it, and the nearest hit inside the window
   * can be a wall standing behind the nearest one there is. The range handed
   * on is then too long, by metres in a room of ordinary size, and nothing
   * downstream can tell.
   *
   * A window a fixed fraction of the room wide also grows as the room does,
   * and the room has as many walls as the scan has rays, so the cost of this
   * rises with the square of the ray count where `findExactAngular` rises in
   * step with it.
   *
   * It is kept, and selectable, because every result this algorithm published
   * before the angular search existed was produced by it.
   */
  static std::vector< std::pair<double,double> > findExactWindowed(
    const Pose& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    const double px = pose.x;
    const double py = pose.y;
    const double pt = pose.t;

    std::vector< std::pair<double,double> > intersections;
    const double mul = 100000000.0;

    int start0 = 0;
    int end0 = lines.size();

    for (std::size_t i = 0; i < num_rays; i++)
    {
      double t_ray = i * 2*M_PI / num_rays + pt - M_PI;
      t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

      const double x_far = px + mul*cos(t_ray);
      const double y_far = py + mul*sin(t_ray);

      const double tan_t_ray = tan(t_ray);
      /* if (fabs(fabs(t_ray) - M_PI/2) == 0.0) */
      const bool tan_peligro = fabs(fabs(t_ray) - M_PI/2) < 0.0001;

      std::pair<double,double> intersection_point;
      bool success = false;
      const int inc = std::max<int>(1, lines.size()/16);

      /*
       * The window is widened until the ray hits something. Bound the widening
       * by the number of segments there are: without a bound, a ray that hits
       * nothing widens the window until start0 overflows, after which the
       * search indexes the segment vector with a negative number and the
       * process dies. A ray can fail to hit when the pose has wandered outside
       * the polygon, or when a run of equal ranges has made a stretch of it
       * collinear.
       */
      const int max_widenings = static_cast<int>(lines.size()) / inc + 2;
      int widenings = 0;

      while (!success && widenings < max_widenings)
      {
        const std::optional<RayHit> hit =
          findExactOneRay(px,py,tan_t_ray, x_far,y_far,lines,
            start0, end0, tan_peligro);

        success = hit.has_value();

        if (success)
        {
          intersection_point = hit->intersection_point;
          start0 = hit->start_segment_id;
        }
        else
          start0 += inc;

        end0 = start0 + inc;
        widenings++;
      }

      /* Widening found nothing, so look at every segment once */
      if (!success)
      {
        start0 = 0;
        end0 = static_cast<int>(lines.size());

        const std::optional<RayHit> hit =
          findExactOneRay(px,py,tan_t_ray, x_far,y_far,lines,
            start0, end0, tan_peligro);

        success = hit.has_value();

        if (success)
        {
          intersection_point = hit->intersection_point;
          start0 = hit->start_segment_id;
          end0 = start0 + inc;
        }
      }

      /*
       * The ray genuinely hits nothing. A scan is continuous, so the nearest
       * thing to the truth is what the previous ray saw; for the first ray
       * there is nothing better than the pose itself.
       */
      if (!success)
      {
        intersection_point = intersections.empty()
          ? std::make_pair(px, py)
          : intersections.back();

        start0 = 0;
        end0 = static_cast<int>(lines.size());
      }

      intersections.push_back(intersection_point);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [X::findExactWindowed]",
      elapsed.count()));
#endif

    return intersections;
  }

  /*****************************************************************************
   * Where one ray meets one wall, if it meets it at all.
   *
   * The ray is handed over as the point it leaves from and a second point a
   * hundred million metres along it, which turns the question into whether two
   * line segments cross. Four determinants settle that. The crossing itself is
   * then where two lines meet, written three ways because a wall perpendicular
   * to the x-axis has no gradient and a ray pointing along the y-axis has no
   * finite one.
   */
  static std::optional< std::pair<double,double> > rayMeetsSegment(
    const double& px, const double& py, const double& tan_t_ray,
    const double& x_far, const double& y_far,
    const std::pair<double, double>& p_1, const std::pair<double, double>& p_2,
    const bool& tan_peligro)
  {
    const double det_1 =
      (p_1.first-px)*(p_2.second-py)-
      (p_2.first-px)*(p_1.second-py);

    const double det_2 =
      (p_1.first-x_far)*(p_2.second-y_far)-
      (p_2.first-x_far)*(p_1.second-y_far);

    if (det_1 * det_2 <= 0.0)
    {
      const double det_3 =
        (px-p_1.first)*(y_far-p_1.second)-
        (x_far-p_1.first)*(py-p_1.second);

      const double det_4 =
        (px-p_2.first)*(y_far-p_2.second)-
        (x_far-p_2.first)*(py-p_2.second);

      if (det_3 * det_4 <= 0.0)
      {
        /* They intersect! */

        double x = 0.0;
        double y = 0.0;

        const double ttp_x = p_2.first - p_1.first;
        const double ttp_y = p_2.second - p_1.second;

        /* The line segment is perpendicular to the x-axis */
        if (ttp_x == 0.0)
        {
          /* The ray is parallel to the x-axis */
          if (x_far == px)
          {
            x = p_1.first;
            y = py;
          }
          else
          {
            x = p_1.first;
            y = y_far + (y_far - py)/(x_far - px) * (x - x_far);
          }
        }
        else
        {
          const double tan_two_points = ttp_y / ttp_x;

          if (!tan_peligro)
          {
            x = (py - p_1.second + tan_two_points * p_1.first
              -tan_t_ray * px) / (tan_two_points - tan_t_ray);

            y = py + tan_t_ray * (x - px);
          }
          else
          {
            x = px;
            y = p_1.second + tan_two_points * (x - p_1.first);
            /* y = (p_2.second + p_1.second)/2; */
          }
        }

        return std::make_pair(x,y);
      }
    }

    return std::nullopt;
  }
};

/* ========================================================================== */
class Utils
{
  public:

  /*****************************************************************************
   * The engine behind every random pose the recovery search tries.
   *
   * A seed of zero draws from hardware entropy, which is what this algorithm
   * has always done and which no run can reproduce. Any other value seeds the
   * engine once and leaves it running, so a whole session replays identically.
   */
  /*****************************************************************************
  */
  static Eigen::Matrix3d
  computeTransform(const Pose& d,
    const Eigen::Matrix3d& M)
  {
    const double dx = d.x;
    const double dy = d.y;
    const double dt = d.t;

    /* Translation matrix */
    Eigen::Matrix3d T;
    T = Eigen::Matrix3d::Identity();
    T(0,2) = dx;
    T(1,2) = dy;

    /* Rotation matrix */
    Eigen::Matrix3d R;
    R = Eigen::Matrix3d::Identity();
    R(0,0) = +cos(dt);
    R(0,1) = -sin(dt);
    R(1,0) = +sin(dt);
    R(1,1) = +cos(dt);

    /* Compute the new transform matrix */
    Eigen::Matrix3d M_;
    M_ = M * T * R;

    return M_;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> > conjugate(
    const std::vector< std::pair<double, double> >& vec)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector< std::pair<double,double> > ret_vector;
    for (std::size_t i = 0; i < vec.size(); i++)
      ret_vector.push_back(std::make_pair(vec[i].first, -vec[i].second));

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    Diagnostics::report(std::format("{:f} [conjugate]",
      elapsed.count()));
#endif

    return ret_vector;
  }

  /*****************************************************************************
  */
  static ScanDifference diffScansPerRay(
    const std::span<const double> scan1, const std::span<const double> scan2,
    const double& inclusion_bound)
  {
    assert (scan1.size() == scan2.size());

    ScanDifference difference;

    double eps = 0.000001;
    if (inclusion_bound < 0.0001)
      eps = 1.0;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("inclusion_bound = {:f}",
      inclusion_bound + eps));
#endif

    double d = 0.0;
    for (unsigned int i = 0; i < scan1.size(); i++)
    {
      d = scan1[i] - scan2[i];

      if (fabs(d) <= inclusion_bound + eps)
        difference.diff.push_back(d);
      else
        difference.diff.push_back(0.0);

      difference.diff_true.push_back(d);
    }

    return difference;
  }

  /*****************************************************************************
  */
  static Pose generatePose(
    const Pose& real_pose,
    const double& dxy, const double& dt)
  {
    assert(dxy >= 0);
    assert(dt >= 0);

    std::random_device rand_dev;
    std::mt19937 generator_x(rand_dev());
    std::mt19937 generator_y(rand_dev());
    std::mt19937 generator_t(rand_dev());
    std::mt19937 generator_sign(rand_dev());

    std::uniform_real_distribution<double> distribution_x(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_y(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_t(-dt, dt);

    const double rx = distribution_x(generator_x);
    const double ry = distribution_y(generator_y);
    const double rt = distribution_t(generator_t);

    Pose virtual_pose;
    virtual_pose.x = real_pose.x + rx;
    virtual_pose.y = real_pose.y + ry;
    virtual_pose.t = real_pose.t + rt;

    virtual_pose.t = wrapAngle(virtual_pose.t);

    return virtual_pose;
  }

  /*****************************************************************************
  */
  static std::optional<Pose> generatePose(
    const Pose& base_pose,
    const std::vector< std::pair<double,double> >& map,
    const double& dxy, const double& dt, const double& dist_threshold,
    const RaySearch ray_search,
    const unsigned int seed = 0)
  {
    assert(dxy >= 0.0);
    assert(dt >= 0.0);

    std::mt19937& generator_x = randomEngine(seed);
    std::mt19937& generator_y = generator_x;
    std::mt19937& generator_t = generator_x;

    std::uniform_real_distribution<double> distribution_x(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_y(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_t(-dt, dt);

    /* A temp real pose */
    Pose real_pose_ass;

    /* Fill in the orientation regardless */
    const double rt = distribution_t(generator_t);
    real_pose_ass.t = base_pose.t + rt;
    real_pose_ass.t = Utils::wrapAngle(real_pose_ass.t);

    /*
     * We assume that the lidar sensor is distanced from the closest obstacle
     * by a certain amount (e.g. the radius of a circular base)
     */
    bool pose_found = false;
    while (!pose_found)
    {
      pose_found = true;
      const double rx = distribution_x(generator_x);
      const double ry = distribution_y(generator_y);

      real_pose_ass.x = base_pose.x + rx;
      real_pose_ass.y = base_pose.y + ry;

      if (isPositionInMap(real_pose_ass, map))
      {
        for (unsigned int i = 0; i < map.size(); i++)
        {
          const double dx = real_pose_ass.x - map[i].first;
          const double dy = real_pose_ass.y - map[i].second;

          if (dx*dx + dy*dy < dist_threshold*dist_threshold)
          {
            pose_found = false;
            break;
          }
        }
      }
      else pose_found = false;
    }

    /* Verify distance threshold */
    const std::vector< std::pair<double,double> > intersections =
      X::find(real_pose_ass, map, map.size(), ray_search);
    const std::vector<double> real_scan =
      points2scan(intersections, real_pose_ass);

    const unsigned int min_dist_idx =
      std::min_element(real_scan.begin(), real_scan.end()) - real_scan.begin();

    if (real_scan[min_dist_idx] > dist_threshold)
      return real_pose_ass;

    return std::nullopt;
  }

  /*****************************************************************************
  */
  static std::optional<Pose> generatePoseWithinMap(
    const std::vector< std::pair<double,double> >& map,
    const double& dist_threshold,
    const RaySearch ray_search)
  {
    /* A temp real pose */
    Pose real_pose_ass;

    /* Generate orientation */
    std::random_device rand_dev_t;
    std::mt19937 generator_t(rand_dev_t());

    std::uniform_real_distribution<double> distribution_t(-M_PI, M_PI);

    /* Fill in the orientation regardless */
    real_pose_ass.t = distribution_t(generator_t);

    /* Find the bounding box of the map */
    double max_x = -1000.0;
    double min_x = +1000.0;
    double max_y = -1000.0;
    double min_y = +1000.0;

    for (unsigned int i = 0; i < map.size(); i++)
    {
      if (map[i].first > max_x)
        max_x = map[i].first;

      if (map[i].first < min_x)
        min_x = map[i].first;

      if (map[i].second > max_y)
        max_y = map[i].second;

      if (map[i].second < min_y)
        min_y = map[i].second;
    }

    std::random_device rand_dev_x;
    std::random_device rand_dev_y;
    std::mt19937 generator_x(rand_dev_x());
    std::mt19937 generator_y(rand_dev_y());

    std::uniform_real_distribution<double> distribution_x(min_x, max_x);
    std::uniform_real_distribution<double> distribution_y(min_y, max_y);

    /*
     * We assume that the lidar sensor is distanced from the closest obstacle
     * by a certain amount (e.g. the radius of a circular base)
     */
    bool pose_found = false;
    while (!pose_found)
    {
      pose_found = true;
      const double rx = distribution_x(generator_x);
      const double ry = distribution_y(generator_y);

      real_pose_ass.x = rx;
      real_pose_ass.y = ry;

      if (isPositionInMap(real_pose_ass, map))
      {
        for (unsigned int i = 0; i < map.size(); i++)
        {
          const double dx = real_pose_ass.x - map[i].first;
          const double dy = real_pose_ass.y - map[i].second;

          if (dx*dx + dy*dy < dist_threshold*dist_threshold)
          {
            pose_found = false;
            break;
          }
        }
      }
      else pose_found = false;
    }

    /* Verify distance threshold */
    const std::vector< std::pair<double,double> > intersections =
      X::find(real_pose_ass, map, map.size(), ray_search);
    const std::vector<double> real_scan =
      points2scan(intersections, real_pose_ass);

    const unsigned int min_dist_idx =
      std::min_element(real_scan.begin(), real_scan.end()) - real_scan.begin();

    if (real_scan[min_dist_idx] > dist_threshold)
      return real_pose_ass;

    return std::nullopt;
  }

  /*****************************************************************************
  */
  static std::vector<double> innerProduct(const std::span<const double> vec1,
    const std::span<const double> vec2)
  {
    assert(vec1.size() == vec2.size());

    std::vector<double> ret_vector;

    for (std::size_t i = 0; i < vec1.size(); i++)
    {
      ret_vector.push_back(vec1[i] * vec2[i]);
    }

    return ret_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> > innerProductComplex(
    const std::vector< std::pair<double, double> >& vec1,
    const std::vector< std::pair<double, double> >& vec2)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    assert(vec1.size() == vec2.size());

    std::vector< std::pair<double, double> > ret_vector;

    for (std::size_t i = 0; i < vec1.size(); i++)
    {
      const double re =
        vec1[i].first * vec2[i].first - vec1[i].second * vec2[i].second;
      const double im =
        vec1[i].first * vec2[i].second + vec1[i].second * vec2[i].first;

      ret_vector.push_back(std::make_pair(re,im));
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    Diagnostics::report(std::format("{:f} [innerProductComplex]",
      elapsed.count()));
#endif

    return ret_vector;
  }

  /*****************************************************************************
  */
  static bool isPositionFartherThan(
    const Pose& pose,
    const std::vector< std::pair<double,double> >& map,
    const double& dist)
  {
    for (unsigned int i = 0; i < map.size(); i++)
    {
      const double dx = pose.x - map[i].first;
      const double dy = pose.y - map[i].second;
      const double d = sqrt(dx*dx + dy*dy);

      if (d < dist)
        return false;
    }

    return true;
  }

  /*****************************************************************************
  */
  static bool isPositionInMap(
    const Pose& pose,
    const std::vector< std::pair<double,double> >& map)
  {
    const Point_2 point(pose.x, pose.y);

    /* Construct polygon from map */
    Polygon_2 poly;
    for (std::size_t p = 0; p < map.size(); p++)
      poly.push_back(Point_2(map[p].first, map[p].second));

    poly.push_back(Point_2(map[map.size()-1].first, map[map.size()-1].second));

    bool inside = false;
    if(CGAL::bounded_side_2(poly.vertices_begin(),
        poly.vertices_end(),
        point, Kernel()) == CGAL::ON_BOUNDED_SIDE)
    {
      inside = true;
    }

    return inside;
  }

  /*****************************************************************************
  */
  static std::pair<double,double> multiplyWithRotationMatrix(
    const std::pair<double,double>& point, const double& angle)
  {
    const double R11 = cos(angle);
    const double R12 = -sin(angle);
    const double R21 = -R12;
    const double R22 = R11;

    const double x = R11 * point.first + R12 * point.second;
    const double y = R21 * point.first + R22 * point.second;

    return std::make_pair(x,y);

  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > multiplyWithRotationMatrix(
    const std::vector< std::pair<double,double> >& points,
    const double& angle)
  {
    std::vector< std::pair<double,double> > return_vector;

    for (std::size_t i = 0; i < points.size(); i++)
      return_vector.push_back(multiplyWithRotationMatrix(points[i], angle));

    return return_vector;
  }

  /*****************************************************************************
  */
  static double norm(const std::pair<double,double>& vec)
  {
    return sqrt(vec.first*vec.first + vec.second*vec.second);
  }

  /*****************************************************************************
  */
  static std::vector<double> norm(
    const std::vector< std::pair<double,double> >& vec)
  {
    std::vector<double> ret_vector;

    for (std::size_t i = 0; i < vec.size(); i++)
      ret_vector.push_back(norm(vec[i]));

    return ret_vector;
  }

  /*****************************************************************************
  */
  static double norm2(const std::vector< std::pair<double,double> >& vec)
  {
    std::vector<double> ret_vector;

    for (std::size_t i = 0; i < vec.size(); i++)
      ret_vector.push_back(norm(vec[i]));

    return accumulate(ret_vector.begin(), ret_vector.end(), 0.0);
  }

  /*****************************************************************************
  */
  static std::pair<double,double> pairDiff(
    const std::pair<double,double>& pair1,
    const std::pair<double,double>& pair2)
  {
    std::pair<double,double> ret_pair;
    ret_pair.first = pair2.first - pair1.first;
    ret_pair.second = pair2.second - pair1.second;

    return ret_pair;
  }

  /*****************************************************************************
  */
  static std::vector<double> points2scan(
    const std::vector< std::pair<double,double> >& points,
    const Pose& pose)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector<double> scan;

    const double px = pose.x;
    const double py = pose.y;

    double dx = 0.0;
    double dy = 0.0;
    for (std::size_t i = 0; i < points.size(); i++)
    {
      dx = points[i].first - px;
      dy = points[i].second - py;
      scan.push_back(sqrt(dx*dx+dy*dy));
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    Diagnostics::report(std::format("{:f} [points2scan]",
      elapsed.count()));
#endif

    return scan;
  }

  static std::mt19937& randomEngine(const unsigned int seed = 0)
  {
    static thread_local std::mt19937 engine{std::random_device{}()};
    static thread_local unsigned int current_seed = 0;

    if (seed != 0 && seed != current_seed)
    {
      engine.seed(seed);
      current_seed = seed;
    }

    return engine;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > scan2points(
    const std::span<const double> scan,
    const Pose pose,
    const double& angle_span = 2*M_PI)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector< std::pair<double,double> > points;

    const double px = pose.x;
    const double py = pose.y;
    const double pt = pose.t;

    /* The angle of the first ray (in the local coordinate system) */
    const double sa = -angle_span/2;

    for (std::size_t i = 0; i < scan.size(); i++)
    {
      const double x =
        px + scan[i] * cos(i * angle_span / scan.size() + pt + sa);
      const double y =
        py + scan[i] * sin(i * angle_span / scan.size() + pt + sa);

      points.push_back(std::make_pair(x,y));
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    Diagnostics::report(std::format("{:f} [scan2points]",
      elapsed.count()));
#endif

    return points;
  }

  /*****************************************************************************
  */
  static std::vector<double> scanFromPose(
    const Pose& pose,
    const std::vector< std::pair<double,double> >& points,
    const unsigned int& num_rays,
    const RaySearch ray_search)
  {
    const std::vector< std::pair<double,double> > intersections =
      X::find(pose, points, num_rays, ray_search);

    return points2scan(intersections, pose);
  }

  /*****************************************************************************
  */
  static constexpr int sgn(const double& a)
  {
    return (a > 0.0) - (a < 0.0);
  }

  /*****************************************************************************
  */
  static std::vector<double>
  subsampleScan(const std::span<const double> scan_in, const size_t& sz,
    const RaySearch ray_search)
  {
    Pose zero_pose;
    zero_pose.x = 0.0;
    zero_pose.y = 0.0;
    zero_pose.t = 0.0;

    /* Turn scan to points */
    const std::vector< std::pair<double,double> > scan_points =
      scan2points(scan_in, zero_pose);

    /* scan_out: the ranges to `scan_points` from `zero_pose` */
    const std::vector<double> scan_out =
      scanFromPose(zero_pose, scan_points, sz, ray_search);

    return scan_out;
  }

  /***************************************************************************
  */
  static std::vector< std::pair<double,double> > vectorDiff(
    const std::vector< std::pair<double,double> >& vec)
  {
    std::vector< std::pair<double,double> > ret_vector;

    for (std::size_t i = 0; i < vec.size()-1; i++)
      ret_vector.push_back(pairDiff(vec[i], vec[i+1]));

    return ret_vector;
  }

  /***************************************************************************
  */
  static std::pair<double,double> vectorStatistics(
    const std::span<const double> v)
  {
    const double sum = std::accumulate(v.begin(), v.end(), 0.0);
    const double mean = sum / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
      [mean](const double v) { return v - mean; });
    const double sq_sum =
      std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    const double stdev = std::sqrt(sq_sum / v.size());

    return std::make_pair(mean, stdev);
  }

  /***************************************************************************
  */
  static constexpr double wrapAngle(const double angle)
  {
    return fmod(angle + 5*M_PI, 2*M_PI) - M_PI;
  }
};


/* ========================================================================== */
class DatasetUtils
{
  public:

  /***************************************************************************
  */
  static std::vector< std::vector< std::pair<double,double> > >
    dataset2points(const char* dataset_filepath)
    {
      const auto [ranges, poses] = readDataset(dataset_filepath);

      [[maybe_unused]] const int num_scans = ranges.size();
      const int num_rays = ranges[0].size();
      const double angle_span = M_PI;

      std::vector< std::vector< std::pair<double,double> > > polygons;
      for (std::size_t s = 0; s < ranges.size(); s++)
      {
        const double px = poses[s].x;
        const double py = poses[s].y;
        const double pt = poses[s].t;

        std::vector< std::pair<double,double> > polygon;

        for (std::size_t r = 0; r < ranges[s].size(); r++)
        {
          const double x =
            px + ranges[s][r] * cos(r*angle_span/(num_rays-1) + pt -angle_span/2);
          double y =
            py + ranges[s][r] * sin(r*angle_span/(num_rays-1) + pt -angle_span/2);

          polygon.push_back(std::make_pair(x,y));
        }

        polygons.push_back(polygon);
      }

      return polygons;
    }

  /*****************************************************************************
  */
  static DatasetScan dataset2rangesAndPose(const char* dataset_filepath)
  {
    return readDatasetScan(dataset_filepath);
  }

  /*****************************************************************************
  */
  static std::vector<double>
  interpolateRanges( const std::span<const double> ranges)
  {
    /* Identify contiguous regions of zero measurement */
    std::vector< std::vector <int> > regions;
    int i = 0;
    int j;
    while(i < static_cast<int>(ranges.size()))
    {
      if (ranges[i] == 0)
      {
        const int region_begin = i;

        /*
         * Every path below writes this before it is read, but only just: the
         * compiler cannot see that and neither can a reader. Give it the value
         * it takes when the run reaches the end of the scan.
         */
        int region_end = static_cast<int>(ranges.size()) - 1;
        bool broke = false;
        for (j = region_begin+1; j < static_cast<int>(ranges.size()); j++)
        {
          if (ranges[j] == 0)
            continue;
          if (ranges[j] > 0)
          {
            region_end = j-1;
            {
              broke = true;
              break;
            }
          }
        }

        if (j == static_cast<int>(ranges.size()) && !broke)
          region_end = j-1;

        std::vector<int> region;
        region.push_back(region_begin);
        region.push_back(region_end);
        regions.push_back(region);

        i = j+1;
      }
      else
        i++;
    }

    /* Inflate to consecutive indices */
    for (unsigned int i = 0; i < regions.size(); i++)
    {
      const int begin = regions[i][0];
      const int end = regions[i][regions[i].size()-1];

      regions[i].clear();

      for (int j = begin; j <= end; j++)
        regions[i].push_back(j);
    }

    /* Nothing was invalid, so there is nothing to interpolate over */
    if (regions.empty())
      return std::vector<double>(ranges.begin(), ranges.end());

    /* Is the first index 0 and the last equal to the size-1? */
    const int num_regions = regions.size();
    const int numel_last_region = regions[num_regions-1].size();

    if (regions[0][0] == 0 &&
      regions[num_regions-1][numel_last_region-1] == static_cast<int>(ranges.size())-1)
    {
      for (std::size_t i = 0; i < regions[0].size(); i++)
        regions[num_regions-1].push_back(regions[0][i]);

      regions.erase(regions.begin(), regions.begin()+1);
    }

    /*
     * for (int i = 0; i < regions.size(); i++)
     * {
     * printf("region %d\n", i);
     * for (int j = 0; j < regions[i].size(); j++)
     * printf("%d,", regions[i][j]);
     * printf("\n");
     * }
     */

    std::vector<double> ranges_interp(ranges.begin(), ranges.end());

    for (unsigned int i = 0; i < regions.size(); i++)
    {
      int interp_begin = regions[i][0]-1;
      int interp_end = regions[i][regions[i].size()-1]+1;

      if (interp_begin < 0)
        interp_begin = ranges.size()-1;

      if (interp_end >= static_cast<int>(ranges.size()))
        interp_end = 0;

      const double range_a = ranges[interp_begin];
      const double range_b = ranges[interp_end];
      double interp = (range_a + range_b)/2;

      for (unsigned int j = 0; j < regions[i].size(); j++)
        ranges_interp[regions[i][j]]= interp;
    }

    /*
     * for (int j = 0; j < ranges.size(); j++)
     * printf("%f\n", ranges_interp[j]);
     */

    return ranges_interp;
  }

  /****************************************************************************
  */
  static void printDataset(const char* dataset_filepath)
  {
    const auto [ranges, poses] = readDataset(dataset_filepath);

    for (std::size_t s = 0; s < ranges.size(); s++)
    {
      Diagnostics::report("NEW SCAN");
      for (std::size_t r = 0; r < ranges[s].size(); r++)
      {
        Diagnostics::report(std::format("r[{}] = {:f}",
          r, ranges[s][r]));
      }

      Diagnostics::report(std::format("FROM POSE ({:f},{:f},{:f})",
        poses[s].x, poses[s].y, poses[s].t));
    }
  }

  /*****************************************************************************
  */
  static Dataset readDataset(const char* filepath)
  {
    Dataset dataset;

    /*
     * First read the first two number: they show
     * (1) the number of scans and
     * (2) the number of rays per scan.
     */
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL)
      exit(EXIT_FAILURE);

    char* line = NULL;
    size_t len = 0;

    unsigned int line_number = 0;
    [[maybe_unused]] long int num_scans = 0;
    long int num_rays = 0;
    while ((getline(&line, &len, fp)) != -1 && line_number < 1)
    {
      line_number++;

      char * pEnd;
      num_scans = strtol (line, &pEnd, 10);
      num_rays = strtol (pEnd, &pEnd, 10);
    }

    fclose(fp);

    if (line)
      free(line);


    /* Begin for all scans */
    fp = fopen(filepath, "r");
    line = NULL;
    len = 0;

    /* The line number read at each iteration */
    line_number = 0;

    /* A vector holding scan ranges for one scan */
    std::vector<double> ranges_one_scan;

    /* loop */
    while ((getline(&line, &len, fp)) != -1)
    {
      line_number++;

      /* We don't have to care about the first line now */
      if (line_number == 1)
        continue;

      /* These lines host the poses from which the scans were taken */
      if ((line_number-1) % (num_rays+1) == 0)
      {
        /* Finished with this scan */
        dataset.ranges.push_back(ranges_one_scan);

        /* Clear the vector so we can begin all over */
        ranges_one_scan.clear();

        /* The pose from which the scan_number-th scan was taken */
        std::string pose(line);  /* convert from char to string */
        std::string::size_type sz;  /* alias of size_t */

        const double px = std::stod(pose,&sz);
        pose = pose.substr(sz);
        const double py = std::stod(pose,&sz);
        double pt = std::stod(pose.substr(sz));
        pt = Utils::wrapAngle(pt);
        dataset.poses.push_back(Pose{px,py,pt});

        continue;
      }

      /* At this point we are in a line holding a range measurement; fo sho */
      double range;
      assert(sscanf(line, "%lf", &range) == 1);
      ranges_one_scan.push_back(range);
    }

    fclose(fp);

    if (line)
      free(line);

    return dataset;
  }

  /*****************************************************************************
  */
  static DatasetScan readDatasetScan(const char* filepath)
  {
    DatasetScan scan;

    /*
     * First read the first two number: they show
     * (1) the number of scans and
     * (2) the number of rays per scan.
     */
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL)
      exit(EXIT_FAILURE);

    char* line = NULL;
    size_t len = 0;

    unsigned int line_number = 0;
    [[maybe_unused]] long int num_scans = 0;
    long int num_rays = 0;
    while ((getline(&line, &len, fp)) != -1 && line_number < 1)
    {
      line_number++;

      char * pEnd;
      num_scans = strtol (line, &pEnd, 10);
      num_rays = strtol (pEnd, &pEnd, 10);
    }

    fclose(fp);

    if (line)
      free(line);


    /* Begin for all scans */
    fp = fopen(filepath, "r");
    line = NULL;
    len = 0;

    /* The line number read at each iteration */
    line_number = 0;

    /* loop */
    while ((getline(&line, &len, fp)) != -1)
    {
      line_number++;

      /* We don't have to care about the first line now */
      if (line_number == 1)
        continue;

      /* These lines host the poses from which the scans were taken */
      if ((line_number-1) % (num_rays+1) == 0)
      {
        /* The pose from which the scan_number-th scan was taken */
        std::string pose_d(line);  /* convert from char to string */
        std::string::size_type sz;  /* alias of size_t */

        const double px = std::stod(pose_d,&sz);
        pose_d = pose_d.substr(sz);
        const double py = std::stod(pose_d,&sz);
        double pt = std::stod(pose_d.substr(sz));
        pt = Utils::wrapAngle(pt);
        scan.pose = Pose{px,py,pt};

        continue;
      }

      /* At this point we are in a line holding a range measurement; fo sho */
      double range_d;
      assert(sscanf(line, "%lf", &range_d) == 1);
      scan.ranges.push_back(range_d);
    }

    fclose(fp);

    if (line)
      free(line);

    return scan;
  }
};


/* ========================================================================== */
class Dump
{
  public:

  /*****************************************************************************
  */
  /*****************************************************************************
  */
  static void convexHulls(const std::vector<Point_2>& real_hull,
    const std::vector<Point_2>& virtual_hull,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "h_rx = [];" << std::endl;
      file << "h_ry = [];" << std::endl;

      for (std::size_t i = 0; i < real_hull.size(); i++)
      {
        file << "h_rx = [h_rx " << real_hull[i].x() << "];" << std::endl;
        file << "h_ry = [h_ry " << real_hull[i].y() << "];" << std::endl;
      }

      file << "h_vx = [];" << std::endl;
      file << "h_vy = [];" << std::endl;

      for (std::size_t i = 0; i < virtual_hull.size(); i++)
      {
        file << "h_vx = [h_vx " << virtual_hull[i].x() << "];" << std::endl;
        file << "h_vy = [h_vy " << virtual_hull[i].y() << "];" << std::endl;
      }

      file.close();
    }
    else
      Diagnostics::report("Could not log hulls ");
  }

  /*****************************************************************************
  */
  static void map(const std::vector< std::pair<double,double> >& map,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "mx = [];" << std::endl;
      file << "my = [];" << std::endl;
      for (std::size_t i = 0; i < map.size(); i++)
      {
        file << "mx = [mx " << map[i].first << "];" << std::endl;
        file << "my = [my " << map[i].second << "];" << std::endl;
      }

      file.close();
    }
    else
      Diagnostics::report("Could not log scans");
  }

  /*****************************************************************************
  */
  static void points(const std::vector< std::pair<double,double> >& real_points,
    const std::vector< std::pair<double,double> >& virtual_points,
    [[maybe_unused]] const unsigned int& id,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rx = [];" << std::endl;
      file << "ry = [];" << std::endl;
      for (std::size_t i = 0; i < real_points.size(); i++)
      {
        file << "rx = [rx " << real_points[i].first << "];" << std::endl;
        file << "ry = [ry " << real_points[i].second << "];" << std::endl;
      }

      file << "vx = [];" << std::endl;
      file << "vy = [];" << std::endl;
      for (std::size_t i = 0; i < virtual_points.size(); i++)
      {
        file << "vx = [vx " << virtual_points[i].first << "];" << std::endl;
        file << "vy = [vy " << virtual_points[i].second << "];" << std::endl;
      }

      file.close();
    }
    else
      Diagnostics::report("Could not log points");
  }

  /*****************************************************************************
  */
  static void polygon(const Polygon_2& poly, const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "px = [];" << std::endl;
      file << "py = [];" << std::endl;

      for (VertexIterator vi = poly.vertices_begin();
        vi != poly.vertices_end(); vi++)
      {
        file << "px = [px " << vi->x() << "];" << std::endl;
        file << "py = [py " << vi->y() << "];" << std::endl;
      }

      file.close();
    }
    else
      Diagnostics::report("Could not log polygon");
  }

  /*****************************************************************************
  */
  static void polygons(const Polygon_2& real_poly,
    const Polygon_2& virtual_poly,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "p_rx = [];" << std::endl;
      file << "p_ry = [];" << std::endl;

      for (VertexIterator vi = real_poly.vertices_begin();
        vi != real_poly.vertices_end(); vi++)
      {
        file << "p_rx = [p_rx " << vi->x() << "];" << std::endl;
        file << "p_ry = [p_ry " << vi->y() << "];" << std::endl;
      }

      file << "p_vx = [];" << std::endl;
      file << "p_vy = [];" << std::endl;

      for (VertexIterator vi = virtual_poly.vertices_begin();
        vi != virtual_poly.vertices_end(); vi++)
      {
        file << "p_vx = [p_vx " << vi->x() << "];" << std::endl;
        file << "p_vy = [p_vy " << vi->y() << "];" << std::endl;
      }

      file.close();
    }
    else
      Diagnostics::report("Could not log polygons ");
  }

  /*****************************************************************************
  */
  static void rangeScan(
    const std::span<const double> real_scan,
    const std::span<const double> virtual_scan,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rr = [];" << std::endl;
      for (std::size_t i = 0; i < real_scan.size(); i++)
        file << "rr = [rr " << real_scan[i] << "];" << std::endl;

      file << "rt = [];" << std::endl;
      for (std::size_t i = 0; i < real_scan.size(); i++)
        file << "rt = [rt " << i * 2 * M_PI / real_scan.size() << "];" << std::endl;

      file << "vr = [];" << std::endl;
      for (std::size_t i = 0; i < virtual_scan.size(); i++)
        file << "vr = [vr " << virtual_scan[i] << "];" << std::endl;

      file << "vt = [];" << std::endl;
      for (std::size_t i = 0; i < virtual_scan.size(); i++)
        file << "vt = [vt " << i * 2 * M_PI / virtual_scan.size() << "];" << std::endl;

      file.close();
    }
    else
      Diagnostics::report("Could not log range scans");
  }

  static void scan(
    const std::span<const double> real_scan,
    const Pose& real_pose,
    const std::span<const double> virtual_scan,
    const Pose& virtual_pose,
    const std::string& dump_filepath)
  {
    const std::vector< std::pair<double,double> > real_scan_points =
      Utils::scan2points(real_scan, real_pose);

    const std::vector< std::pair<double,double> > virtual_scan_points =
      Utils::scan2points(virtual_scan, virtual_pose);

    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rx = [];" << std::endl;
      file << "ry = [];" << std::endl;

      for (std::size_t i = 0; i < real_scan.size(); i++)
      {
        file << "rx = [rx " << real_scan_points[i].first << "];" << std::endl;
        file << "ry = [ry " << real_scan_points[i].second << "];" << std::endl;
      }

      file << "vx = [];" << std::endl;
      file << "vy = [];" << std::endl;
      for (std::size_t i = 0; i < virtual_scan.size(); i++)
      {
        file << "vx = [vx " << virtual_scan_points[i].first << "];" << std::endl;
        file << "vy = [vy " << virtual_scan_points[i].second << "];" << std::endl;
      }

      file << "r00 = [" << real_pose.x <<
        ", " << real_pose.y << "];" << std::endl;
      file << "v00 = [" << virtual_pose.x <<
        ", " << virtual_pose.y << "];" << std::endl;

      file.close();
    }
    else
      Diagnostics::report("Could not log scans");
  }

};

/* ========================================================================== */
class ScanCompletion
{
  public:

  /*****************************************************************************
  */
  static void completeScan(std::vector<double>& scan, const int& method)
  {
    if (method == 1)
      completeScan1(scan);
    else if (method == 3)
      completeScan3(scan);
    else if (method == 4)
      completeScan4(scan);
    else
      completeScan1(scan);
  }

  /*****************************************************************************
  */
  static void completeScan1(std::vector<double>& scan)
  {
    const std::vector<double> scan_copy = scan;

    for (int i = scan_copy.size()-2; i > 0; i--)
      scan.push_back(scan_copy[i]);

    /* Rotate so that it starts from -M_PI rather than -M_PI / 2 */
    const int num_pos = scan.size() / 4;

    std::rotate(scan.begin(),
      scan.begin() + scan.size() - num_pos,
      scan.end());
  }

  /*****************************************************************************
  */
  static void completeScan2(std::vector<double>& scan,
    const Pose& pose)
  {
    const std::vector<double> scan_copy = scan;

    /* Locate the first and last points of the scan in the 2D plane */
    const std::vector< std::pair<double,double> > points =
      Utils::scan2points(scan_copy, pose);
    const std::pair<double,double> start_point = points[0];
    const std::pair<double,double> end_point = points[points.size()-1];

    const double dx = start_point.first - end_point.first;
    const double dy = start_point.second - end_point.second;
    const double d = sqrt(dx*dx + dy*dy);
    const double r = d/2;

    for (int i = scan_copy.size()-2; i > 0; i--)
      scan.push_back(r);

    /* Rotate so that it starts from -M_PI rather than -M_PI / 2 */
    const int num_pos = scan.size() / 4;

    std::rotate(scan.begin(),
      scan.begin() + scan.size() - num_pos,
      scan.end());
  }

  /*****************************************************************************
  */
  static void completeScan3(std::vector<double>& scan)
  {
    const std::vector<double> scan_copy = scan;

    for (std::size_t i = 1; i < scan_copy.size()-1; i++)
      scan.push_back(scan_copy[i]);

    /* Rotate so that it starts from -M_PI rather than -M_PI / 2 */
    const int num_pos = scan.size() / 4;

    std::rotate(scan.begin(),
      scan.begin() + scan.size() - num_pos,
      scan.end());
  }

  /*****************************************************************************
  */
  static void completeScan4(std::vector<double>& scan)
  {
    /* Find closest and furthest points in original scan */
    const double min_range = *std::min_element(scan.begin(), scan.end());
    [[maybe_unused]] const double max_range = *std::max_element(scan.begin(), scan.end());
    const double fill_range = min_range;

    const unsigned int scan_size = scan.size();

    for (std::size_t i = 1; i < scan_size-1; i++)
      scan.push_back(fill_range);

    /* Rotate so that it starts from -M_PI rather than -M_PI / 2 */
    assert(fmod(scan.size(), 2) == 0);
    const int num_pos = scan.size() / 4;

    std::rotate(scan.begin(),
      scan.begin() + scan.size() - num_pos,
      scan.end());
  }

  /*****************************************************************************
  */
  static CompletedScan completeScan5(
    const Pose& pose,
    const std::span<const double> scan_in,
    const unsigned int& num_rays,
    const RaySearch ray_search)
  {
    const std::vector< std::pair<double,double> > scan_points =
      Utils::scan2points(scan_in, pose, M_PI);

    CompletedScan completed;

    Pose pose_within_points = pose;

    const double farther_than = 0.01;
    bool is_farther_than = false;

    while (!is_farther_than)
    {
      do pose_within_points = Utils::generatePose(pose, 0.05, 0.0);
      while(!Utils::isPositionInMap(pose_within_points, scan_points));

      completed.map =
        X::find(pose_within_points, scan_points, num_rays, ray_search);

      is_farther_than =
        Utils::isPositionFartherThan(pose_within_points, completed.map,
          farther_than);
    }

    completed.map_origin = pose_within_points;
    completed.scan = Utils::points2scan(completed.map, completed.map_origin);

    return completed;
  }
};


/* ========================================================================== */
/*
 * The transforms below cannot take their working buffers from a vector or from
 * new: FFTW's planner records the alignment of the arrays it was shown, and a
 * plan may only be executed on arrays aligned the same way, which is what
 * fftw_malloc guarantees and nothing else does.
 *
 * Every one of those buffers followed the same shape, allocate then fill then
 * execute then free, with the free written out by hand at the end of the
 * function. This gives them an owner instead.
 */
struct FFTWDeleter
{
  void operator()(void* const memory) const { fftw_free(memory); }
};

template <typename T>
using FFTWBuffer = std::unique_ptr<T[], FFTWDeleter>;

template <typename T>
FFTWBuffer<T> fftwBuffer(const std::size_t count)
{
  return FFTWBuffer<T>(static_cast<T*>(fftw_malloc(count * sizeof(T))));
}
/* ========================================================================== */
class DFTUtils
{
  public:

  /*****************************************************************************
   * This package builds under -Ofast, which implies -ffinite-math-only. Under
   * that setting the compiler is entitled to assume that no infinity or
   * not-a-number value ever occurs, so it is free to fold a call to
   * std::isfinite down to a constant true. The checks further down this class
   * exist to catch a non finite Fourier coefficient, so they cannot rely on a
   * call the optimiser is allowed to erase; the exponent bits are read
   * directly instead, following the reasoning already applied to
   * isValidRange in fsm_lidar_odometry.cpp for exactly this reason.
   * @brief Returns whether value holds neither infinity nor not-a-number.
   * @param[in] value [const double] The value to test.
   * @return [bool] True when value is finite.
   */
  static bool isFinite(const double value)
  {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    const std::uint64_t exponent = (bits >> 52) & 0x7FFU;

    return exponent != 0x7FFU;
  }

  /*****************************************************************************
   * Plans are expensive to create and, under FFTW_MEASURE, creating one runs
   * timing trials. The transforms below were creating and destroying one on
   * every call, at every oversampling size, which dominated their cost.
   *
   * Plans are keyed by size and kept for the life of the process. Creation is
   * serialised because FFTW's planner is not thread safe; execution is not,
   * because executing a plan on freshly supplied arrays is.
   */
  /*****************************************************************************
   * @brief Performs DFT in a vector of doubles via fftw. Returns the DFT
   * coefficients vector in the order described in
   * http://www.fftw.org/fftw3_doc/Real_002dto_002dReal-Transform-Kinds.html#Real_002dto_002dReal-Transform-Kinds
   * @param[in] rays_diff [const std::vector<double>&] The vector of differences
   * in range between a world scan and a map scan.
   * @return [std::vector<double>] The vector's DFT coefficients.
   */
  static std::vector<double> dft(const std::span<const double> rays_diff)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    const size_t num_rays = rays_diff.size();

    const FFTWBuffer<double> in = fftwBuffer<double>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    const fftw_plan p = forwardPlan(num_rays);

    /* Transfer the input vector to a structure preferred by fftw */
    for (unsigned int i = 0; i < num_rays; i++)
      in[i] = rays_diff[i];

    /* Execute plan */
    fftw_execute_r2r(p, in.get(), out.get());

    /* Store all DFT coefficients */
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [dft]",
      elapsed.count()));
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector<double> dft(const std::span<const double> rays_diff,
    const fftw_plan& r2rp)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    const size_t num_rays = rays_diff.size();

    const FFTWBuffer<double> in = fftwBuffer<double>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    /*
     * Create plan
     * fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);
     */

    /* Transfer the input vector to a structure preferred by fftw */
    for (unsigned int i = 0; i < num_rays; i++)
      in[i] = rays_diff[i];


    /* Execute plan */
    fftw_execute_r2r(r2rp, in.get(), out.get());

    /* Store all DFT coefficients */
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [dft]",
      elapsed.count()));
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > dftBatch(
    const std::vector< std::vector<double> >& scans)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    /* What will be returned */
    std::vector< std::vector<double> > coeff_vector_v;

    /* Input/output arrays for fftw */
    const size_t num_rays = scans[0].size();

    const FFTWBuffer<double> in = fftwBuffer<double>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    /* Create plan once */
    const fftw_plan p = forwardPlan(num_rays);

    for (unsigned int v = 0; v < scans.size(); v++)
    {
      /* Transfer the input vector to a structure preferred by fftw */
      for (unsigned int i = 0; i < num_rays; i++)
        in[i] = scans[v][i];

      /* Execute plan with new input/output arrays */
      fftw_execute_r2r(p, in.get(), out.get());

      /* Store all DFT coefficients for the v-th scan */
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]);

      coeff_vector_v.push_back(dft_coeffs);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [dftBatch]",
      elapsed.count()));
#endif

    return coeff_vector_v;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > dftBatch(
    const std::vector< std::vector<double> >& scans,
    const fftw_plan& r2rp)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    /* What will be returned */
    std::vector< std::vector<double> > coeff_vector_v;

    /* Input/output arrays for fftw */
    const size_t num_rays = scans[0].size();

    const FFTWBuffer<double> in = fftwBuffer<double>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    /*
     * Create plan once
     * fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);
     */

    for (unsigned int v = 0; v < scans.size(); v++)
    {
      /* Transfer the input vector to a structure preferred by fftw */
      for (unsigned int i = 0; i < num_rays; i++)
        in[i] = scans[v][i];

      /* Execute plan with new input/output arrays */
      fftw_execute_r2r(r2rp, in.get(), out.get());

      /* Store all DFT coefficients for the v-th scan */
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]);

      coeff_vector_v.push_back(dft_coeffs);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [dftBatch]",
      elapsed.count()));
#endif

    return coeff_vector_v;
  }

  /*****************************************************************************
  */
  static void fftshift(std::vector<double>& vec)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    std::rotate(
      vec.begin(),
      vec.begin() + static_cast<unsigned int>(vec.size()/2),
      vec.end());

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [fftshift]",
      elapsed.count()));
#endif
  }

  static fftw_plan forwardPlan(const std::size_t size)
  {
    static std::mutex mutex;
    static std::map<std::size_t, fftw_plan> plans;

    const std::lock_guard<std::mutex> lock(mutex);

    const auto found = plans.find(size);
    if (found != plans.end())
      return found->second;

    const FFTWBuffer<double> in = fftwBuffer<double>(size);
    const FFTWBuffer<double> out = fftwBuffer<double>(size);
    const fftw_plan plan = fftw_plan_r2r_1d(size, in.get(), out.get(),
      FFTW_R2HC, FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);

    plans.emplace(size, plan);
    return plan;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> >
  getDFTCoefficientsPairs(const std::span<const double> coeffs)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector< std::pair<double, double> > fft_coeff_pairs;
    for (std::size_t i = 0; i <= coeffs.size()/2; i++)
    {
      if (i == 0 || i == coeffs.size()/2)
        fft_coeff_pairs.push_back(std::make_pair(coeffs[i], 0.0));
      else
      {
        fft_coeff_pairs.push_back(
          std::make_pair(coeffs[i], coeffs[coeffs.size()-i]));
      }
    }

    const std::vector< std::pair<double, double> > fft_coeff_pairs_bak =
      fft_coeff_pairs;
    for (int i = fft_coeff_pairs_bak.size()-2; i > 0; i--)
    {
      fft_coeff_pairs.push_back(
        std::make_pair(fft_coeff_pairs_bak[i].first, -fft_coeff_pairs_bak[i].second));
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [getDFTCoefficientsPairs]",
      elapsed.count()));
#endif

    return fft_coeff_pairs;
  }

  /*****************************************************************************
   * @brief Calculates the X1 coefficient of the rays_diff input vector.
   * @param[in] rays_diff [const std::vector<double>&] The difference in range
   * between a world and a map scan.
   * @return [std::vector<double>] A vector of size two, of which the first
   * position holds the real part of the first DFT coefficient, and the
   * second the imaginary part of it.
   */
  static std::vector<double> getFirstDFTCoefficient(
    const std::span<const double> rays_diff)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    /* A vector holding the coefficients of the DFT */
    std::vector<double> dft_coeff_vector;

    /* Do the DFT thing */
    const std::vector<double> dft_coeffs = dft(rays_diff);

    /*
     * The real and imaginary part of the first coefficient are
     * out[1] and out[N-1] respectively
     */

    /* The real part of the first coefficient */
    const double x1_r = dft_coeffs[1];

    /* The imaginary part of the first coefficient */
    const double x1_i = dft_coeffs[rays_diff.size()-1];

    /*
     * std::isfinite folds to a constant true under this package's -Ofast
     * build, so a non finite coefficient would pass straight through
     * unnoticed. isFinite above reads the bits instead and cannot be folded
     * away the same manner.
     */
    /* Is x1_r finite? */
    if (isFinite(x1_r))
      dft_coeff_vector.push_back(x1_r);
    else
      dft_coeff_vector.push_back(0.0);

    /* Is x1_i finite? */
    if (isFinite(x1_i))
      dft_coeff_vector.push_back(x1_i);
    else
      dft_coeff_vector.push_back(0.0);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [getFirstDFTCoefficient]",
      elapsed.count()));
#endif

    return dft_coeff_vector;
  }

  /****************************************************************************
  */
  static std::vector<double> getFirstDFTCoefficient(
    const std::span<const double> rays_diff,
    const fftw_plan& r2rp)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    /* A vector holding the coefficients of the DFT */
    std::vector<double> dft_coeff_vector;

    /* Do the DFT thing */
    const std::vector<double> dft_coeffs = dft(rays_diff, r2rp);

    /*
     * The real and imaginary part of the first coefficient are
     * out[1] and out[N-1] respectively
     */

    /* The real part of the first coefficient */
    const double x1_r = dft_coeffs[1];

    /* The imaginary part of the first coefficient */
    const double x1_i = dft_coeffs[rays_diff.size()-1];

    /*
     * std::isfinite folds to a constant true under this package's -Ofast
     * build, so a non finite coefficient would pass straight through
     * unnoticed. isFinite above reads the bits instead and cannot be folded
     * away the same manner.
     */
    /* Is x1_r finite? */
    if (isFinite(x1_r))
      dft_coeff_vector.push_back(x1_r);
    else
      dft_coeff_vector.push_back(0.0);

    /* Is x1_i finite? */
    if (isFinite(x1_i))
      dft_coeff_vector.push_back(x1_i);
    else
      dft_coeff_vector.push_back(0.0);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [getFirstDFTCoefficient]",
      elapsed.count()));
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector<double> idft(
    const std::vector<std::pair<double, double> >& rays_diff)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    const size_t num_rays = rays_diff.size();

    const FFTWBuffer<fftw_complex> in = fftwBuffer<fftw_complex>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    const fftw_plan p = inversePlan(num_rays);

    /* Transfer the input vector to a structure preferred by fftw */
    for (unsigned int i = 0; i < num_rays; i++)
    {
      in[i][0] = rays_diff[i].first;
      in[i][1] = rays_diff[i].second;
    }

    /* Execute plan */
    fftw_execute_dft_c2r(p, in.get(), out.get());

    /* Store all DFT coefficients */
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]/num_rays);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [idft]",
      elapsed.count()));
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > idftBatch(
    const std::vector< std::vector<std::pair<double, double> > >& scans)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    /* What will be returned */
    std::vector< std::vector<double> > dft_coeffs_v;

    const size_t num_rays = scans[0].size();

    const FFTWBuffer<fftw_complex> in = fftwBuffer<fftw_complex>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    /* Create plan once */
    const fftw_plan p = inversePlan(num_rays);


    for (unsigned int v = 0; v < scans.size(); v++)
    {
      /* Transfer the input vector to a structure preferred by fftw */
      for (unsigned int i = 0; i < num_rays; i++)
      {
        in[i][0] = scans[v][i].first;
        in[i][1] = scans[v][i].second;
      }

      /* Execute plan */
      fftw_execute_dft_c2r(p, in.get(), out.get());

      /* Store all DFT coefficients */
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]/num_rays);

      dft_coeffs_v.push_back(dft_coeffs);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [idftBatch]",
      elapsed.count()));
#endif

    return dft_coeffs_v;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > idftBatch(
    const std::vector< std::vector<std::pair<double, double> > >& scans,
    const fftw_plan& c2rp)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    /* What will be returned */
    std::vector< std::vector<double> > dft_coeffs_v;

    const size_t num_rays = scans[0].size();

    const FFTWBuffer<fftw_complex> in = fftwBuffer<fftw_complex>(num_rays);
    const FFTWBuffer<double> out = fftwBuffer<double>(num_rays);

    /*
     * Create plan once
     * fftw_plan p = fftw_plan_dft_c2r_1d(num_rays, in, out, FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);
     */

    for (unsigned int v = 0; v < scans.size(); v++)
    {
      /* Transfer the input vector to a structure preferred by fftw */
      for (unsigned int i = 0; i < num_rays; i++)
      {
        in[i][0] = scans[v][i].first;
        in[i][1] = scans[v][i].second;
      }

      /* Execute plan */
      fftw_execute_dft_c2r(c2rp, in.get(), out.get());

      /* Store all DFT coefficients */
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]/num_rays);

      dft_coeffs_v.push_back(dft_coeffs);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    Diagnostics::report(std::format("{:f} [idftBatch]",
      elapsed.count()));
#endif

    return dft_coeffs_v;
  }

  /*****************************************************************************
  */
  static fftw_plan inversePlan(const std::size_t size)
  {
    static std::mutex mutex;
    static std::map<std::size_t, fftw_plan> plans;

    const std::lock_guard<std::mutex> lock(mutex);

    const auto found = plans.find(size);
    if (found != plans.end())
      return found->second;

    const FFTWBuffer<fftw_complex> in = fftwBuffer<fftw_complex>(size);
    const FFTWBuffer<double> out = fftwBuffer<double>(size);
    const fftw_plan plan =
      fftw_plan_dft_c2r_1d(size, in.get(), out.get(), FSM_LIDAR_ODOMETRY_FFTW_PLAN_FLAG);

    plans.emplace(size, plan);
    return plan;
  }
};


/* ========================================================================== */
class Translation
{
  public:

  /*****************************************************************************
  */
  static TranslationOutput tff(
    const std::span<const double> real_scan,
    const Pose& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const int& max_iterations,
    [[maybe_unused]] const double& dist_bound,
    const bool& pick_min,
    const fftw_plan& r2rp,
    const RaySearch ray_search)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("input pose  ({:f},{:f},{:f}) [Translation::tff]",
      virtual_pose.x,
      virtual_pose.y,
      virtual_pose.t));
#endif

    TranslationOutput output;

    /*
     * The orientation is not the translation stage's to change: it comes back
     * exactly as it went in. Both callers relied on that by passing the same
     * object as input and output, so the field was simply left alone. Saying
     * it plainly costs nothing and removes the aliasing the old shape needed.
     */
    output.pose.t = virtual_pose.t;

    Pose current_pose = virtual_pose;

    std::vector<double> errors_xy;

    std::vector<double> deltas;
    std::vector<double> sum_d_vs;
    std::vector<double> x_es;
    std::vector<double> y_es;
    /* Start the clock */
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

    /* Iterate */
    unsigned int it = 1;
    double inclusion_bound = 1000.0;
    double err = 1.0 / real_scan.size();
    std::vector<double> d_v;
    double sum_d_v = 1.0 / real_scan.size();

    for (it = 1; it <= static_cast<unsigned int>(max_iterations); it++)
    {
      /* Measure the time to find intersections */
      std::chrono::high_resolution_clock::time_point int_start =
        std::chrono::high_resolution_clock::now();

      /*
       * Find the intersections of the rays from the estimated pose and
       * the map.
       */
      const std::vector< std::pair<double,double> > virtual_scan_intersections =
        X::find(current_pose, map, real_scan.size(), ray_search);

      std::chrono::high_resolution_clock::time_point int_end =
        std::chrono::high_resolution_clock::now();
      output.intersections_time =
        std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

      /* Find the corresponding ranges */
      const std::vector<double> virtual_scan_it =
        Utils::points2scan(virtual_scan_intersections, current_pose);

      assert(virtual_scan_it.size() == real_scan.size());

      /*
       * inclusion_bound = real_scan.size()/2*err;
       * inclusion_bound = 0.01*sum_d;
       * inclusion_bound = M_PI * (sum_d + err) / real_scan.size();
       * inclusion_bound = 2*M_PI * sum_d_v / real_scan.size();
       */
      inclusion_bound = real_scan.size()/4*err;

      /* Obtain the correction vector */
      const TranslationCorrection correction =
        tffCore(real_scan, virtual_scan_it, current_pose.t,
          inclusion_bound, r2rp);

      d_v = correction.d_v;
      [[maybe_unused]] const double norm_x1 = correction.norm_x1;

      /* These are the corrections */
      double x_e = correction.x_e;
      double y_e = correction.y_e;


      /* The norm of the correction vector */
      double err_sq = x_e*x_e + y_e*y_e;
      err = sqrt(err_sq);

      /* Correct the position */
      current_pose.x += x_e;
      current_pose.y += y_e;

      [[maybe_unused]] const double dx = current_pose.x - virtual_pose.x;
      [[maybe_unused]] const double dy = current_pose.y - virtual_pose.y;

      /* Check constraints */
      if(!Utils::isPositionInMap(current_pose, map))
      {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
        Diagnostics::report("OUT OF BOUNDS");
#endif

        output.iterations = it;
        output.pose = current_pose;
        output.criterion = -2.0;
        output.out_of_map = true;
        return output;
      }

      /*
       * inclusion_bound =
       * pow(2,2)*(sum_d + err_sq)*(2*it + max_iterations) / max_iterations / real_scan.size(); 1125
       * inclusion_bound = pow(2,2) * (sum_d + err_sq) / real_scan.size(); 1142 3436
       * inclusion_bound = pow(2,2) * (sum_d + err) / real_scan.size(); 1144 3407
       * inclusion_bound = pow(2,2) * sum_d / real_scan.size(); 1155 3454
       * inclusion_bound = 0.01*sum_d; 1168 3487
       * inclusion_bound = 100*err;
       */

      for (unsigned int d = 0; d < d_v.size(); d++)
        d_v[d] = fabs(d_v[d]);

      sum_d_v = std::accumulate(d_v.begin(), d_v.end(), 0.0);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Diagnostics::report(std::format("err = {:f}",
        err));
      Diagnostics::report(std::format("norm_x1 = {:f}",
        norm_x1));
      Diagnostics::report(std::format("sum_d_v = {:f}",
        sum_d_v));
#endif

      if (pick_min)
      {
        x_es.push_back(x_e);
        y_es.push_back(y_e);
        sum_d_vs.push_back(sum_d_v);
      }

      /* Break if translation is negligible */
      const double eps = 0.0000001;
      if (fabs(x_e) < eps && fabs(y_e) < eps)
        break;
    }

    if (pick_min)
    {
      const std::vector<double> crit_v = sum_d_vs;
      unsigned int min_sum_d_idx =
        std::min_element(crit_v.begin(), crit_v.end()) -crit_v.begin();
      sum_d_v = sum_d_vs[min_sum_d_idx];
      const double x_tot = std::accumulate(x_es.begin(), x_es.begin()+min_sum_d_idx, 0.0);
      const double y_tot = std::accumulate(y_es.begin(), y_es.begin()+min_sum_d_idx, 0.0);

      output.pose.x = x_tot + virtual_pose.x;
      output.pose.y = y_tot + virtual_pose.y;
    }
    else
      output.pose = current_pose;

    output.iterations = it;

    /* Stop the clock */
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("output pose ({:f},{:f},{:f}) [Translation::tff]",
      output.pose.x,
      output.pose.y,
      output.pose.t));
#endif

    output.criterion = sum_d_v / real_scan.size();
    return output;
  }

  /*****************************************************************************
  */
  static TranslationCorrection tffCore(
    const std::span<const double> real_scan,
    const std::span<const double> virtual_scan,
    const double& current_t,
    const double& inclusion_bound,
    const fftw_plan& r2rp)
  {
    assert(inclusion_bound >= 0);

    const auto [diff, diff_true] =
      Utils::diffScansPerRay(real_scan, virtual_scan, inclusion_bound);

    /* X1 */
    const std::vector<double> X1 = DFTUtils::getFirstDFTCoefficient(diff, r2rp);

    const double norm_x1 = sqrt(X1[0]*X1[0] + X1[1]*X1[1]);

    /* Find the x-wise and y-wise errors */
    const double t = M_PI + current_t;
    const std::vector<double> errors_xy = turnDFTCoeffsIntoErrors(X1, diff.size(), t);

    const double x_e = errors_xy[0];
    const double y_e = errors_xy[1];

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("(x_e,y_e) = ({:f},{:f})",
      x_e, y_e));
#endif

    return TranslationCorrection{x_e, y_e, diff_true, norm_x1};
  }

  /*****************************************************************************
  */
  static std::vector<double> turnDFTCoeffsIntoErrors(
    const std::span<const double> dft_coeff,
    const int& num_valid_rays,
    const double& starting_angle)
  {
    double x_err = 0.0;
    double y_err = 0.0;

    if (num_valid_rays > 0)
    {
      /* The error in the x- direction */
      x_err = 1.0 / num_valid_rays *
        (-dft_coeff[0] * cos(starting_angle)
         -dft_coeff[1] * sin(starting_angle));

      /* The error in the y- direction */
      y_err = 1.0 / num_valid_rays *
        (-dft_coeff[0] * sin(starting_angle)
         +dft_coeff[1] * cos(starting_angle));
    }

    std::vector<double> errors;
    errors.push_back(x_err);
    errors.push_back(y_err);

    return errors;
  }

};


/* ========================================================================== */
class Rotation
{
public:

  /*****************************************************************************
  */
  static constexpr double angleById(const unsigned int& rotation_id,
    const unsigned int scan_size)
  {
    double dt = 2*M_PI*rotation_id / scan_size;

    dt = Utils::wrapAngle(dt);

    return dt;
  }

  /*****************************************************************************
  */
  static RotationOutput fmt(
    const std::span<const double> real_scan,
    const Pose& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    const std::string& batch_or_sequential,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    const RaySearch ray_search)
  {
    if (batch_or_sequential.compare("batch") == 0)
      return fmt2Batch(real_scan, virtual_pose, map, magnification_size,
        r2rp, c2rp, ray_search);
    else if (batch_or_sequential.compare("sequential") == 0)
      return fmt2Sequential(real_scan, virtual_pose, map, magnification_size,
        ray_search);
    else
    {
      Diagnostics::report("[Rotation::fmt] Use 'batch' or 'sequential' instead ");
      RotationOutput output; output.angles.push_back(-1.0); return output;
    }
  }

  /*****************************************************************************
  */
  static std::vector<Correlation> fmt0AutoBatch(
    const std::vector< std::vector<double> > & virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp)
  {
    assert(virtual_scans.size() > 0);

    /* Find fft of virtual scan */
    const std::vector< std::vector<double> > fft_virtuals =
      DFTUtils::dftBatch(virtual_scans, r2rp);

    std::vector< std::vector< std::pair<double, double> > > Q_0_v;

    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      /* Virtual scan dft coefficients */
      const std::vector< std::pair<double, double> > fft_virtual_coeffs =
        DFTUtils::getDFTCoefficientsPairs(fft_virtuals[i]);

      /* Virtual scan dft coefficients conjugates */
      const std::vector< std::pair<double, double> > fft_virtual_coeffs_conj =
        Utils::conjugate(fft_virtual_coeffs);

      /* The numerator of Q_0 */
      const std::vector< std::pair<double, double> > numerator =
        Utils::innerProductComplex(fft_virtual_coeffs_conj, fft_virtual_coeffs);

      Q_0_v.push_back(numerator);
    }

    const std::vector< std::vector<double> > q_0_v =
      DFTUtils::idftBatch(Q_0_v, c2rp);

    std::vector<Correlation> correlations;
    for (unsigned int i = 0; i < q_0_v.size(); i++)
    {
      const unsigned int q_0_max_id =
        std::max_element(q_0_v.at(i).begin(), q_0_v.at(i).end())
        -q_0_v.at(i).begin();

      correlations.push_back(Correlation{q_0_v[i], q_0_max_id});
    }

    return correlations;
  }

  /*****************************************************************************
  */
  static Correlation fmt0AutoSequential(
    const std::span<const double> real_scan)
  {
    /* Find fft of real scan */
    const std::vector<double> fft_real = DFTUtils::dft(real_scan);

    const std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);

    /* Find conjugates of real coefficients */
    const std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    /* The numerator of Q_0 */
    const std::vector< std::pair<double, double> > numerator =
      Utils::innerProductComplex(fft_real_coeffs_conj, fft_real_coeffs);

    const std::vector< std::pair<double, double> > Q_0 = numerator;

    Correlation correlation;
    correlation.q_0 = DFTUtils::idft(Q_0);
    correlation.q_0_max_id =
      std::max_element(correlation.q_0.begin(), correlation.q_0.end())
      - correlation.q_0.begin();

    return correlation;
  }

  /*****************************************************************************
  */
  static std::vector<Correlation> fmt0Batch(
    const std::span<const double> real_scan,
    const std::vector< std::vector<double> > & virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp)
  {
    assert(virtual_scans.size() > 0);
    assert(real_scan.size() == virtual_scans[0].size());

    /* Find fft of real scan */
    const std::vector<double> fft_real = DFTUtils::dft(real_scan);

    /* Find fft of virtual scan */
    const std::vector< std::vector<double> > fft_virtuals =
      DFTUtils::dftBatch(virtual_scans, r2rp);

    /*
     * fft_real is in halfcomplex format; fft_real_coeffs is in normal format
     * (you get the full complex transform)
     */
    const std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);

    /* Find conjugates of real coefficients */
    const std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    std::vector< std::vector< std::pair<double, double> > > Q_0_v;
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      const std::vector< std::pair<double, double> > fft_virtual_coeffs =
        DFTUtils::getDFTCoefficientsPairs(fft_virtuals[i]);

      /* The numerator of Q_0 */
      const std::vector< std::pair<double, double> > numerator =
        Utils::innerProductComplex(fft_real_coeffs_conj, fft_virtual_coeffs);

      /*
       * / * The denominator of Q_0 * /
       * double denominator =
       * Utils::norm2(fft_real_coeffs) * Utils::norm2(fft_virtual_coeffs);

       * for (int i = 0; i < numerator.size(); i++)
       * {
       * numerator[i].first /= denominator;
       * numerator[i].second /= denominator;
       * }
      */

      Q_0_v.push_back(numerator);
    }

    const std::vector< std::vector<double> > q_0_v =
      DFTUtils::idftBatch(Q_0_v, c2rp);

    std::vector<Correlation> correlations;
    for (unsigned int i = 0; i < q_0_v.size(); i++)
    {
      const unsigned int q_0_max_id =
        std::max_element(q_0_v.at(i).begin(), q_0_v.at(i).end())
        - q_0_v.at(i).begin();

      correlations.push_back(Correlation{q_0_v[i], q_0_max_id});
    }

    return correlations;
  }

  /*****************************************************************************
  */
  static Correlation fmt0Sequential(
    const std::span<const double> real_scan,
    const std::span<const double> virtual_scan)
  {
    assert(real_scan.size() == virtual_scan.size());

    /* Find fft of real scan */
    const std::vector<double> fft_real = DFTUtils::dft(real_scan);
    /*
     * DFTUtils::fftshift(&fft_real);
     */

    /* Find fft of virtual scan */
    const std::vector<double> fft_virtual = DFTUtils::dft(virtual_scan);
    /*
     * DFTUtils::fftshift(&fft_virtual);
     */

    /*
     * fft_real is in halfcomplex format; fft_real_coeffs is in normal format
     * (you get the full complex transform)
     */
    const std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);
    const std::vector< std::pair<double, double> > fft_virtual_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_virtual);

    /* Find conjugates of real coefficients */
    const std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    /* The numerator of Q_0 */
    const std::vector< std::pair<double, double> > numerator =
      Utils::innerProductComplex(fft_real_coeffs_conj, fft_virtual_coeffs);

    /* The denominator of Q_0 */
    [[maybe_unused]] const double denominator =
      Utils::norm2(fft_real_coeffs) * Utils::norm2(fft_virtual_coeffs);

    /*
     * for (int i = 0; i < numerator.size(); i++)
     * {
     * numerator[i].first /= denominator;
     * numerator[i].second /= denominator;
     * }
     */

    const std::vector< std::pair<double, double> > Q_0 = numerator;

    Correlation correlation;
    correlation.q_0 = DFTUtils::idft(Q_0);
    correlation.q_0_max_id =
      std::max_element(correlation.q_0.begin(), correlation.q_0.end())
      - correlation.q_0.begin();

    return correlation;
  }

  /*****************************************************************************
  */
  static std::vector<FMTOutput> fmt1Batch(
    const std::span<const double> real_scan,
    const std::vector< std::vector< double > >& virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp)
  {
    const std::vector<Correlation> correlations =
      fmt0Batch(real_scan, virtual_scans, r2rp, c2rp);

    /* Calculate PD */
    const auto [q_ss, q_ss_max_id] = fmt0AutoSequential(real_scan);

    const std::vector<Correlation> auto_correlations =
      fmt0AutoBatch(virtual_scans, r2rp, c2rp);

    std::vector<FMTOutput> outputs;
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      const std::vector<double>& q_0_i = correlations[i].q_0;
      const unsigned int q_0_max_id_i = correlations[i].q_0_max_id;

      FMTOutput output;

      /* Calculate angle */
      double angle = static_cast<double>(
        (real_scan.size()-q_0_max_id_i))/(real_scan.size())*2*M_PI;
      angle = Utils::wrapAngle(angle);

      output.angle = angle;

      /* Calculate pd */
      const double pd = 2*q_0_i[q_0_max_id_i]
        / (q_ss[q_ss_max_id]
          + auto_correlations[i].q_0[auto_correlations[i].q_0_max_id]);
      output.pd = pd;

      /* Calculate SNR */
      std::vector<double> q_0_background = q_0_i;
      q_0_background.erase(q_0_background.begin() + q_0_max_id_i);

      const std::pair<double,double> q_0_mmnts = Utils::vectorStatistics(q_0_background);

      const double snr =
        fabs((q_0_i[q_0_max_id_i] - q_0_mmnts.first)) / q_0_mmnts.second;
      output.snr = snr;

      /* Calculate FAHM */
      unsigned int count = 0;
      for (unsigned int f = 0; f < q_0_i.size(); f++)
      {
        if (q_0_i[f] >= 0.5 * q_0_i[q_0_max_id_i])
          count++;
      }

      const double fahm = static_cast<double>(count) / q_0_i.size();
      output.fahm = fahm;

      outputs.push_back(output);
    }

    return outputs;
  }

  /*****************************************************************************
  */
  static FMTOutput fmt1Sequential(
    const std::span<const double> real_scan,
    const std::span<const double> virtual_scan)
  {
    const auto [q_0, q_0_max_id] = fmt0Sequential(real_scan, virtual_scan);

    FMTOutput output;

    /* Calculate angle */
    output.angle = static_cast<double>(
      (real_scan.size()-q_0_max_id))/(real_scan.size())*2*M_PI;
    output.angle = Utils::wrapAngle(output.angle);

    /* Calculate SNR */
    std::vector<double> q_0_background = q_0;
    q_0_background.erase(q_0_background.begin() + q_0_max_id);

    const std::pair<double,double> q_0_mmnts = Utils::vectorStatistics(q_0_background);

    output.snr = fabs((q_0[q_0_max_id] - q_0_mmnts.first)) / q_0_mmnts.second;

    /* Calculate FAHM */
    unsigned int count = 0;
    for (unsigned int i = 0; i < q_0.size(); i++)
    {
      if (q_0[i] >= 0.5 * q_0[q_0_max_id])
        count++;
    }

    output.fahm = static_cast<double>(count) / q_0.size();

    /* Calculate PD */
    const auto [q_ss, q_ss_max_id] = fmt0Sequential(real_scan, real_scan);

    const auto [q_rr, q_rr_max_id] = fmt0Sequential(virtual_scan, virtual_scan);

    output.pd = 2*q_0[q_0_max_id] / (q_ss[q_ss_max_id] + q_rr[q_rr_max_id]);

    return output;
  }

  /***************************************************************************
   * FMT batch execution functions (faster)
   */
  static RotationOutput fmt2Batch(
    const std::span<const double> real_scan,
    const Pose& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    const RaySearch ray_search)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("input pose  ({:f},{:f},{:f}) [Rotation::fmt2]",
      virtual_pose.x,
      virtual_pose.y,
      virtual_pose.t));
#endif

    RotationOutput output;

    Pose zero_pose;
    zero_pose.x = 0.0;
    zero_pose.y = 0.0;
    zero_pose.t = 0.0;


    const unsigned int num_virtual_scans = pow(2,magnification_size);
    const int virtual_scan_size_max = num_virtual_scans * real_scan.size();

    /* Measure the time to find intersections */
    std::chrono::high_resolution_clock::time_point int_start =
      std::chrono::high_resolution_clock::now();

    const std::vector< std::pair<double,double> > virtual_scan_points =
      X::find(virtual_pose, map, virtual_scan_size_max, ray_search);

    std::chrono::high_resolution_clock::time_point int_end =
      std::chrono::high_resolution_clock::now();
    output.intersections_time =
      std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

    const std::vector<double> virtual_scan_fine =
      Utils::points2scan(virtual_scan_points, virtual_pose);

    /*
     * Downsample from upper limit:
     * construct the upper-most resolution and downsample from there.
     */
    std::vector< std::vector< double> > virtual_scans(num_virtual_scans);

    for (std::size_t i = 0; i < virtual_scan_fine.size(); i++)
    {
      unsigned int k = fmod(i,num_virtual_scans);
      virtual_scans[k].push_back(virtual_scan_fine[i]);
    }

    /*
     * Make sure that all virtual scans are equal to the real scan in terms of
     * size
     */
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
      assert(virtual_scans[i].size() == real_scan.size());

    /* The real scan's (the original) angle increment */
    const double ang_inc = 2*M_PI / real_scan.size();
    const double mul = 1.0 / num_virtual_scans;

    /*
     * Compute the angles and metrics of matching the real scan against each and
     * all virtual scans
     */
    const std::vector<FMTOutput> fmt_outputs =
      fmt1Batch(real_scan, virtual_scans, r2rp, c2rp);

    std::vector<double> un_angles;
    std::vector<double> snrs;
    std::vector<double> fahms;
    std::vector<double> pds;

    for (const FMTOutput& fmt_output : fmt_outputs)
    {
      un_angles.push_back(fmt_output.angle);
      snrs.push_back(fmt_output.snr);
      fahms.push_back(fmt_output.fahm);
      pds.push_back(fmt_output.pd);
    }

    /*
     * Correct the angles returned to get the proper pose from which each
     * virtual scan was taken (needed due to over-sampling the map)
     */
    std::vector<double> angles;
    for (unsigned int a = 0; a < num_virtual_scans; a++)
    {
      double angle_a = -un_angles[a] + a*mul*ang_inc;
      angle_a = Utils::wrapAngle(angle_a);

      angles.push_back(angle_a);
    }

    /* Select some of all the angles based on criteria enforced by rankFMTOutput */
    const std::vector<unsigned int> optimal_ids =
      rankFMTOutput(snrs, fahms, pds, 3, magnification_size, 0.00001);

    for (unsigned int i = 0; i < optimal_ids.size(); i++)
    {
      output.angles.push_back(angles[optimal_ids[i]]);

      output.rc0.push_back(pds[optimal_ids[i]]);
      output.rc1.push_back(snrs[optimal_ids[i]] / fahms[optimal_ids[i]]);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    for (unsigned int i = 0; i < output.angles.size(); i++)
    {
      Diagnostics::report(std::format("cand. poses ({:f},{:f},{:f}) [Rotation::fmt2]",
        virtual_pose.x,
        virtual_pose.y,
        virtual_pose.t+output.angles[i]));
    }
#endif

    return output;
  }

  /***************************************************************************
   * FMT sequential execution functions (slower)
   */
  static RotationOutput fmt2Sequential(
    const std::span<const double> real_scan,
    const Pose& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    const RaySearch ray_search)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("input pose  ({:f},{:f},{:f}) [Rotation::fmt2]",
      virtual_pose.x,
      virtual_pose.y,
      virtual_pose.t));
#endif

    RotationOutput output;

    Pose zero_pose;
    zero_pose.x = 0.0;
    zero_pose.y = 0.0;
    zero_pose.t = 0.0;


    const unsigned int num_virtual_scans = pow(2,magnification_size);
    const int virtual_scan_size_max = num_virtual_scans * real_scan.size();

    /* Measure the time to find intersections */
    std::chrono::high_resolution_clock::time_point int_start =
      std::chrono::high_resolution_clock::now();

    const std::vector< std::pair<double,double> > virtual_scan_points =
      X::find(virtual_pose, map, virtual_scan_size_max, ray_search);

    std::chrono::high_resolution_clock::time_point int_end =
      std::chrono::high_resolution_clock::now();
    output.intersections_time =
      std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

    const std::vector<double> virtual_scan_fine =
      Utils::points2scan(virtual_scan_points, virtual_pose);

    /*
     * Downsample from upper limit:
     * construct the upper-most resolution and downsample from there.
     */
    std::vector< std::vector< double> > virtual_scans(num_virtual_scans);

    for (std::size_t i = 0; i < virtual_scan_fine.size(); i++)
    {
      unsigned int k = fmod(i,num_virtual_scans);
      virtual_scans[k].push_back(virtual_scan_fine[i]);
    }

    /*
     * Make sure that all virtual scans are equal to the real scan in terms of
     * size
     */
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
      assert(virtual_scans[i].size() == real_scan.size());

    /* The real scan's (the original) angle increment */
    const double ang_inc = 2*M_PI / real_scan.size();
    const double mul = 1.0 / num_virtual_scans;


    std::vector<double> orientations;
    std::vector<double> snrs;
    std::vector<double> fahms;
    std::vector<double> pds;

    for (unsigned int a = 0; a < num_virtual_scans; a++)
    {
      const auto [angle, snr, fahm, pd] =
        fmt1Sequential(real_scan, virtual_scans[a]);

      double ornt_a = -angle + a*mul*ang_inc;
      ornt_a = Utils::wrapAngle(ornt_a);

      orientations.push_back(ornt_a);
      snrs.push_back(snr);
      fahms.push_back(fahm);
      pds.push_back(pd);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Diagnostics::report(std::format("a = {}",
        a));
      Diagnostics::report(std::format("angle to out = {:f}",
        virtual_pose.t + ornt_a));
      Diagnostics::report(std::format("snr = {:.10f}",
        snr));
      Diagnostics::report(std::format("fahm = {:f}",
        fahm));
      Diagnostics::report(std::format("pd = {:.20f}",
        pd));
#endif
    }

    /* Select some of all the angles based on criteria enforced by rankFMTOutput */
    const std::vector<unsigned int> optimal_ids =
      rankFMTOutput(snrs, fahms, pds, 3, magnification_size, 0.00001);

    for (unsigned int i = 0; i < optimal_ids.size(); i++)
    {
      double angle = orientations[optimal_ids[i]];
      angle = Utils::wrapAngle(angle);
      output.angles.push_back(angle);

      output.rc0.push_back(pds[optimal_ids[i]]);
      output.rc1.push_back(snrs[optimal_ids[i]] / fahms[optimal_ids[i]]);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    for (unsigned int i = 0; i < output.angles.size(); i++)
    {
      Diagnostics::report(std::format("cand. poses ({:f},{:f},{:f}) [Rotation::fmt2]",
        virtual_pose.x,
        virtual_pose.y,
        virtual_pose.t+output.angles[i]));
    }
#endif

    return output;
  }

  /*****************************************************************************
  */
  static std::vector<unsigned int> rankFMTOutput(
    const std::span<const double> snr,
    const std::span<const double> fahm,
    const std::span<const double> pd,
    const unsigned int& method,
    const unsigned int& magnification_size,
    const double& pd_threshold)
  {
    assert (snr.size() == fahm.size());
    assert (fahm.size() == pd.size());
    assert (pd_threshold >= 0);
    assert (method <= 3);

    /*
     * Return the indices of those angles for which criteria are near
     * the maximum criterion
     */
    std::vector<unsigned int> best_ids;

    /* Simply the one please */
    if (method == 0)
    {
      /* What are the criteria for ranking angles? */
      const std::vector<double> criteria(pd.begin(), pd.end());

      /* Identify maximum criterion */
      [[maybe_unused]] const double max_c = *std::max_element(criteria.begin(), criteria.end());

      best_ids.push_back(
        std::max_element(criteria.begin(), criteria.end()) -criteria.begin());
    }

    /* The one + those within pd_threshold around it */
    if (method == 1)
    {
      /* What are the criteria for ranking angles? */
      const std::vector<double> criteria(pd.begin(), pd.end());

      /* Identify maximum criterion */
      const double max_c = *std::max_element(criteria.begin(), criteria.end());

      for (unsigned int i = 0; i < criteria.size(); i++)
      {
        if (fabs(criteria[i]-max_c) <= pd_threshold)
          best_ids.push_back(i);
      }
    }

    /* The one + those within (max critetia - min crtieria)/2 */
    if (method == 2)
    {
      /* What are the criteria for ranking angles? */
      const std::vector<double> criteria(pd.begin(), pd.end());

      /* Identify maximum criterion */
      const double max_c = *std::max_element(criteria.begin(), criteria.end());
      const double min_c = *std::min_element(criteria.begin(), criteria.end());


      for (unsigned int i = 0; i < criteria.size(); i++)
      {
        if (fabs(criteria[i]-max_c) <= (max_c - min_c)/2)
          best_ids.push_back(i);
      }
    }

    /* Pick `pick_num_surr` around max criterion every time */
    std::set<unsigned int> best_ids_set;
    if (method == 3)
    {
      /* What are the criteria for ranking angles? */
      std::vector<double> criteria(pd.begin(), pd.end());

      /* Identify maximum criterion */
      const int max_c_idx =
        std::max_element(criteria.begin(), criteria.end()) - criteria.begin();
      [[maybe_unused]] const double max_c = criteria[max_c_idx];

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Diagnostics::report(std::format("best id = {}",
        max_c_idx));
#endif

      const int vendalia_method = 1;

      int pick_num_surr = 0;
      if (vendalia_method == 0)
      {
        pick_num_surr = pow(2,magnification_size) / pow(2,3);
        if (pick_num_surr == 0)
          pick_num_surr = 1;
      }
      if (vendalia_method == 1)
        pick_num_surr = pow(2,7) / pow(2,magnification_size);
      if (vendalia_method == 2)
        pick_num_surr = 4;

      for (int i =  -pick_num_surr + max_c_idx;
        i <= +pick_num_surr + max_c_idx; i++)
      {
        int k = i;

        while(k < 0)
          k += criteria.size();

        while (k > static_cast<int>(criteria.size()))
          k -= criteria.size();

        if (k == static_cast<int>(criteria.size()))
          k = 0;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
        Diagnostics::report(std::format("k = {}",
          k));
#endif
        best_ids_set.insert(k);
      }

      /*
       * for (unsigned int i = 0; i < criteria.size(); i++)
       * {
       * if (fabs(criteria[i]-max_c) <= pd_threshold)
       * best_ids_set.insert(i);
       * }
       */

      for (std::set<unsigned int>::iterator it = best_ids_set.begin();
        it != best_ids_set.end(); it++) best_ids.push_back(*it);
    }

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report("BEST IDS = [");
    for (unsigned int i = 0; i < best_ids.size(); i++)
      Diagnostics::report(std::format("{} ",
        best_ids[i]));

    Diagnostics::report("]");
#endif

    return best_ids;
  }
};


/* ========================================================================== */
class Match
{
  public:

  /*****************************************************************************
  */
  static bool canGiveNoMore(
    const std::span<const double> xs,
    const std::span<const double> ys,
    const std::span<const double> ts,
    const double& xy_eps,
    const double& t_eps)
  {
    assert(xs.size() == ys.size());

    const unsigned int sz = xs.size();
    bool xy_converged = false;
    bool t_converged = false;

    if (sz < 2)
      return false;
    else
    {
      for (unsigned int i = 2; i < sz; i++)
      {
        if (fabs(ts[sz-1] - ts[sz-i]) < t_eps)
          t_converged = true;

        if (fabs(xs[sz-1] - xs[sz-i]) < xy_eps &&
          fabs(ys[sz-1] - ys[sz-i]) < xy_eps)
          xy_converged = true;

        if (xy_converged && t_converged)
          return true;
      }

      return false;
    }
  }

  /*****************************************************************************
  */
  static MatchOutput fmtdbh(
    const std::span<const double> real_scan,
    const Pose& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    const input_params& ip)
  {
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

    MatchOutput match;
    output_params* const op = &match.op;
    Pose* const result_pose = &match.pose;

    *result_pose = virtual_pose;

    /* Maximum counter value means a new recovery attempt */
    int min_counter = 0;
    const int max_counter = ip.max_counter;
    int counter = min_counter;

    /* By a factor of what do you need to over-sample angularly? */
    unsigned int min_magnification_size = ip.min_magnification_size;
    const unsigned int max_magnification_size = ip.max_magnification_size;
    unsigned int current_magnification_size = min_magnification_size;

    /* How many times do I attempt recovery? */
    unsigned int num_recoveries = 0;
    const unsigned int max_recoveries = ip.max_recoveries;

    /* These three vectors hold the trajectory for each iteration */
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> ts;

    /* Two rotation criteria */
    std::vector<double> rc0_v;
    std::vector<double> rc1_v;

    /* One translation criterion */
    std::vector<double> tc_v;

    std::vector<double> dxys;
    std::chrono::duration<double> intersections_time;

    /*
     * The best candidate angle found at each iterations is stored and made a
     * candidate each time. Its criterion is its translation criterion after
     * ni-1 translations
     */
    double best_cand_angle = 0.0;
    double best_min_tc = 100000.0;

    /*
     * best_cand_angle's own pair of rotation criteria, carried alongside it.
     * rc0 and rc1, below, hold one entry per angle the rotation stage
     * returned this iteration, and a carried-over best_cand_angle is not one
     * of them, so neither vector has an entry that belongs to it. What does
     * belong to it is the pair recorded here whenever it is set below, the
     * one moment it is a genuine member of the rotation stage's own output.
     * Before that has happened the pair cannot mean anything and holds -2.0,
     * the sentinel this function already uses elsewhere for "not a real
     * reading".
     */
    double best_cand_rc0 = -2.0;
    double best_cand_rc1 = -2.0;

    /* A lock for going overdrive when the rotation criterion is near-excellent */
    [[maybe_unused]] const bool up_lock = false;
    int total_iterations = 0;
    int num_iterations = 0;

    while (current_magnification_size <= max_magnification_size)
    {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Diagnostics::report(std::format("current_magnification_size = {} ---",
        current_magnification_size));
      Diagnostics::report(std::format("counter                    = {} ---",
        counter));
#endif

      /*
       * ----------------- Rotation correction phase ---------------------------
       */
#ifdef FSM_LIDAR_ODOMETRY_TRACE
      std::chrono::high_resolution_clock::time_point start_rotation =
        std::chrono::high_resolution_clock::now();
#endif

      const RotationOutput rotation_output = Rotation::fmt(real_scan,
        *result_pose, map, current_magnification_size, "batch", r2rp, c2rp,
        ip.ray_search);

      const std::vector<double>& rc0 = rotation_output.rc0;
      const std::vector<double>& rc1 = rotation_output.rc1;
      std::vector<double> cand_angles = rotation_output.angles;

      intersections_time = rotation_output.intersections_time;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      std::chrono::high_resolution_clock::time_point end_rotation =
        std::chrono::high_resolution_clock::now();

      op->rotation_times += std::chrono::duration_cast<
        std::chrono::duration<double> >(end_rotation-start_rotation).count();

      op->intersections_times += intersections_time.count();
#endif

      bool ca_exists = false;
      for (unsigned int i = 0; i < cand_angles.size(); i++)
      {
        if (cand_angles[i] == best_cand_angle)
        {
          ca_exists = true;
          break;
        }
      }
      if (!ca_exists)
        cand_angles.push_back(best_cand_angle);

      /* ---------------- Candidate angles sifting -------------------------- */
      unsigned int min_tc_idx = 0;
      if (cand_angles.size() > 1)
      {
        std::vector<double> tcs_sift;
        for (unsigned int ca = 0; ca < cand_angles.size(); ca++)
        {
          /* How many test iterations? */
          const unsigned int ni = 2;
          [[maybe_unused]] int tr_i = 0;

          Pose cand_pose = *result_pose;
          cand_pose.t += cand_angles[ca];

#ifdef FSM_LIDAR_ODOMETRY_TRACE
          std::chrono::high_resolution_clock::time_point start_translation =
            std::chrono::high_resolution_clock::now();
#endif

          const TranslationOutput translation_output =
            Translation::tff(real_scan, cand_pose, map,
              ni, ip.xy_bound, false, r2rp, ip.ray_search);

          const double tc = translation_output.criterion;
          tr_i = translation_output.iterations;
          intersections_time = translation_output.intersections_time;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
          std::chrono::high_resolution_clock::time_point end_translation =
            std::chrono::high_resolution_clock::now();

          op->translation_times += std::chrono::duration_cast<
            std::chrono::duration<double> >(end_translation-start_translation).count();

          op->intersections_times += intersections_time.count();
#endif

#ifdef FSM_LIDAR_ODOMETRY_TRACE
          op->translation_iterations += tr_i;
#endif

          if (translation_output.out_of_map)
            tcs_sift.push_back(1000000.0);
          else
            tcs_sift.push_back(tc);
        }

        /* The index of the angle with the least translation criterion */
        min_tc_idx =
          std::min_element(tcs_sift.begin(), tcs_sift.end()) - tcs_sift.begin();

        /*
         * Check if the newly-found angle is the angle with the least
         * translation criterion so far
         */
        if (tcs_sift[min_tc_idx] < best_min_tc)
        {
          best_min_tc = tcs_sift[min_tc_idx];
          best_cand_angle = cand_angles[min_tc_idx];

          /*
           * A genuine rotation candidate carries its own rc0/rc1. When the
           * winner is instead the carried-over angle appended below,
           * cand_angles[min_tc_idx] is best_cand_angle already and the pair
           * recorded for it last time still applies, so nothing is
           * overwritten here in that case.
           */
          if (min_tc_idx < rc0.size())
          {
            best_cand_rc0 = rc0[min_tc_idx];
            best_cand_rc1 = rc1[min_tc_idx];
          }
        }
      }

      /*
       * min_tc_idx indexes cand_angles, which carries the rotation stage's
       * own candidates plus best_cand_angle, appended above when it was not
       * already among them. rc0 and rc1 carry only the former, one entry per
       * angle the rotation stage returned, with no entry for the appended
       * one. Reading rc0[min_tc_idx] and rc1[min_tc_idx] unconditionally
       * therefore read one past the end of both vectors whenever the
       * appended, carried-over angle wins the sift, which happens whenever
       * the previous best beats every angle found this iteration: undefined
       * behaviour that a release build does not catch, and that then flows
       * into the reported rotation criterion. Reading from rc0/rc1 only when
       * min_tc_idx is a genuine index into them, and from the pair carried
       * alongside best_cand_angle otherwise, records a real measurement
       * either way.
       */
      if (min_tc_idx < rc0.size())
      {
        rc0_v.push_back(rc0[min_tc_idx]);
        rc1_v.push_back(rc1[min_tc_idx]);
      }
      else
      {
        rc0_v.push_back(best_cand_rc0);
        rc1_v.push_back(best_cand_rc1);
      }

      /*
       * Update the current orientation estimate with the angle that sports the
       * least translation criterion overall
       */
      result_pose->t += cand_angles[min_tc_idx];
      result_pose->t = Utils::wrapAngle(result_pose->t);

      /* ... and store it */
      ts.push_back(result_pose->t);

      /*
       * ---------------- Translation correction phase -------------------------
       */
      num_iterations =
        (current_magnification_size+1)*ip.num_iterations;

      [[maybe_unused]] int tr_iterations = -1;
      [[maybe_unused]] const double int_time_trans = 0.0;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      std::chrono::high_resolution_clock::time_point start_translation =
        std::chrono::high_resolution_clock::now();
#endif

      const TranslationOutput translation_output = Translation::tff(real_scan,
        *result_pose, map, num_iterations, ip.xy_bound, true, r2rp,
        ip.ray_search);

      const double trans_criterion = translation_output.criterion;
      tr_iterations = translation_output.iterations;
      intersections_time = translation_output.intersections_time;
      *result_pose = translation_output.pose;

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      std::chrono::high_resolution_clock::time_point end_translation =
        std::chrono::high_resolution_clock::now();

      op->translation_times += std::chrono::duration_cast<
        std::chrono::duration<double> >(end_translation-start_translation).count();

      op->intersections_times += intersections_time.count();
#endif

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      op->translation_iterations += tr_iterations;
#endif

      tc_v.push_back(trans_criterion);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Diagnostics::report(std::format("rc0 = {:f}",
        rc0_v.back()));
      Diagnostics::report(std::format("rc1 = {:f}",
        rc1_v.back()));
      Diagnostics::report(std::format("tc  = {:f}",
        tc_v.back()));
#endif

      xs.push_back(result_pose->x);
      ys.push_back(result_pose->y);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
      Pose traj_i;
      traj_i.x = xs.back();
      traj_i.y = ys.back();
      traj_i.t = ts.back();
      op->trajectory.push_back(traj_i);
#endif


      /* --------------------- Recovery modes ------------------------------- */
      bool l2_recovery = false;

      /* Perilous pose at exterior of map's bounds detected */
      if (translation_output.out_of_map)
      {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
        Diagnostics::report("Will trigger recovery due to condition 0");
#endif
        l2_recovery = true;
      }

      /* Do not allow more than `max_counter` iterations per resolution */
      if (counter > max_counter)
      {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
        Diagnostics::report("Will trigger recovery due to condition 4");
#endif
        /* l2_recovery = true; */

        counter = 0;
        current_magnification_size++;
      }


      /* Recover if need be */
      if (l2_recovery)
      {
        if (num_recoveries > max_recoveries)
        {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
          Diagnostics::report("ERROR: MAXIMUM RECOVERIES");
#endif
          break;
        }

        num_recoveries++;
        *result_pose = l2recovery(virtual_pose, map, ip.xy_bound, ip.t_bound,
          ip.ray_search, ip.rng_seed);

        counter = min_counter;
        current_magnification_size = min_magnification_size;
      }
      else
      {
        counter++;

        /* -------------------------- Level-up ------------------------------ */
        const double xy_eps = 10.1;
        const double t_eps = 0.00001;
        if (canGiveNoMore(xs,ys,ts, xy_eps, t_eps) && counter > min_counter)
        {
          current_magnification_size += 1;
          counter = 0;
        }
      }

      total_iterations++;
    }

    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report(std::format("{:f} [Match::fmt]",
      elapsed.count()));
#endif

    op->exec_time = elapsed.count();
    op->rc = rc0_v.back();
    op->tc = tc_v.back();
    op->num_recoveries = num_recoveries;
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    op->rotation_iterations = total_iterations;
#endif

    return match;
  }

  /*****************************************************************************
  */
  static Pose l2recovery(
    const Pose& input_pose,
    const std::vector< std::pair<double,double> >& map,
    const double& xy_bound, const double& t_bound,
    const RaySearch ray_search,
    const unsigned int seed = 0)
  {
#ifdef FSM_LIDAR_ODOMETRY_TRACE
    Diagnostics::report("*********************************");
    Diagnostics::report("************CAUTION**************");
    Diagnostics::report("Level 2 recovery mode activated");
    Diagnostics::report("*********************************");
#endif

    std::optional<Pose> output_pose;
    while (!(output_pose = Utils::generatePose(input_pose, map,
        1*xy_bound, t_bound, 0.0, ray_search, seed)));

    return *output_pose;
  }
};
}
#endif
