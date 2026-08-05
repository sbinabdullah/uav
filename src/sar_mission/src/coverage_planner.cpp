#include "coverage_planner.hpp"

#include <algorithm>
#include <cmath>

namespace sar_mission
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
}

CoveragePlanner::CoveragePlanner(const GridConfig & cfg)
: cfg_(cfg), lane_spacing_(0.0)
{
  const double h = cfg_.altitude_m;
  const double across = 2.0 * h * std::tan(0.5 * cfg_.hfov_deg * kDegToRad);
  const double along = 2.0 * h * std::tan(0.5 * cfg_.vfov_deg * kDegToRad);
  const double footprint = std::min(across, along);
  const double overlap = std::clamp(cfg_.side_overlap, 0.0, 0.9);
  lane_spacing_ = std::max(1.0, footprint * (1.0 - overlap));
}

std::vector<Waypoint> CoveragePlanner::plan() const
{
  const double heading = cfg_.lane_heading_deg * kDegToRad;
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);

  // Rotate AOI corners into the lane frame (u = along-lane, v = across-lane)
  const std::array<std::pair<double, double>, 4> corners{{
    {cfg_.north_min, cfg_.east_min}, {cfg_.north_min, cfg_.east_max},
    {cfg_.north_max, cfg_.east_min}, {cfg_.north_max, cfg_.east_max}}};

  double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
  for (const auto & [n, e] : corners) {
    const double u = n * cos_h + e * sin_h;
    const double v = -n * sin_h + e * cos_h;
    u_min = std::min(u_min, u); u_max = std::max(u_max, u);
    v_min = std::min(v_min, v); v_max = std::max(v_max, v);
  }

  const int n_lanes = std::max(1, static_cast<int>(std::ceil((v_max - v_min) / lane_spacing_)) + 1);
  const float down = -static_cast<float>(cfg_.altitude_m);

  std::vector<Waypoint> out;
  out.reserve(n_lanes * 2);
  for (int i = 0; i < n_lanes; ++i) {
    const double v = std::min(v_min + i * lane_spacing_, v_max);
    const bool forward = (i % 2 == 0);
    const double u0 = forward ? u_min : u_max;
    const double u1 = forward ? u_max : u_min;
    const float yaw = static_cast<float>(forward ? heading : heading + M_PI);
    for (const double u : {u0, u1}) {
      const double n = u * cos_h - v * sin_h;
      const double e = u * sin_h + v * cos_h;
      out.push_back({static_cast<float>(n), static_cast<float>(e), down, yaw});
    }
  }
  return out;
}

}  // namespace sar_mission
