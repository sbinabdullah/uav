#pragma once

#include <array>
#include <vector>

namespace sar_mission
{

struct GridConfig
{
  // AOI bounds in PX4 local NED (metres, relative to takeoff point)
  double north_min;
  double north_max;
  double east_min;
  double east_max;

  double altitude_m;       // survey altitude above takeoff point (positive up)
  double hfov_deg;         // camera horizontal field of view
  double vfov_deg;         // camera vertical field of view
  double side_overlap;     // fraction 0..0.9 between adjacent lanes
  double lane_heading_deg; // lane direction, NED yaw convention (0=N, 90=E)
};

struct Waypoint
{
  float north;
  float east;
  float down;   // negative = up
  float yaw;    // NED yaw in radians
};

// Boustrophedon (lawnmower) coverage planner.
// Lane spacing uses the smaller of the two footprint axes so coverage is
// guaranteed regardless of camera roll about the nadir axis.
class CoveragePlanner
{
public:
  explicit CoveragePlanner(const GridConfig & cfg);

  std::vector<Waypoint> plan() const;

  double lane_spacing() const { return lane_spacing_; }

private:
  GridConfig cfg_;
  double lane_spacing_;
};

}  // namespace sar_mission
