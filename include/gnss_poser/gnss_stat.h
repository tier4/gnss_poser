#pragma once

namespace GNSSPoser {

enum class CoordinateSystem { 
  UTM = 0,
  MGRS = 1,
  PLANE = 2,
};

struct GNSSStat {
  GNSSStat()
    : coordinate_system(CoordinateSystem::MGRS)
    , northup(true)
    , zone(0)
    , x(0)
    , y(0)
    , z(0)
    , latitude(0)
    , longitude(0)
    , altitude(0)
  {
  }

  CoordinateSystem coordinate_system;
  bool northup;
  int zone;
  double x;
  double y;
  double z;
  double latitude;
  double longitude;
  double altitude;
};

}