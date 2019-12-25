#pragma once

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>

#include "gnss_poser/gnss_stat.h"

namespace GNSSPoser {

enum class MGRSPrecision { 
  _10_KIRO_METER = 1,
  _1_KIRO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};

GNSSStat NavSatFix2UTM(const sensor_msgs::NavSatFix &nav_sat_fix_msg)
{
  GNSSStat utm;
  utm.coordinate_system = CoordinateSystem::UTM;

  try{
    GeographicLib::UTMUPS::Forward(nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm.zone, utm.northup, utm.x, utm.y);

    // GeographicLib::Geoid egm2008("egm2008-1");
    // utm.z = egm2008.ConvertHeight(nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude, GeographicLib::Geoid::ELLIPSOIDTOGEOID);
    utm.z = nav_sat_fix_msg.altitude;

    utm.latitude = nav_sat_fix_msg.latitude;
    utm.longitude = nav_sat_fix_msg.longitude;
    utm.altitude = nav_sat_fix_msg.altitude;
  }
  catch(const GeographicLib::GeographicErr err){
    ROS_ERROR_STREAM("Failed to convert from LLH to UTM" << err.what());
  }
  return utm;
}

GNSSStat UTM2MGRS(const GNSSStat &utm, const MGRSPrecision &precision)
{
  constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

  GNSSStat mgrs = utm;
  mgrs.coordinate_system = CoordinateSystem::MGRS;
  try {
    std::string mgrs_code;
    GeographicLib::MGRS::Forward(utm.zone, utm.northup, utm.x, utm.y, utm.latitude, static_cast<int>(precision), mgrs_code);
    mgrs.zone = std::stod(mgrs_code.substr(0, GZD_ID_size));
    mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) * std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) - static_cast<int>(precision));  // set unit as [m]
    mgrs.y = std::stod(mgrs_code.substr(GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) * std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) - static_cast<int>(precision)) ;  // set unit as [m]
    mgrs.z = utm.z;  // TODO
  }
  catch (const GeographicLib::GeographicErr err) {
    ROS_ERROR_STREAM("Failed to convert from UTM to MGRS" << err.what());
  }
  return mgrs;
}

GNSSStat NavSatFix2MGRS(const sensor_msgs::NavSatFix &nav_sat_fix_msg, const MGRSPrecision &precision)
{
  const auto utm = NavSatFix2UTM(nav_sat_fix_msg);
  const auto mgrs = UTM2MGRS(utm, precision);
  return mgrs;
}

}