// Copyright 2024 Universidad Politecnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file frame_conversions.hpp
 * @brief Pure coordinate-frame + unit translation for the AERPAW adapter.
 *
 * This header is the single home for AERPAW<->AS2 frame/unit conversion. It is
 * deliberately free of ROS, sockets and any robotics algorithm - it only maps
 * numbers between the two conventions the adapter joins:
 *
 *   AeroStack2 odom frame : ENU  (x=East, y=North, z=Up),   yaw in rad, CCW, 0=East.
 *   ArduPilot / aerpawlib : NED  (x=North, y=East, z=Down), heading in deg, CW, 0=North.
 *
 * Keeping this isolated lets it be unit-tested without a vehicle or ROS and stops
 * conversion math from leaking into the platform node or the rest of AeroStack2.
 *
 * @see docs/COORDINATE_FRAMES.md for the full per-system conventions (axes, units,
 * orientation/velocity/yaw) this module implements. All frame conversions MUST go
 * through these functions - never inline a transform elsewhere.
 */

#ifndef AS2_PLATFORM_AERPAW__FRAME_CONVERSIONS_HPP_
#define AS2_PLATFORM_AERPAW__FRAME_CONVERSIONS_HPP_

#include <cmath>

namespace aerpaw_platform
{
namespace frames
{

/// Metres per degree of latitude (WGS-84, good enough over a testbed area).
constexpr double kMetersPerDegreeLat = 111132.92;
/// Metres per degree of longitude at the equator; scale by cos(lat) for a local frame.
constexpr double kMetersPerDegreeLonEq = 111412.84;

/**
 * @brief A plain 3-vector. Component meaning depends on the frame it is used in
 * (ENU: x=E,y=N,z=U ; NED: x=N,y=E,z=D). Kept frame-agnostic on purpose.
 */
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Metres per degree of longitude at a given latitude (degrees).
inline double meters_per_degree_lon(double lat_deg)
{
  return kMetersPerDegreeLonEq * std::cos(lat_deg * M_PI / 180.0);
}

/**
 * @brief ENU vector -> NED vector. Pure axis permutation + Up->Down sign flip.
 *
 * Works for both positions and earth-frame (NED) velocities: no heading is
 * involved because the vector is already expressed in the earth-fixed frame.
 */
inline Vec3 enu_to_ned(const Vec3 & enu)
{
  return Vec3{enu.y, enu.x, -enu.z};  // north=+y_enu, east=+x_enu, down=-z_enu
}

/// NED vector -> ENU vector (inverse of enu_to_ned).
inline Vec3 ned_to_enu(const Vec3 & ned)
{
  return Vec3{ned.y, ned.x, -ned.z};  // east=+y_ned, north=+x_ned, up=-z_ned
}

/**
 * @brief Extract the ENU yaw (rad, CCW, 0=East) from a quaternion.
 *
 * Identical to tf2::getYaw / Matrix3x3::getRPY's yaw term, but kept header-only
 * so the adapter has no runtime dependency on tf2 and the math stays unit-testable.
 */
inline double quat_to_yaw_enu(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/**
 * @brief A unit quaternion (x,y,z,w). Header-only, so the adapter needs no tf2.
 *
 * AS2 attitude convention (matching the reference multirotor platform): world is
 * ENU, body "base_link" is FLU (forward/left/up), orientation is the world->body
 * expressed as the quaternion that rotates a body vector into the world frame
 * (i.e. q maps body axes onto world axes; R = quat_to_R(q)).
 */
struct Quat
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// Euler (roll,pitch,yaw about ENU/FLU axes, intrinsic ZYX) -> quaternion.
inline Quat enu_euler_to_quat(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  return Quat{
    sr * cp * cy - cr * sp * sy,   // x
    cr * sp * cy + sr * cp * sy,   // y
    cr * cp * sy - sr * sp * cy,   // z
    cr * cp * cy + sr * sp * sy};  // w
}

/**
 * @brief AERPAW attitude (MAVSDK NED/FRD euler, radians, yaw=heading from North
 * CW) -> AS2 ENU/FLU quaternion.
 *
 * Axis convention NED(FRD)->ENU(FLU): forward(+x) unchanged, right(+y)->left(-y),
 * down(+z)->up(-z), which yields roll_enu=roll_ned, pitch_enu=-pitch_ned and the
 * heading->ENU-yaw map used elsewhere. Verified by unit tests.
 */
inline Quat ned_euler_to_enu_quat(double roll_ned, double pitch_ned, double yaw_ned_rad)
{
  const double roll_enu = roll_ned;
  const double pitch_enu = -pitch_ned;
  const double yaw_enu = M_PI / 2.0 - yaw_ned_rad;
  return enu_euler_to_quat(roll_enu, pitch_enu, yaw_enu);
}

/// Rotation matrix columns from a unit quaternion (world<-body; R = q).
inline void quat_to_rot(const Quat & q, double R[9])
{
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  R[0] = 1 - 2 * (yy + zz); R[1] = 2 * (xy - wz);     R[2] = 2 * (xz + wy);
  R[3] = 2 * (xy + wz);     R[4] = 1 - 2 * (xx + zz); R[5] = 2 * (yz - wx);
  R[6] = 2 * (xz - wy);     R[7] = 2 * (yz + wx);     R[8] = 1 - 2 * (xx + yy);
}

/// Rotate a world (ENU) vector into the body (FLU) frame using attitude q.
inline Vec3 world_vec_to_body(const Quat & q, const Vec3 & v_world)
{
  double R[9];
  quat_to_rot(q, R);
  // body = R^T * world
  return Vec3{
    R[0] * v_world.x + R[3] * v_world.y + R[6] * v_world.z,
    R[1] * v_world.x + R[4] * v_world.y + R[7] * v_world.z,
    R[2] * v_world.x + R[5] * v_world.y + R[8] * v_world.z};
}

/**
 * @brief ENU yaw (rad, CCW, 0=East) -> ArduPilot heading (deg, CW, 0=North).
 * Result normalised to [0, 360).
 */
inline double enu_yaw_rad_to_heading_deg(double yaw_enu_rad)
{
  const double yaw_enu_deg = yaw_enu_rad * 180.0 / M_PI;
  double heading = std::fmod(90.0 - yaw_enu_deg + 360.0, 360.0);
  if (heading < 0.0) {
    heading += 360.0;
  }
  return heading;
}

/**
 * @brief ArduPilot heading (deg, CW, 0=North) -> ENU yaw (rad, CCW, 0=East).
 * Inverse of enu_yaw_rad_to_heading_deg.
 */
inline double heading_deg_to_enu_yaw_rad(double heading_deg)
{
  return (90.0 - heading_deg) * M_PI / 180.0;
}

/**
 * @brief GPS (lat/lon deg, local Up) -> flat-earth local ENU position about a home.
 *
 * @param lat_deg   Vehicle latitude (deg).
 * @param lon_deg   Vehicle longitude (deg).
 * @param up        Local up distance (m); caller supplies rel-alt, not MSL.
 * @param home_lat  Home latitude (deg) - the odom origin.
 * @param home_lon  Home longitude (deg) - the odom origin.
 * @return ENU vector (x=East, y=North, z=up) relative to home.
 */
inline Vec3 gps_to_local_enu(
  double lat_deg, double lon_deg, double up, double home_lat, double home_lon)
{
  const double east = (lon_deg - home_lon) * meters_per_degree_lon(home_lat);
  const double north = (lat_deg - home_lat) * kMetersPerDegreeLat;
  return Vec3{east, north, up};
}

}  // namespace frames
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__FRAME_CONVERSIONS_HPP_
