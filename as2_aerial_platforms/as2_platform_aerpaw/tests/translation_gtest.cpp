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
 * @file translation_gtest.cpp
 * @brief Unit tests for the adapter's isolated translation modules.
 *
 * These need no vehicle, no ROS graph and no aerpawlib: they prove the
 * coordinate-frame, command and state translation is self-contained and correct.
 */

#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "as2_platform_aerpaw/adapter_types.hpp"
#include "as2_platform_aerpaw/command_capability.hpp"
#include "as2_platform_aerpaw/command_translation.hpp"
#include "as2_platform_aerpaw/frame_conversions.hpp"
#include "as2_platform_aerpaw/state_translation.hpp"
#include "as2_platform_aerpaw/telemetry_guard.hpp"
#include "as2_platform_aerpaw/vehicle_identity.hpp"

using aerpaw_platform::GeoHome;
using aerpaw_platform::PlatformBackend;
using aerpaw_platform::TelemetryState;
using aerpaw_platform::command::goto_enu;
using aerpaw_platform::command::speed_enu;
using aerpaw_platform::frames::enu_to_ned;
using aerpaw_platform::frames::enu_yaw_rad_to_heading_deg;
using aerpaw_platform::frames::gps_to_local_enu;
using aerpaw_platform::frames::heading_deg_to_enu_yaw_rad;
using aerpaw_platform::frames::ned_to_enu;
using aerpaw_platform::frames::quat_to_yaw_enu;
using aerpaw_platform::frames::Quat;
using aerpaw_platform::frames::Vec3;

TEST(FrameConversions, EnuNedRoundTrip)
{
  const Vec3 enu{1.0, 2.0, 3.0};
  const Vec3 ned = enu_to_ned(enu);
  EXPECT_DOUBLE_EQ(ned.x, 2.0);   // north = +y_enu
  EXPECT_DOUBLE_EQ(ned.y, 1.0);   // east  = +x_enu
  EXPECT_DOUBLE_EQ(ned.z, -3.0);  // down  = -z_enu
  const Vec3 back = ned_to_enu(ned);
  EXPECT_DOUBLE_EQ(back.x, 1.0);
  EXPECT_DOUBLE_EQ(back.y, 2.0);
  EXPECT_DOUBLE_EQ(back.z, 3.0);
}

TEST(FrameConversions, HeadingYawConversions)
{
  EXPECT_NEAR(enu_yaw_rad_to_heading_deg(0.0), 90.0, 1e-9);        // East -> 90
  EXPECT_NEAR(enu_yaw_rad_to_heading_deg(M_PI / 2.0), 0.0, 1e-9);  // North -> 0
  const double y = 1.234;
  EXPECT_NEAR(heading_deg_to_enu_yaw_rad(enu_yaw_rad_to_heading_deg(y)), y, 1e-9);
}

TEST(FrameConversions, QuatYawMatchesHeading)
{
  // Yaw-only quaternion about +z (ENU). yaw=90deg => East heading 0? build q.
  const double yaw = M_PI / 2.0;
  const double z = std::sin(yaw / 2.0), w = std::cos(yaw / 2.0);
  EXPECT_NEAR(quat_to_yaw_enu(0.0, 0.0, z, w), yaw, 1e-9);
  // heading for that yaw should be 0 (North).
  EXPECT_NEAR(enu_yaw_rad_to_heading_deg(quat_to_yaw_enu(0.0, 0.0, z, w)), 0.0, 1e-9);
}

TEST(FrameConversions, GpsToLocalEnu)
{
  const Vec3 p = gps_to_local_enu(35.0 + 0.001, -78.0, 10.0, 35.0, -78.0);
  EXPECT_GT(p.y, 0.0);                       // north positive
  EXPECT_NEAR(p.z, 10.0, 1e-9);              // up passthrough
  EXPECT_NEAR(p.x, 0.0, 1e-9);               // same longitude -> east 0
}

TEST(FrameConversions, NedAttitudeToEnuQuat)
{
  // Level, heading North -> ENU yaw 90deg; body +x maps to world +y (North).
  const Quat qn = aerpaw_platform::frames::ned_euler_to_enu_quat(0.0, 0.0, 0.0);
  EXPECT_NEAR(quat_to_yaw_enu(qn.x, qn.y, qn.z, qn.w), M_PI / 2.0, 1e-9);
  double m[9];
  aerpaw_platform::frames::quat_to_rot(qn, m);
  EXPECT_NEAR(m[0], 0.0, 1e-9);   // body x (forward) has 0 world-east component
  EXPECT_NEAR(m[3], 1.0, 1e-9);   // body x -> world north

  // Heading East (yaw_ned=90) -> ENU yaw 0 -> body +x -> world +x (East).
  const Quat qe = aerpaw_platform::frames::ned_euler_to_enu_quat(0.0, 0.0, M_PI / 2.0);
  EXPECT_NEAR(quat_to_yaw_enu(qe.x, qe.y, qe.z, qe.w), 0.0, 1e-9);
}

TEST(FrameConversions, WorldVecToBodyPreservesMagnitude)
{
  const Quat q = aerpaw_platform::frames::ned_euler_to_enu_quat(0.2, -0.1, 1.0);
  const Vec3 w{3.0, 4.0, 5.0};
  const Vec3 b = aerpaw_platform::frames::world_vec_to_body(q, w);
  EXPECT_NEAR(std::hypot(std::hypot(b.x, b.y), b.z), std::hypot(std::hypot(w.x, w.y), w.z), 1e-9);
}

TEST(CommandTranslation, SpeedNedMapping)
{
  const auto j = speed_enu(Vec3{3.0, 4.0, 5.0});  // ENU (E,N,U)
  EXPECT_EQ(j["cmd"], "set_velocity");
  EXPECT_DOUBLE_EQ(j["vx"].get<double>(), 4.0);   // north
  EXPECT_DOUBLE_EQ(j["vy"].get<double>(), 3.0);   // east
  EXPECT_DOUBLE_EQ(j["vz"].get<double>(), -5.0);  // down
}

TEST(CommandTranslation, GotoCarriesHomeAndHeading)
{
  GeoHome home;
  home.lat = 35.727;
  home.lon = -78.72;
  home.set = true;
  const auto j = goto_enu(Vec3{10.0, 20.0, 25.0}, 0.0, home);
  EXPECT_EQ(j["cmd"], "goto_ned");
  EXPECT_DOUBLE_EQ(j["north"].get<double>(), 20.0);
  EXPECT_DOUBLE_EQ(j["east"].get<double>(), 10.0);
  EXPECT_DOUBLE_EQ(j["down"].get<double>(), -25.0);
  EXPECT_DOUBLE_EQ(j["heading"].get<double>(), 90.0);
  EXPECT_DOUBLE_EQ(j["home_lat"].get<double>(), 35.727);
  EXPECT_DOUBLE_EQ(j["home_lon"].get<double>(), -78.72);
}

TEST(StateTranslation, OdometryIsEnuAboutHome)
{
  GeoHome home;
  home.lat = 35.0;
  home.lon = -78.0;
  home.set = true;
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.lat = 35.0;
  tel.lon = -78.0;
  tel.rel_alt = 25.0;
  tel.vx_ned = 1.0;   // north
  tel.vy_ned = 2.0;   // east
  tel.vz_ned = -0.5;  // down
  const auto odom = aerpaw_platform::state::makeOdometry(tel, home, "earth", "base_link");
  EXPECT_NEAR(odom.pose.pose.position.x, 0.0, 1e-9);   // east
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, 1e-9);   // north
  EXPECT_NEAR(odom.pose.pose.position.z, 25.0, 1e-9);  // up
  EXPECT_EQ(odom.header.frame_id, "earth");
  EXPECT_EQ(odom.child_frame_id, "base_link");
}

TEST(StateTranslation, OdometryOrientationFromAttitudeNotIdentity)
{
  GeoHome home;
  home.set = true;
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.yaw_ned_rad = 0.0;  // heading North -> ENU yaw 90deg
  const auto odom = aerpaw_platform::state::makeOdometry(tel, home, "odom", "base_link");
  // orientation must NOT be left at identity (0,0,0,1); yaw=90 -> qz=sin(45).
  EXPECT_NEAR(odom.pose.pose.orientation.w, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.z, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.x, 0.0, 1e-9);
}

TEST(StateTranslation, OdometryTwistIsBodyFrameAndRatesMarkedUnknown)
{
  GeoHome home;
  home.set = true;
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.yaw_ned_rad = 0.0;              // facing North
  tel.vx_ned = 1.0;                   // north
  tel.vy_ned = 2.0;                   // east
  const auto odom = aerpaw_platform::state::makeOdometry(tel, home, "odom", "base_link");
  // facing North: forward(body x)=north=1 ; body y=left=west so east(+) -> -2
  EXPECT_NEAR(odom.twist.twist.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(odom.twist.twist.linear.y, -2.0, 1e-9);
  // angular velocity NOT available -> zero value + unknown covariance marker
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(odom.twist.covariance[15], -1.0);
}

TEST(StateTranslation, ImuOrientationPresentRatesAndAccelMarkedUnavailable)
{
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.roll_rad = 0.0;
  tel.pitch_rad = 0.0;
  tel.yaw_ned_rad = M_PI / 2.0;  // heading East -> ENU yaw 0
  const auto imu = aerpaw_platform::state::makeImu(tel, "base_link");
  // orientation available (identity-ish for yaw=0 level)
  EXPECT_NEAR(imu.orientation.w, 1.0, 1e-6);
  // rates + accel are NOT fabricated: value 0 AND covariance -1 (unknown)
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(imu.angular_velocity_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 0.0);   // no fake 9.81
  EXPECT_DOUBLE_EQ(imu.linear_acceleration_covariance[0], -1.0);
}

TEST(StateTranslation, GpsCarriesRealFixQuality)
{
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.lat = 35.727;
  tel.lon = -78.72;
  tel.alt_msl = 145.0;
  tel.gps_fix_type = 3;
  const auto gps = aerpaw_platform::state::makeGps(tel, "earth");
  EXPECT_DOUBLE_EQ(gps.latitude, 35.727);
  EXPECT_DOUBLE_EQ(gps.altitude, 145.0);
  EXPECT_EQ(gps.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  tel.gps_fix_type = 0;
  const auto gps_nofix = aerpaw_platform::state::makeGps(tel, "earth");
  EXPECT_EQ(gps_nofix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST(StateTranslation, BatteryFromAerpaw)
{
  TelemetryState tel;
  tel.kind = aerpaw_platform::InboundKind::TELEMETRY;
  tel.battery_valid = true;
  tel.battery_voltage = 22.4;
  tel.battery_current = 5.0;
  tel.battery_level = 87.0;  // 0-100
  const auto bat = aerpaw_platform::state::makeBattery(tel);
  EXPECT_NEAR(bat.voltage, 22.4f, 1e-4);
  EXPECT_NEAR(bat.percentage, 0.87f, 1e-4);   // AS2 wants 0..1
  EXPECT_TRUE(bat.present);
  EXPECT_EQ(bat.power_supply_status, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
}

TEST(VehicleIdentity, PortAndNamespaceMapping)
{
  EXPECT_EQ(aerpaw_platform::identity::cmd_port_for(0), 15760);
  EXPECT_EQ(aerpaw_platform::identity::cmd_port_for(1), 15762);
  EXPECT_EQ(aerpaw_platform::identity::tel_port_for(2), 15765);
  EXPECT_EQ(aerpaw_platform::identity::default_namespace(3), "drone3");
}

// Stage 4: one AERPAW UAV -> one AS2 platform, N of them coexisting uniquely.
TEST(VehicleMap, FleetOfThreeIsUnique)
{
  const auto map = aerpaw_platform::identity::build_vehicle_map(3);
  ASSERT_EQ(map.size(), 3u);
  EXPECT_EQ(map[0].ns, "drone0");
  EXPECT_EQ(map[2].ns, "drone2");
  EXPECT_EQ(map[1].cmd_port, 15762);
  EXPECT_EQ(map[1].tel_port, 15763);
  EXPECT_EQ(map[0].vehicle_id, "drone0");     // unique id by default
  EXPECT_FALSE(map[0].aerpaw_conn.empty());   // unique endpoint per vehicle
  EXPECT_TRUE(aerpaw_platform::identity::validate_unique(map).empty());
}

TEST(VehicleMap, ExplicitIdsAndConnsOverrideDefaults)
{
  const auto map = aerpaw_platform::identity::build_vehicle_map(
    2, {"uav-north", "uav-south"}, {"udpin://10.14.1.11:14550", "udpin://10.14.1.12:14550"},
    aerpaw_platform::PlatformBackend::DIGITAL_TWIN);
  EXPECT_EQ(map[0].vehicle_id, "uav-north");
  EXPECT_EQ(map[1].vehicle_id, "uav-south");
  EXPECT_EQ(map[0].aerpaw_conn, "udpin://10.14.1.11:14550");
  EXPECT_EQ(map[0].backend, aerpaw_platform::PlatformBackend::DIGITAL_TWIN);
  EXPECT_TRUE(aerpaw_platform::identity::validate_unique(map).empty());
}

TEST(VehicleMap, DetectsDuplicateIdCollision)
{
  // Two vehicles given the same id must be flagged (would otherwise be ambiguous).
  auto map = aerpaw_platform::identity::build_vehicle_map(2);
  map[1].vehicle_id = map[0].vehicle_id;
  EXPECT_FALSE(aerpaw_platform::identity::validate_unique(map).empty());
}

TEST(Backend, StringParsing)
{
  EXPECT_EQ(aerpaw_platform::backend_from_string("digital_twin"), PlatformBackend::DIGITAL_TWIN);
  EXPECT_EQ(aerpaw_platform::backend_from_string("physical"), PlatformBackend::PHYSICAL);
  EXPECT_EQ(aerpaw_platform::backend_from_string("sitl"), PlatformBackend::SITL);
  EXPECT_TRUE(aerpaw_platform::is_real_hardware(PlatformBackend::PHYSICAL));
  EXPECT_FALSE(aerpaw_platform::is_real_hardware(PlatformBackend::DIGITAL_TWIN));
}

namespace cap = aerpaw_platform::capability;
using as2_msgs::msg::ControlMode;

TEST(CommandCapability, LifecycleAndPositionAlwaysSupported)
{
  for (auto b : {PlatformBackend::SITL, PlatformBackend::DIGITAL_TWIN, PlatformBackend::PHYSICAL}) {
    EXPECT_TRUE(cap::is_supported(cap::arm(b)));
    EXPECT_TRUE(cap::is_supported(cap::takeoff(b)));
    EXPECT_TRUE(cap::is_supported(cap::land(b)));
    EXPECT_TRUE(cap::is_supported(cap::position(b)));
    EXPECT_TRUE(cap::is_supported(cap::hover(b)));
  }
}

TEST(CommandCapability, VelocityBlockedOnTestbedOnly)
{
  EXPECT_EQ(cap::speed(PlatformBackend::SITL), cap::Support::SUPPORTED);
  EXPECT_EQ(cap::speed(PlatformBackend::DIGITAL_TWIN), cap::Support::UNSUPPORTED_ON_TESTBED);
  EXPECT_EQ(cap::speed(PlatformBackend::PHYSICAL), cap::Support::UNSUPPORTED_ON_TESTBED);
}

TEST(CommandCapability, OffboardClassesUnsupportedAlways)
{
  for (auto b : {PlatformBackend::SITL, PlatformBackend::DIGITAL_TWIN, PlatformBackend::PHYSICAL}) {
    EXPECT_EQ(cap::trajectory(b), cap::Support::UNSUPPORTED_ALWAYS);
    EXPECT_EQ(cap::attitude(b), cap::Support::UNSUPPORTED_ALWAYS);
    EXPECT_EQ(cap::body_rates(b), cap::Support::UNSUPPORTED_ALWAYS);
    EXPECT_EQ(cap::yaw_speed(b), cap::Support::UNSUPPORTED_ALWAYS);
  }
}

TEST(CommandCapability, ForControlModeMatrix)
{
  EXPECT_TRUE(cap::is_supported(
    cap::for_control_mode(PlatformBackend::PHYSICAL, ControlMode::POSITION, ControlMode::YAW_ANGLE)));
  EXPECT_TRUE(cap::is_supported(
    cap::for_control_mode(PlatformBackend::SITL, ControlMode::SPEED, ControlMode::YAW_ANGLE)));
  EXPECT_FALSE(cap::is_supported(
    cap::for_control_mode(PlatformBackend::PHYSICAL, ControlMode::SPEED, ControlMode::YAW_ANGLE)));
  EXPECT_FALSE(cap::is_supported(
    cap::for_control_mode(PlatformBackend::SITL, ControlMode::POSITION, ControlMode::YAW_SPEED)));
  EXPECT_FALSE(cap::is_supported(
    cap::for_control_mode(PlatformBackend::SITL, ControlMode::TRAJECTORY, ControlMode::YAW_ANGLE)));
}

// Stage 13: Digital Twin and Physical Testbed are the SAME AERPAW environment; the
// robotics capability matrix must be identical for them (only SITL/dev env differs).
TEST(CommandCapability, DigitalTwinEqualsPhysicalTestbed)
{
  EXPECT_TRUE(aerpaw_platform::is_aerpaw_environment(PlatformBackend::DIGITAL_TWIN));
  EXPECT_TRUE(aerpaw_platform::is_aerpaw_environment(PlatformBackend::PHYSICAL));
  EXPECT_FALSE(aerpaw_platform::is_aerpaw_environment(PlatformBackend::SITL));
  // The ONLY DT-vs-physical difference is the safety nuance, not a capability:
  EXPECT_FALSE(aerpaw_platform::is_real_hardware(PlatformBackend::DIGITAL_TWIN));
  EXPECT_TRUE(aerpaw_platform::is_real_hardware(PlatformBackend::PHYSICAL));

  const int modes[] = {ControlMode::HOVER, ControlMode::POSITION, ControlMode::SPEED,
    ControlMode::SPEED_IN_A_PLANE, ControlMode::ATTITUDE, ControlMode::BODY_RATES,
    ControlMode::TRAJECTORY};
  const int yaws[] = {ControlMode::YAW_ANGLE, ControlMode::YAW_SPEED};
  for (int m : modes) {
    for (int y : yaws) {
      EXPECT_EQ(
        static_cast<int>(cap::for_control_mode(PlatformBackend::DIGITAL_TWIN, m, y)),
        static_cast<int>(cap::for_control_mode(PlatformBackend::PHYSICAL, m, y)))
        << "capability differs between DT and physical for mode=" << m << " yaw=" << y;
    }
  }
}

TEST(CommandTranslation, HoverAtRepositionForm)
{
  GeoHome home;
  home.lat = 35.727;
  home.lon = -78.72;
  home.set = true;
  const auto j = goto_enu(Vec3{5.0, 6.0, 25.0}, 0.0, home);
  // hover_at == a goto_ned reposition to current pose (supported primitive)
  EXPECT_EQ(j["cmd"], "goto_ned");
  EXPECT_DOUBLE_EQ(j["north"].get<double>(), 6.0);
  EXPECT_DOUBLE_EQ(j["east"].get<double>(), 5.0);
  EXPECT_DOUBLE_EQ(j["down"].get<double>(), -25.0);
}

// Stage 11: online position-reference updates -> send only when it changed.
TEST(CommandTranslation, PositionReferenceChangedGate)
{
  auto changed = aerpaw_platform::command::position_reference_changed;
  const Vec3 a{0.0, 0.0, 25.0};
  // identical reference -> no resend
  EXPECT_FALSE(changed(a, a, 0.0, 0.0, 0.5, 0.1));
  // tiny move (< eps) -> no resend
  EXPECT_FALSE(changed(a, Vec3{0.3, 0.0, 25.0}, 0.0, 0.0, 0.5, 0.1));
  // real move (> eps) -> resend
  EXPECT_TRUE(changed(a, Vec3{2.0, 0.0, 25.0}, 0.0, 0.0, 0.5, 0.1));
  // alt change counts
  EXPECT_TRUE(changed(a, Vec3{0.0, 0.0, 30.0}, 0.0, 0.0, 0.5, 0.1));
  // yaw change beyond eps -> resend
  EXPECT_TRUE(changed(a, a, 0.0, 0.5, 0.5, 0.1));
  // yaw wrap: 359deg ~ 0deg is a small change -> no resend
  const double near = 3.13;   // ~179.3deg
  const double far = -3.13;   // ~-179.3deg  (true diff ~ 0.02 rad, not ~6.26)
  EXPECT_FALSE(changed(a, a, near, far, 0.5, 0.1));
}

// Stage 7: the two directions use the SAME permutation, so AS2->AERPAW and
// AERPAW->AS2 are exact inverses (no hidden extra transform).
TEST(FrameRoundTrip, CommandAndStateAreInverses)
{
  GeoHome home;
  home.lat = 35.0;
  home.lon = -78.0;
  home.set = true;
  const Vec3 as2_enu{3.0, 4.0, 25.0};  // E, N, U (odom)
  const auto cmd = goto_enu(as2_enu, 0.0, home);
  // Rebuild the same ENU vector from the wire NED values using the state-side map.
  const Vec3 back = ned_to_enu(
    Vec3{cmd["north"].get<double>(), cmd["east"].get<double>(), cmd["down"].get<double>()});
  EXPECT_NEAR(back.x, as2_enu.x, 1e-9);
  EXPECT_NEAR(back.y, as2_enu.y, 1e-9);
  EXPECT_NEAR(back.z, as2_enu.z, 1e-9);
}

TEST(FrameRoundTrip, YawHeadingIsInverse)
{
  // AS2 yaw (ENU) -> AERPAW heading (wire) and back.
  for (double y : {-3.0, -0.5, 0.0, 0.7, M_PI, 2.5}) {
    const double heading_deg = enu_yaw_rad_to_heading_deg(y);
    const double y2 = heading_deg_to_enu_yaw_rad(heading_deg);
    // normalise to (-pi, pi]
    double d = std::fmod(y2 - y + M_PI, 2 * M_PI);
    if (d < 0) {
      d += 2 * M_PI;
    }
    EXPECT_NEAR(d - M_PI, 0.0, 1e-9);
  }
}

// Stage 17: input guards keep malformed / stale data from propagating.
using aerpaw_platform::guard::command_is_valid;
using aerpaw_platform::guard::telemetry_is_fresh;
using aerpaw_platform::guard::telemetry_is_valid;

TEST(TelemetryGuard, RejectsInvalidSamples)
{
  TelemetryState t;
  t.kind = aerpaw_platform::InboundKind::TELEMETRY;
  t.lat = 35.727; t.lon = -78.698; t.alt_msl = 85.0; t.rel_alt = 25.0;
  EXPECT_TRUE(telemetry_is_valid(t));
  // NaN position -> invalid
  TelemetryState bad = t; bad.lat = std::nan("");
  EXPECT_FALSE(telemetry_is_valid(bad));
  // out-of-range lat -> invalid
  TelemetryState oor = t; oor.lat = 120.0;
  EXPECT_FALSE(telemetry_is_valid(oor));
  // (0,0) no-fix sentinel -> invalid
  TelemetryState zero = t; zero.lat = 0.0; zero.lon = 0.0;
  EXPECT_FALSE(telemetry_is_valid(zero));
  // NaN velocity -> invalid
  TelemetryState nanv = t; nanv.vy_ned = std::nan("");
  EXPECT_FALSE(telemetry_is_valid(nanv));
}

TEST(TelemetryGuard, RejectsStaleOrFrozen)
{
  // fresh: recent age + advancing ts
  EXPECT_TRUE(telemetry_is_fresh(0.1, 1.0, 100.0, 99.0));
  // stale by age
  EXPECT_FALSE(telemetry_is_fresh(2.0, 1.0, 100.0, 99.0));
  // frozen: same ts as last accepted
  EXPECT_FALSE(telemetry_is_fresh(0.1, 1.0, 100.0, 100.0));
}

TEST(TelemetryGuard, CommandValidity)
{
  EXPECT_TRUE(command_is_valid(Vec3{1.0, 2.0, 3.0}, 0.5));
  Vec3 nanpos{std::nan(""), 0.0, 0.0};
  EXPECT_FALSE(command_is_valid(nanpos, 0.0));
  EXPECT_FALSE(command_is_valid(Vec3{0.0, 0.0, 0.0}, std::nan("")));
}
