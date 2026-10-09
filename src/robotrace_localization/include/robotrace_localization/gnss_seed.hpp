#pragma once

// ROS-independent helpers. Coordinates are WGS84 ellipsoidal LLH and map ENU.
// A Seed's position is the REAR ANTENNA, not the IMU: the consumer applies
// the full 3-D lever arm after combining GNSS yaw with IMU roll/pitch.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace robotrace_localization {
constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kGpsWeekMs = 604800000;

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline bool finite(const Vec3& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

inline bool validLlh(double lat_deg, double lon_deg, double height_m) {
  return std::isfinite(lat_deg) && std::isfinite(lon_deg) &&
         std::isfinite(height_m) && std::abs(lat_deg) <= 90.0 &&
         std::abs(lon_deg) <= 180.0 && height_m > -10000.0 &&
         height_m < 100000.0;
}

inline Vec3 llhToEcef(double lat_deg, double lon_deg, double height_m) {
  if (!validLlh(lat_deg, lon_deg, height_m))
    throw std::invalid_argument("Invalid WGS84 latitude/longitude/height");
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double e2 = f * (2.0 - f);
  const double lat = lat_deg * kPi / 180.0;
  const double lon = lon_deg * kPi / 180.0;
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double n = a / std::sqrt(1.0 - e2 * sl * sl);
  return {(n + height_m) * cl * std::cos(lon),
          (n + height_m) * cl * std::sin(lon),
          (n * (1.0 - e2) + height_m) * sl};
}

class EnuOrigin {
 public:
  EnuOrigin(double lat_deg, double lon_deg, double height_m)
      : origin_(llhToEcef(lat_deg, lon_deg, height_m)) {
    const double lat = lat_deg * kPi / 180.0;
    const double lon = lon_deg * kPi / 180.0;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    rotation_ = {{-so, co, 0.0,
                  -sl * co, -sl * so, cl,
                  cl * co, cl * so, sl}};
  }

  Vec3 convert(double lat_deg, double lon_deg, double height_m) const {
    const Vec3 e = llhToEcef(lat_deg, lon_deg, height_m);
    const Vec3 d{e.x - origin_.x, e.y - origin_.y, e.z - origin_.z};
    return {rotation_[0] * d.x + rotation_[1] * d.y + rotation_[2] * d.z,
            rotation_[3] * d.x + rotation_[4] * d.y + rotation_[5] * d.z,
            rotation_[6] * d.x + rotation_[7] * d.y + rotation_[8] * d.z};
  }

 private:
  Vec3 origin_;
  std::array<double, 9> rotation_;
};

inline bool rtkFixed(std::uint8_t flags, std::uint8_t fix_type) {
  // GNSS fix OK, differential solution, carrier-phase FIXED. Other status
  // bits may also be set; comparing the entire flags byte to 131 is wrong.
  return fix_type == 3 && (flags & 0xC3) == 0x83;
}

inline double gpsTowDifferenceSeconds(std::uint32_t a, std::uint32_t b) {
  if (a >= kGpsWeekMs || b >= kGpsWeekMs)
    return INFINITY;
  const std::uint32_t raw = a > b ? a - b : b - a;
  return std::min(raw, kGpsWeekMs - raw) * 0.001;
}

// NavPVT has no std_msgs/Header. Derive its measurement epoch from the UTC
// calendar + signed nanosecond fraction. Leap-second samples are rejected
// instead of ambiguously mapping second 60 into POSIX time.
inline double utcUnixSeconds(int year, int month, int day, int hour,
                             int minute, int second, int nano,
                             std::uint8_t valid) {
  if ((valid & 0x07) != 0x07 || year < 1980 || year > 2099 ||
      month < 1 || month > 12 || hour < 0 || hour > 23 ||
      minute < 0 || minute > 59 || second < 0 || second > 59 ||
      nano < -1000000000 || nano > 1000000000)
    throw std::invalid_argument("NavPVT UTC fields are invalid/unavailable");
  const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
  const int month_days[12] = {31, leap ? 29 : 28, 31, 30, 31, 30,
                              31, 31, 30, 31, 30, 31};
  if (day < 1 || day > month_days[month - 1])
    throw std::invalid_argument("NavPVT UTC calendar date is invalid");
  std::int64_t days = 0;
  for (int y = 1970; y < year; ++y)
    days += 365 + (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
  for (int m = 1; m < month; ++m)
    days += month_days[m - 1];
  days += day - 1;
  return static_cast<double>(days * 86400 + hour * 3600 + minute * 60 +
                             second) + nano * 1e-9;
}

struct GnssSample {
  double stamp = 0.0;  // UTC measurement epoch, seconds.
  std::uint32_t tow_ms = 0;
  std::uint8_t flags = 0;
  std::uint8_t fix_type = 0;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double height_m = 0.0;
  double h_accuracy_m = 0.0;
  double v_accuracy_m = 0.0;
};

struct SeedConfig {
  double pair_tolerance_s = 0.010;
  double max_h_accuracy_m = 0.20;
  double max_v_accuracy_m = 0.50;
  double min_h_accuracy_m = 0.01;
  double min_v_accuracy_m = 0.02;
  double baseline_min_m = 0.80;
  double baseline_max_m = 2.20;
  double yaw_offset_rad = 0.0;
  double min_yaw_sigma_rad = kPi / 180.0;
  double max_yaw_sigma_rad = 20.0 * kPi / 180.0;
};

struct Seed {
  Vec3 rear_enu;
  double yaw_rad = 0.0;
  double stamp = 0.0;
  double baseline_m = 0.0;  // Horizontal baseline, not 3-D antenna distance.
  double pair_dt_s = 0.0;
  double h_variance_m2 = 0.0;
  double v_variance_m2 = 0.0;
  double yaw_variance_rad2 = 0.0;
};

inline bool sampleValid(const GnssSample& s, const SeedConfig& c,
                        std::string* reason = nullptr) {
  const char* error = nullptr;
  if (!rtkFixed(s.flags, s.fix_type)) error = "RTK FIX + 3D fix required";
  else if (!std::isfinite(s.stamp) || s.stamp <= 0.0)
    error = "invalid UTC measurement timestamp";
  else if (s.tow_ms >= kGpsWeekMs) error = "invalid GNSS iTOW";
  else if (!validLlh(s.lat_deg, s.lon_deg, s.height_m))
    error = "invalid GNSS latitude/longitude/height";
  else if (!std::isfinite(s.h_accuracy_m) || s.h_accuracy_m <= 0.0 ||
           s.h_accuracy_m > c.max_h_accuracy_m)
    error = "horizontal accuracy unavailable or above threshold";
  else if (!std::isfinite(s.v_accuracy_m) || s.v_accuracy_m <= 0.0 ||
           s.v_accuracy_m > c.max_v_accuracy_m)
    error = "vertical accuracy unavailable or above threshold";
  if (error && reason) *reason = error;
  return !error;
}

inline bool makeSeed(const GnssSample& rear, const GnssSample& front,
                     const EnuOrigin& origin, const SeedConfig& c, Seed* out,
                     std::string* reason = nullptr) {
  if (!out) throw std::invalid_argument("Seed output must not be null");
  if (!sampleValid(rear, c, reason) || !sampleValid(front, c, reason))
    return false;
  const double dt = std::abs(front.stamp - rear.stamp);
  if (dt > c.pair_tolerance_s + 1e-6 ||
      gpsTowDifferenceSeconds(front.tow_ms, rear.tow_ms) >
          c.pair_tolerance_s + 1e-6) {
    if (reason) *reason = "front/rear UTC or iTOW epoch mismatch";
    return false;
  }
  const Vec3 r = origin.convert(rear.lat_deg, rear.lon_deg, rear.height_m);
  const Vec3 f = origin.convert(front.lat_deg, front.lon_deg, front.height_m);
  const double baseline = std::hypot(f.x - r.x, f.y - r.y);
  if (!finite(r) || !finite(f) || !std::isfinite(baseline) ||
      baseline < c.baseline_min_m || baseline > c.baseline_max_m) {
    if (reason) *reason = "horizontal antenna baseline outside bounds";
    return false;
  }
  const double rh = std::max(rear.h_accuracy_m, c.min_h_accuracy_m);
  const double fh = std::max(front.h_accuracy_m, c.min_h_accuracy_m);
  const double rv = std::max(rear.v_accuracy_m, c.min_v_accuracy_m);
  // Conservative isotropic approximation; NavPVT does not expose a full
  // antenna-position covariance or cross-receiver error correlation.
  const double sigma_yaw = std::max(c.min_yaw_sigma_rad,
                                   std::hypot(rh, fh) / baseline);
  if (!std::isfinite(sigma_yaw) || sigma_yaw > c.max_yaw_sigma_rad) {
    if (reason) *reason = "yaw uncertainty exceeds threshold";
    return false;
  }
  const double yaw = std::atan2(f.y - r.y, f.x - r.x) + c.yaw_offset_rad;
  out->rear_enu = r;
  out->yaw_rad = std::atan2(std::sin(yaw), std::cos(yaw));
  out->stamp = rear.stamp;
  out->baseline_m = baseline;
  out->pair_dt_s = dt;
  out->h_variance_m2 = rh * rh;
  out->v_variance_m2 = rv * rv;
  out->yaw_variance_rad2 = sigma_yaw * sigma_yaw;
  return true;
}
}  // namespace robotrace_localization
