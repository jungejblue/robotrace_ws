#include <robotrace_localization/gnss_seed.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace robotrace_localization;

void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

template <typename Callable>
void rejects(Callable f, const char* message) {
  bool rejected = false;
  try { f(); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, message);
}

int main() {
  const EnuOrigin origin(36.729539, 127.4408323, 103.230);
  const Vec3 zero = origin.convert(36.729539, 127.4408323, 103.230);
  check(std::hypot(zero.x, zero.y) < 1e-8 && std::abs(zero.z) < 1e-8,
        "origin must map to ENU zero");
  const Vec3 above = origin.convert(36.729539, 127.4408323, 104.230);
  check(std::abs(above.x) < 1e-8 && std::abs(above.y) < 1e-8 &&
        std::abs(above.z - 1.0) < 1e-8, "ellipsoid height increases map U");
  check(origin.convert(36.729539, 127.4408423, 103.230).x > 0.8,
        "longitude increase must be east positive");
  check(origin.convert(36.729549, 127.4408323, 103.230).y > 1.0,
        "latitude increase must be north positive");
  const Vec3 equator = llhToEcef(0.0, 0.0, 0.0);
  check(std::abs(equator.x - 6378137.0) < 1e-8 &&
        std::abs(equator.y) < 1e-8 && std::abs(equator.z) < 1e-8,
        "known WGS84 equatorial ECEF point");
  rejects([] { EnuOrigin bad(91.0, 0.0, 0.0); }, "latitude range");
  rejects([] { llhToEcef(0.0, 0.0,
      std::numeric_limits<double>::quiet_NaN()); }, "NaN geodesy rejected");

  check(rtkFixed(131, 3), "flags 131 identifies valid 3D RTK FIX");
  check(rtkFixed(163, 3), "additional valid heading bit allowed");
  check(!rtkFixed(67, 3), "RTK FLOAT rejected");
  check(!rtkFixed(129, 3), "missing differential status rejected");
  check(!rtkFixed(131, 2) && !rtkFixed(131, 4), "only 3D GNSS fix accepted");
  check(std::abs(gpsTowDifferenceSeconds(kGpsWeekMs - 10, 10) - 0.02) < 1e-9,
        "iTOW GPS-week wrap handled");
  check(!std::isfinite(gpsTowDifferenceSeconds(kGpsWeekMs, 0)),
        "out-of-range iTOW rejected");
  const double epoch = utcUnixSeconds(2026, 5, 22, 8, 20, 4, -40000000, 7);
  check(std::abs(epoch - 1779438003.960) < 1e-6,
        "UTC calendar plus signed nano reproduces bag epoch");
  check(std::abs(utcUnixSeconds(2000, 2, 29, 0, 0, 0, 0, 7) -
                 951782400.0) < 1e-8, "leap-day calendar");
  rejects([] { utcUnixSeconds(2026, 2, 29, 0, 0, 0, 0, 7); },
          "invalid February date rejected");
  rejects([] { utcUnixSeconds(2026, 5, 22, 8, 20, 60, 0, 7); },
          "POSIX-ambiguous leap-second sample rejected");
  rejects([] { utcUnixSeconds(2026, 5, 22, 8, 20, 4, 0, 1); },
          "valid UTC time flag required");
  rejects([] { utcUnixSeconds(2026, 5, 22, 8, 20, 4, 0, 3); },
          "fully resolved UTC flag required");

  SeedConfig cfg;
  GnssSample rear;
  rear.stamp = epoch; rear.tow_ms = 100000; rear.flags = 131; rear.fix_type = 3;
  rear.lat_deg = 36.7294137; rear.lon_deg = 127.4408566;
  rear.height_m = 103.074; rear.h_accuracy_m = 0.015;
  rear.v_accuracy_m = 0.030;
  GnssSample front = rear;
  front.lat_deg = 36.7294252; front.lon_deg = 127.4408462;
  front.height_m = 102.443;
  Seed seed;
  std::string reason;
  check(makeSeed(rear, front, origin, cfg, &seed, &reason),
        "project's initial FIX positions produce a seed");
  check(std::abs(seed.rear_enu.x - 2.171) < 0.01 &&
        std::abs(seed.rear_enu.y + 13.905) < 0.01 &&
        std::abs(seed.rear_enu.z + 0.156) < 0.01,
        "seed remains REAR antenna ENU, no lever arm applied");
  check(std::abs(seed.yaw_rad * 180.0 / kPi - 126.05) < 0.1,
        "front-minus-rear yields expected ENU yaw");
  check(seed.h_variance_m2 > 0.0 && seed.v_variance_m2 > 0.0 &&
        seed.yaw_variance_rad2 >= cfg.min_yaw_sigma_rad * cfg.min_yaw_sigma_rad,
        "covariance floors prevent singular certainty");
  front.stamp += 0.08; front.tow_ms += 80;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "different GNSS epochs rejected");
  front.stamp = rear.stamp + 0.011; front.tow_ms = rear.tow_ms;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "UTC epoch gap above default 10 ms gate rejected even with equal iTOW");
  front.stamp = rear.stamp; front.tow_ms = rear.tow_ms + 11;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "iTOW gap above 10 ms gate rejected even with equal UTC");
  front = rear;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "coincident antennas cannot define yaw");
  front.lat_deg = 36.7294252; front.lon_deg = 127.4408462;
  front.h_accuracy_m = 0.0;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "unavailable GNSS accuracy must not imply zero uncertainty");
  front.h_accuracy_m = 0.015;
  front.flags = 67;
  check(!makeSeed(rear, front, origin, cfg, &seed, &reason),
        "front FLOAT cannot generate startup yaw");
  std::cout << "PASS: WGS84 ENU, RTK flags, GNSS UTC/iTOW, seed yaw and rejection checks\n";
  return 0;
}
