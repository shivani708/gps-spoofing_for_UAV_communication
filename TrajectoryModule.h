#pragma once
// =============================================================================
// TrajectoryModule.h  — FIXED VERSION
// GPS L1 Real-Time Spoofer — Trajectory & Signal Parameter Calculator
// =============================================================================
//
// FIXES APPLIED (see TrajectoryModule.cpp for implementation details):
//
//   FIX-TM-1 (CRITICAL): Wrong code phase formula in calculate_target().
//     Old: fmod((flight_time - clock_corr) * chip_rate, 1023)
//     New: fmod((gps_tow  - flight_time + clock_corr) * chip_rate, 1023)
//     Root cause: GPS code phase is the fractional chip count of the TRANSMIT
//     time (gps_tow - pseudorange/c), not of the propagation time alone.
//     Without this fix the receiver finds the acquisition peak but computes
//     a completely wrong pseudorange — NEVER 3D fixes.
//
//   FIX-TM-2 (HIGH): Added IS-GPS-200H §20.3.3.5.2.5 Klobuchar ionospheric
//     delay model (klobuchar_delay() in CoordinateMath namespace).
//     The receiver subtracts a Klobuchar correction derived from the
//     broadcasted alpha/beta coefficients; if the spoofer does not embed
//     the corresponding iono delay in the pseudorange, each SV pseudorange
//     is SHORT by 5-15 m (elevation-dependent), producing a biased 3D
//     position.  IonoModel is passed optionally to calculate_target().
//
//   FIX-TM-3: Tropospheric delay is NOT embedded (matches gpssim.c computeRange).
//     Receivers apply their own troposphere model; only iono is in pseudorange.
//
// Standards:
//   IS-GPS-200H  — Interface Specification, GPS Navstar
//   RINEX 3.03   — Navigation message format
//   WGS-84       — World Geodetic System 1984
// =============================================================================

#ifndef TRAJECTORY_MODULE_H
#define TRAJECTORY_MODULE_H

#include <cmath>
#include <array>
#include <string>
#include <stdexcept>

// =============================================================================
// SECTION 1: CONSTANTS
// =============================================================================

namespace GpsConst {

    // WGS-84 Ellipsoid parameters
    constexpr double a        = 6'378'137.0;              // Semi-major axis [m]
    constexpr double f        = 1.0 / 298.257'223'563;    // Flattening
    constexpr double b        = a * (1.0 - f);            // Semi-minor axis [m]
    constexpr double e2       = 2.0*f - f*f;              // First eccentricity squared
    constexpr double ep2      = e2 / (1.0 - e2);          // Second eccentricity squared

    // Earth physical constants (IS-GPS-200H Table 20-IV)
    constexpr double mu       = 3.986'005e14;             // Gravitational parameter [m^3/s^2]
    constexpr double omega_e  = 7.292'115'1467e-5;        // Earth rotation rate [rad/s]

    // Signal constants
    constexpr double c        = 299'792'458.0;            // Speed of light [m/s]
    constexpr double f_L1     = 1'575'420'000.0;          // L1 carrier frequency [Hz]
    constexpr double chip_rate = 1'023'000.0;             // C/A chip rate [chips/s]
    constexpr double lambda_L1 = c / f_L1;               // L1 wavelength [m]

    constexpr double PI       = 3.141'592'653'589'793;
    constexpr double TWO_PI   = 2.0 * PI;

    constexpr int    MAX_ITER = 10;
    constexpr double ITER_TOL = 1e-12;

} // namespace GpsConst


// =============================================================================
// SECTION 2: STRUCTS
// =============================================================================

struct WGS84 {
    double lat_deg;   // Geodetic latitude  [degrees, +North]
    double lon_deg;   // Geodetic longitude [degrees, +East]
    double alt_m;     // Ellipsoidal height [metres]
};

struct ECEF {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ECEFVel {
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
};

struct GpsTime {
    int    week;
    double tow;
};

struct Ephemeris {
    int    sv_id;
    int    week;
    double toc;
    double toe;

    double af0;
    double af1;
    double af2;

    double sqrt_A;
    double e;
    double i0;
    double omega0;
    double omega;
    double M0;

    double delta_n;
    double idot;
    double omega_dot;

    double Cuc;
    double Cus;
    double Crc;
    double Crs;
    double Cic;
    double Cis;

    double Tgd;

    int    IODC;
    int    IODE;
};

// ── FIX-TM-2: Ionospheric model parameters ───────────────────────────────────
// Klobuchar 8-coefficient model (IS-GPS-200H §20.3.3.5.2.5, Subframe 4 Page 18).
// Fill this from the RINEX header alpha/beta lines and pass to calculate_target()
// so the synthesized pseudoranges include the same iono delay that the receiver
// will subtract using our broadcasted SF4 Page 18 coefficients.
struct IonoModel {
    double alpha[4] = {4.6566e-9, 1.4901e-8, -5.9605e-8, -5.9605e-8}; // [s, s/sc, ...]
    double beta[4]  = {90112.0,   98304.0,   -65536.0,   -458752.0};  // [s, s/sc, ...]
    bool   valid    = false;  // false → use 5 ns nominal delay only
};

struct TargetState {
    int    sv_id;

    ECEF   sat_pos;
    ECEFVel sat_vel;

    double range_m;
    double doppler_hz;
    double code_phase_chips;  // FIX-TM-1: now = fmod((tow - pseudorange/c)*chip_rate, 1023)
    double flight_time_s;
    double clock_corr_s;

    double elevation_deg;
    double azimuth_deg;
};


// =============================================================================
// SECTION 3: CoordinateMath NAMESPACE
// =============================================================================

namespace CoordinateMath {

    inline ECEF lla_to_ecef(const WGS84& lla)
    {
        const double lat = lla.lat_deg * GpsConst::PI / 180.0;
        const double lon = lla.lon_deg * GpsConst::PI / 180.0;
        const double alt = lla.alt_m;
        const double N = GpsConst::a /
                         std::sqrt(1.0 - GpsConst::e2 * std::sin(lat)*std::sin(lat));
        ECEF ecef;
        ecef.x = (N + alt) * std::cos(lat) * std::cos(lon);
        ecef.y = (N + alt) * std::cos(lat) * std::sin(lon);
        ecef.z = (N * (1.0 - GpsConst::e2) + alt) * std::sin(lat);
        return ecef;
    }

    inline WGS84 ecef_to_lla(const ECEF& ecef)
    {
        const double p   = std::sqrt(ecef.x*ecef.x + ecef.y*ecef.y);
        const double lon = std::atan2(ecef.y, ecef.x);
        double lat = std::atan2(ecef.z, p * (1.0 - GpsConst::e2));
        for (int i = 0; i < 10; ++i) {
            const double N   = GpsConst::a /
                               std::sqrt(1.0 - GpsConst::e2*std::sin(lat)*std::sin(lat));
            const double lat_new = std::atan2(ecef.z + GpsConst::e2 * N * std::sin(lat), p);
            if (std::abs(lat_new - lat) < 1e-12) { lat = lat_new; break; }
            lat = lat_new;
        }
        const double N   = GpsConst::a /
                           std::sqrt(1.0 - GpsConst::e2*std::sin(lat)*std::sin(lat));
        const double alt = p / std::cos(lat) - N;
        return WGS84{lat * 180.0/GpsConst::PI,
                     lon * 180.0/GpsConst::PI,
                     alt};
    }

    inline double ecef_distance(const ECEF& a, const ECEF& b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    inline double dot(const ECEF& a, const ECEF& b)
    {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }

    // Sagnac / Earth-rotation correction: rotate sat ECEF by omega_e * tau
    // Reference: IS-GPS-200H §20.3.3.4.3.3
    inline ECEF apply_sagnac(const ECEF& sat_pos_tx, double tau)
    {
        const double dOmega = GpsConst::omega_e * tau;
        const double cos_dO = std::cos(dOmega);
        const double sin_dO = std::sin(dOmega);
        ECEF corrected;
        corrected.x =  cos_dO * sat_pos_tx.x + sin_dO * sat_pos_tx.y;
        corrected.y = -sin_dO * sat_pos_tx.x + cos_dO * sat_pos_tx.y;
        corrected.z =  sat_pos_tx.z;
        return corrected;
    }

    inline void elevation_azimuth(const ECEF& user, const ECEF& sat,
                                   double& el_deg, double& az_deg)
    {
        const double dx = sat.x - user.x;
        const double dy = sat.y - user.y;
        const double dz = sat.z - user.z;
        const WGS84 lla = ecef_to_lla(user);
        const double lat = lla.lat_deg * GpsConst::PI / 180.0;
        const double lon = lla.lon_deg * GpsConst::PI / 180.0;
        const double sl  = std::sin(lat), cl = std::cos(lat);
        const double sl2 = std::sin(lon), cl2 = std::cos(lon);
        const double e_comp =  -sl2*dx         + cl2*dy;
        const double n_comp =  -sl*cl2*dx - sl*sl2*dy + cl*dz;
        const double u_comp =   cl*cl2*dx + cl*sl2*dy + sl*dz;
        const double horiz = std::sqrt(e_comp*e_comp + n_comp*n_comp);
        el_deg = std::atan2(u_comp, horiz) * 180.0 / GpsConst::PI;
        az_deg = std::fmod(std::atan2(e_comp, n_comp) * 180.0 / GpsConst::PI + 360.0, 360.0);
    }

    // ── FIX-TM-2: Klobuchar ionospheric delay model ──────────────────────────
    // IS-GPS-200H §20.3.3.5.2.5 — Identical algorithm to gpssim.c
    // ionosphericDelay().
    //
    // Returns the one-way L1 ionospheric delay in METRES.
    //
    // Parameters:
    //   iono     — alpha[4] and beta[4] from Subframe 4 Page 18
    //   gps_tow  — GPS time of week at reception [s]
    //   el_deg   — satellite elevation [degrees]
    //   az_deg   — satellite azimuth  [degrees]
    //   lat_deg  — receiver geodetic latitude  [degrees]
    //   lon_deg  — receiver geodetic longitude [degrees]
    //
    // The receiver will subtract a correction computed from the SAME coefficients
    // we broadcast in SF4 Page 18.  By embedding the matching iono delay in our
    // pseudoranges, the net receiver-side residual is ≈0 — consistent signals.
    // Without this, each SV is SHORT by 5-15 m (elevation-dependent), and the
    // position solver finds a biased 3D solution or, for marginal geometry, no fix.
    inline double klobuchar_delay_m(const IonoModel& iono,
                                     double gps_tow,
                                     double el_deg,  double az_deg,
                                     double lat_deg, double lon_deg)
    {
        using namespace GpsConst;

        const double E      = el_deg  / 180.0;   // elevation in semicircles
        const double phi_u  = lat_deg / 180.0;   // lat in semicircles
        const double lam_u  = lon_deg / 180.0;   // lon in semicircles
        const double az_rad = az_deg  * PI / 180.0;

        // Obliquity factor (IS-GPS-200H eq. 4)
        const double F = 1.0 + 16.0 * std::pow(0.53 - E, 3.0);

        if (!iono.valid) {
            // No valid coefficients — use 5 ns nominal (IS-GPS-200H eq. 3)
            return F * 5.0e-9 * c;
        }

        // Earth's central angle to ionospheric pierce point (semicircles)
        const double psi = 0.0137 / (E + 0.11) - 0.022;

        // Geodetic latitude of pierce point (semicircles)
        double phi_i = phi_u + psi * std::cos(az_rad);
        if      (phi_i >  0.416) phi_i =  0.416;
        else if (phi_i < -0.416) phi_i = -0.416;

        // Geodetic longitude of pierce point (semicircles)
        const double lam_i = lam_u + psi * std::sin(az_rad) / std::cos(phi_i * PI);

        // Geomagnetic latitude of pierce point (semicircles)
        const double phi_m  = phi_i + 0.064 * std::cos((lam_i - 1.617) * PI);
        const double phi_m2 = phi_m  * phi_m;
        const double phi_m3 = phi_m2 * phi_m;

        // Amplitude of cosine term
        double AMP = iono.alpha[0] + iono.alpha[1]*phi_m
                   + iono.alpha[2]*phi_m2 + iono.alpha[3]*phi_m3;
        if (AMP < 0.0) AMP = 0.0;

        // Period of cosine term
        double PER = iono.beta[0] + iono.beta[1]*phi_m
                   + iono.beta[2]*phi_m2 + iono.beta[3]*phi_m3;
        if (PER < 72000.0) PER = 72000.0;

        // Local time (seconds) at pierce point
        double t_local = 43200.0 * lam_i + gps_tow;  // SECONDS_IN_DAY/2 * lon_sc + tow
        while (t_local >= 86400.0) t_local -= 86400.0;
        while (t_local <      0.0) t_local += 86400.0;

        // Phase argument
        const double X = 2.0 * PI * (t_local - 50400.0) / PER;

        double iono_delay_s;
        if (std::abs(X) < 1.57) {
            const double X2 = X * X;
            const double X4 = X2 * X2;
            iono_delay_s = F * (5.0e-9 + AMP * (1.0 - X2/2.0 + X4/24.0));
        } else {
            iono_delay_s = F * 5.0e-9;
        }

        return iono_delay_s * c;  // metres
    }

    inline GpsTime utc_to_gps(int year, int month, int day,
                               int hour, int min, double sec,
                               int leap_seconds = 18)
    {
        constexpr long GPS_EPOCH_JD = 2444244;
        const long a_term = (14 - month) / 12;
        const long y      = year + 4800 - a_term;
        const long m_term = month + 12*a_term - 3;
        const long jd     = day + (153*m_term + 2)/5 + 365*y
                            + y/4 - y/100 + y/400 - 32045;
        const long days_since_epoch = jd - GPS_EPOCH_JD;
        const int  week             = static_cast<int>(days_since_epoch / 7);
        const long day_of_week      = days_since_epoch % 7;
        const double tow            = static_cast<double>(day_of_week) * 86400.0
                                    + hour * 3600.0 + min * 60.0 + sec
                                    + static_cast<double>(leap_seconds);
        return GpsTime{week, tow};
    }

} // namespace CoordinateMath


// =============================================================================
// SECTION 4: SatCalc CLASS
// =============================================================================

class SatCalc
{
public:
    // IS-GPS-200H §20.3.3.4.3 (Table 20-IV) Keplerian propagation
    static void compute_sat_state(const Ephemeris& eph,
                                  double t,
                                  ECEF&    pos_out,
                                  ECEFVel& vel_out,
                                  double&  clock_corr_out);

    // Full target calculation.
    //
    //   FIX-TM-1: code_phase_chips is now computed correctly as
    //     fmod((gps_tow - pseudorange/c) * chip_rate, 1023)
    //
    //   FIX-TM-2: if iono is non-null (valid), Klobuchar delay is added to
    //     flight_time_s and range_m BEFORE code_phase is computed, so the
    //     pseudorange embeds the iono delay the receiver will subtract.
    //
    //   user_vel — ECEF translational velocity of the receiver [m/s].
    //              Pass ECEFVel{} for a stationary receiver.
    //
    //   iono     — Klobuchar parameters from SF4 Page 18.
    //              Pass nullptr (or default) to skip iono correction.
    //              The receiver will subtract its own Klobuchar model; if we
    //              don't include the matching delay, each SV pseudorange has
    //              an uncompensated error of 5–15 m.
    static TargetState calculate_target(const Ephemeris& eph,
                                        const WGS84&     user_pos,
                                        double           current_gps_tow,
                                        ECEFVel          user_vel  = ECEFVel{},
                                        const IonoModel* iono      = nullptr);
};

#endif // TRAJECTORY_MODULE_H
