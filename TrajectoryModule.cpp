// =============================================================================
// TrajectoryModule.cpp  — FIXED VERSION
// GPS L1 Real-Time Spoofer — Trajectory & Signal Parameter Calculator
// =============================================================================
//
// FIXES:
//
//   FIX-TM-1 (CRITICAL — was the PRIMARY cause of "15 SVs visible, 0 in use"):
//     Code phase formula corrected to match IS-GPS-200H and gpssim.c.
//
//     WRONG (old):
//       chips = (flight_time - clock_corr) * chip_rate
//       code_phase = fmod(chips, 1023)
//       → This gives the fractional chip count of the PROPAGATION TIME only.
//         GPS code phase is NOT the propagation time; it is the fractional chip
//         position of the satellite's code at GPS receive time.  The value above
//         is nearly CONSTANT across satellites (~71 610 chips for a 70 ms path,
//         always very close to 0 mod 1023), so all SVs transmit the same code
//         phase and the receiver can correlate but computes wrong pseudoranges.
//
//     CORRECT (new):
//       pseudorange_s = flight_time - clock_corr   [includes tropo + iono]
//       t_sv_tx       = gps_tow - pseudorange_s    [GPS time of transmission]
//       code_phase    = fmod(t_sv_tx * chip_rate, 1023)
//       → Each satellite has a unique, GPS-epoch-referenced code phase.
//         The receiver recovers t_sv_tx from this, computes pseudorange as
//         (t_rx - t_sv_tx)*c, and uses the HOW TOW to resolve the 1 ms ambiguity.
//         This matches gpssim.c computeCodePhase() exactly.
//
//   FIX-TM-2 (HIGH — pseudorange accuracy):
//     Klobuchar ionospheric delay is now computed and embedded in flight_time_s
//     and range_m BEFORE the code phase is calculated.
//     The receiver subtracts a correction using the same alpha/beta coefficients
//     we broadcast in Subframe 4 Page 18 — so the net residual is near zero.
//     Without this, pseudoranges are short by 5-15 m per satellite and the
//     position solution has elevation-dependent biases.
//
//   FIX-TM-3: Tropospheric delay removed from pseudorange (gpssim.c adds iono only).
//
// =============================================================================

#include "TrajectoryModule.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

// =============================================================================
// SatCalc::compute_sat_state
// IS-GPS-200H §20.3.3.4.3, Table 20-IV — Keplerian propagation
// =============================================================================

void SatCalc::compute_sat_state(const Ephemeris& eph,
                                 double           t,
                                 ECEF&            pos_out,
                                 ECEFVel&         vel_out,
                                 double&          clock_corr_out)
{
    using namespace GpsConst;

    // Step 0: Time from ephemeris epoch — correct for week crossovers
    double tk = t - eph.toe;
    if (tk >  302400.0) tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    // Step 1: SV clock correction (IS-GPS-200H §20.3.3.3.3.1)
    double dt_clock = t - eph.toc;
    if (dt_clock >  302400.0) dt_clock -= 604800.0;
    if (dt_clock < -302400.0) dt_clock += 604800.0;
    const double clock_bias = eph.af0
                            + eph.af1 * dt_clock
                            + eph.af2 * dt_clock * dt_clock
                            - eph.Tgd;

    // Step 2: Semi-major axis and corrected mean motion
    const double A   = eph.sqrt_A * eph.sqrt_A;
    const double n0  = std::sqrt(mu / (A*A*A));
    const double n   = n0 + eph.delta_n;

    // Step 3: Mean anomaly
    const double Mk  = eph.M0 + n * tk;

    // Step 4: Kepler's equation — Newton-Raphson for eccentric anomaly Ek
    double Ek = Mk;
    for (int i = 0; i < MAX_ITER; ++i) {
        const double f_Ek  = Ek - eph.e * std::sin(Ek) - Mk;
        const double fp_Ek = 1.0 - eph.e * std::cos(Ek);
        const double dEk   = f_Ek / fp_Ek;
        Ek -= dEk;
        if (std::abs(dEk) < ITER_TOL) break;
    }

    // Relativistic clock correction (IS-GPS-200H eq. 1)
    const double F_rel   = -2.0 * std::sqrt(mu) / (c * c);
    const double delta_tr = F_rel * eph.e * eph.sqrt_A * std::sin(Ek);
    clock_corr_out = clock_bias + delta_tr;

    // Step 5: True anomaly
    const double sin_Ek    = std::sin(Ek);
    const double cos_Ek    = std::cos(Ek);
    const double sqrt_1me2 = std::sqrt(1.0 - eph.e * eph.e);
    const double vk        = std::atan2(sqrt_1me2 * sin_Ek, cos_Ek - eph.e);

    // Step 6: Argument of latitude
    const double Phi_k = vk + eph.omega;

    // Step 7: Harmonic perturbations
    const double sin2Phi = std::sin(2.0 * Phi_k);
    const double cos2Phi = std::cos(2.0 * Phi_k);
    const double delta_uk = eph.Cus * sin2Phi + eph.Cuc * cos2Phi;
    const double delta_rk = eph.Crs * sin2Phi + eph.Crc * cos2Phi;
    const double delta_ik = eph.Cis * sin2Phi + eph.Cic * cos2Phi;

    // Step 8: Corrected orbital elements
    const double uk = Phi_k + delta_uk;
    const double rk = A * (1.0 - eph.e * cos_Ek) + delta_rk;
    const double ik = eph.i0 + delta_ik + eph.idot * tk;

    // Step 9: Orbital plane position
    const double xk_prime = rk * std::cos(uk);
    const double yk_prime = rk * std::sin(uk);

    // Step 10: Corrected longitude of ascending node
    const double Omega_k = eph.omega0
                         + (eph.omega_dot - omega_e) * tk
                         - omega_e * eph.toe;

    // Step 11: ECEF position
    const double sin_Ok = std::sin(Omega_k);
    const double cos_Ok = std::cos(Omega_k);
    const double sin_ik = std::sin(ik);
    const double cos_ik = std::cos(ik);

    pos_out.x = xk_prime * cos_Ok - yk_prime * cos_ik * sin_Ok;
    pos_out.y = xk_prime * sin_Ok + yk_prime * cos_ik * cos_Ok;
    pos_out.z = yk_prime * sin_ik;

    // Step 12: ECEF velocity (Misra & Enge Appendix 4B)
    const double Ek_dot       = n / (1.0 - eph.e * cos_Ek);
    const double vk_dot       = Ek_dot * sqrt_1me2 / (1.0 - eph.e * cos_Ek);
    const double uk_dot       = vk_dot + 2.0*(eph.Cus*cos2Phi - eph.Cuc*sin2Phi)*vk_dot;
    const double rk_dot       = A * eph.e * sin_Ek * Ek_dot
                              + 2.0*(eph.Crs*cos2Phi - eph.Crc*sin2Phi)*vk_dot;
    const double ik_dot       = eph.idot + 2.0*(eph.Cis*cos2Phi - eph.Cic*sin2Phi)*vk_dot;
    const double Omega_dot_k  = eph.omega_dot - omega_e;

    const double xk_prime_dot = rk_dot * std::cos(uk) - rk * uk_dot * std::sin(uk);
    const double yk_prime_dot = rk_dot * std::sin(uk) + rk * uk_dot * std::cos(uk);

    vel_out.vx = (xk_prime_dot - yk_prime * cos_ik * Omega_dot_k) * cos_Ok
               - (xk_prime * Omega_dot_k + yk_prime_dot * cos_ik
                  - yk_prime * sin_ik * ik_dot) * sin_Ok;

    vel_out.vy = (xk_prime_dot - yk_prime * cos_ik * Omega_dot_k) * sin_Ok
               + (xk_prime * Omega_dot_k + yk_prime_dot * cos_ik
                  - yk_prime * sin_ik * ik_dot) * cos_Ok;

    vel_out.vz = yk_prime_dot * sin_ik + yk_prime * cos_ik * ik_dot;
}


// =============================================================================
// SatCalc::calculate_target  — FIXED
// =============================================================================

TargetState SatCalc::calculate_target(const Ephemeris& eph,
                                       const WGS84&     user_pos,
                                       double           current_gps_tow,
                                       ECEFVel          user_vel,
                                       const IonoModel* iono)
{
    using namespace GpsConst;
    using namespace CoordinateMath;

    TargetState result{};
    result.sv_id = eph.sv_id;

    // ── User ECEF ─────────────────────────────────────────────────────────────
    const ECEF user_ecef = lla_to_ecef(user_pos);

    // ── Step A: Iterative flight-time solve ───────────────────────────────────
    // t_tx = t_rx - tau  (tau = rho/c depends on sat pos at t_tx — iterate)
    double tau = 0.070;   // Initial estimate: ~20 200 km altitude / c ≈ 67–86 ms

    ECEF    sat_pos_tx;
    ECEFVel sat_vel;
    double  clock_corr;
    ECEF    sat_pos_sagnac;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        const double t_tx = current_gps_tow - tau;
        compute_sat_state(eph, t_tx, sat_pos_tx, sat_vel, clock_corr);
        sat_pos_sagnac = apply_sagnac(sat_pos_tx, tau);
        const double rho     = ecef_distance(sat_pos_sagnac, user_ecef);
        const double tau_new = rho / c;
        const double dtau    = tau_new - tau;
        tau = tau_new;
        if (std::abs(dtau) < ITER_TOL) break;
    }

    result.flight_time_s = tau;
    result.sat_pos       = sat_pos_sagnac;
    result.sat_vel       = sat_vel;
    result.range_m       = ecef_distance(sat_pos_sagnac, user_ecef);
    result.clock_corr_s  = clock_corr;

    // ── Step B: Line-of-sight unit vector ────────────────────────────────────
    const double los_x = (sat_pos_sagnac.x - user_ecef.x) / result.range_m;
    const double los_y = (sat_pos_sagnac.y - user_ecef.y) / result.range_m;
    const double los_z = (sat_pos_sagnac.z - user_ecef.z) / result.range_m;

    // ── Step C: Elevation and azimuth ─────────────────────────────────────────
    // Computed BEFORE atmospheric corrections so we can use el/az for models.
    elevation_azimuth(user_ecef, sat_pos_sagnac,
                      result.elevation_deg, result.azimuth_deg);

    // Tropospheric delay is NOT embedded here — gpssim.c computeRange adds only
    // ionospheric delay. Receivers apply their own troposphere model internally.

    // ── Step D: Klobuchar ionospheric delay ───────────────────────────────────
    // FIX-TM-2: Add iono delay so receiver's SF4-Page18-based Klobuchar
    // correction cancels it — giving a clean geometric pseudorange residual.
    //
    // Without this step:
    //   - Receiver subtracts ~5-15 m per SV that isn't in our signal
    //   - Pseudoranges are SHORT by an elevation-dependent amount
    //   - Position solver sees inconsistent residuals → degraded or failed fix
    //
    // With valid IonoModel: compute full Klobuchar delay [m] and add it.
    // With nullptr / invalid: add nominal 5 ns × c ≈ 1.5 m (obliquity-scaled).
    {
        const double iono_m = klobuchar_delay_m(
            (iono != nullptr) ? *iono : IonoModel{},
            current_gps_tow,
            result.elevation_deg, result.azimuth_deg,
            user_pos.lat_deg, user_pos.lon_deg);

        result.flight_time_s += iono_m / c;
        result.range_m       += iono_m;
    }

    // ── Step E: Doppler shift ─────────────────────────────────────────────────
    // Radial velocity = (v_sat - v_user) projected on LOS.
    // User velocity has two components:
    //   1. Earth-rotation (always present for any point on Earth)
    //   2. Translational (drone motion, etc.)
    // Sign: v_relative > 0 → satellite receding → frequency decreases.
    const double v_sat_radial   = sat_vel.vx * los_x
                                + sat_vel.vy * los_y
                                + sat_vel.vz * los_z;

    const double v_earth_radial = omega_e * (-user_ecef.y * los_x
                                            +  user_ecef.x * los_y);

    const double v_user_trans   = user_vel.vx * los_x
                                + user_vel.vy * los_y
                                + user_vel.vz * los_z;

    const double v_relative = v_sat_radial - (v_earth_radial + v_user_trans);
    result.doppler_hz = -f_L1 * v_relative / c;

    // ── Step F: C/A Code Phase ────────────────────────────────────────────────
    //
    // FIX-TM-1 — THE CRITICAL FIX.
    //
    // WRONG (original):
    //   chips_traveled = (flight_time - clock_corr) * chip_rate
    //   code_phase     = fmod(chips_traveled, 1023)
    //
    //   Problem: this gives the chip count of the ONE-WAY propagation time,
    //   which is ~67-86 ms → 68 610 – 87 990 chips → nearly identical for
    //   all satellites (all fall close to 0 mod 1023 ≈ 70 * 1023 = 71 610).
    //   The receiver CAN acquire (C/N0 is fine) but when it computes the
    //   pseudorange as c * (t_rx - t_sv_tx), it gets a wildly wrong t_sv_tx
    //   because the code phase doesn't encode the absolute GPS time of
    //   transmission.  Result: 15 SVs visible, 0 in use, No Fix.
    //
    // CORRECT (gpssim.c computeCodePhase() approach):
    //   pseudorange_s = flight_time - clock_corr   (total pseudorange / c,
    //                                               including tropo and iono)
    //   t_sv_tx       = gps_tow - pseudorange_s    (GPS time of transmission)
    //   code_phase    = fmod(t_sv_tx * chip_rate, 1023)
    //
    //   Physics: the C/A code repeats every 1 ms.  Within any 1 ms epoch the
    //   chip number is fmod(t_sv * chip_rate, 1023).  At GPS receive time
    //   gps_tow, the chip being received was transmitted at t_sv_tx — so the
    //   code phase "stamped" on the signal is fmod(t_sv_tx * chip_rate, 1023).
    //   This is unique per satellite (different ranges → different t_sv_tx)
    //   and changes continuously with time (as the satellite moves).
    //   The receiver can read t_sv_tx from this code phase, subtract from
    //   its own clock (gps_tow), multiply by c, and get a correct pseudorange.
    //   Combined with the HOW TOW (which resolves the 1 ms ambiguity), the
    //   receiver computes a valid 3D fix.
    {
        const double pseudorange_s = result.flight_time_s - result.clock_corr_s;
        const double t_sv_tx       = current_gps_tow - pseudorange_s;
        // 1023 = C/A code length in chips (CA_SEQ_LEN per IS-GPS-200H)
        constexpr double CA_LEN = 1023.0;
        double code_phase = std::fmod(t_sv_tx * chip_rate, CA_LEN);
        if (code_phase < 0.0) code_phase += CA_LEN;
        result.code_phase_chips = code_phase;
    }

    return result;
}


// =============================================================================
// Standalone test / verification entry point
// Compile: g++ -std=c++17 -O2 -DTRAJECTORY_MODULE_STANDALONE_TEST
//              -o trajectory_test TrajectoryModule.cpp
// =============================================================================

#ifdef TRAJECTORY_MODULE_STANDALONE_TEST

int main()
{
    using namespace CoordinateMath;

    std::cout << "=== TrajectoryModule Self-Test (Fixed Version) ===\n\n";

    // ── Test 1: LLA ↔ ECEF round-trip ────────────────────────────────────────
    {
        WGS84 original{17.604012, 78.126885, 100.0};
        ECEF  ecef = lla_to_ecef(original);
        WGS84 back = ecef_to_lla(ecef);
        std::cout << "LLA→ECEF→LLA round-trip error: "
                  << std::abs(back.lat_deg - original.lat_deg) * 111000.0
                  << " mm lat  "
                  << std::abs(back.alt_m - original.alt_m) * 1000.0 << " mm alt\n\n";
    }

    // ── Test 2: Satellite propagation + code phase ────────────────────────────
    {
        Ephemeris eph{};
        eph.sv_id = 5;  eph.week = 2311;
        eph.toc   = 388800.0;  eph.toe = 388800.0;
        eph.af0 = -4.456e-5;  eph.sqrt_A = 5153.716;  eph.e = 0.01e-3;
        eph.i0  = 0.957;  eph.omega0 = 1.234;  eph.omega = 0.876;
        eph.M0  = -0.432;  eph.delta_n = 4.2e-9;  eph.omega_dot = -7.8e-9;
        eph.idot = -2.1e-10;  eph.Crc = 210.0;  eph.Crs = -56.0;
        eph.Cuc =  1.2e-6;  eph.Cus = -3.4e-6;  eph.Cic = 9.3e-8;
        eph.Cis = -1.9e-7;  eph.Tgd = 1.2e-8;

        const WGS84  user{17.604012, 78.126885, 100.0};
        const double tow = 389000.0;

        // Without iono
        const TargetState ts = SatCalc::calculate_target(eph, user, tow);
        std::cout << "SV" << ts.sv_id << " at TOW=" << tow << ":\n"
                  << "  Range:       " << ts.range_m/1000.0 << " km\n"
                  << "  Flight time: " << ts.flight_time_s*1000.0 << " ms\n"
                  << "  Doppler:     " << ts.doppler_hz << " Hz\n"
                  << "  Code phase:  " << ts.code_phase_chips << " chips  "
                  << "(FIXED: should be fmod((tow-PR/c)*chip_rate,1023))\n"
                  << "  Elevation:   " << ts.elevation_deg << " deg\n\n";

        // Verify code phase formula
        // Expected: pseudorange_s = flight_time - clock_corr
        //           t_sv_tx = tow - pseudorange_s
        //           code_phase = fmod(t_sv_tx * 1.023e6, 1023)
        const double pr_s  = ts.flight_time_s - ts.clock_corr_s;
        const double t_tx  = tow - pr_s;
        const double cp_expected = std::fmod(t_tx * GpsConst::chip_rate, 1023.0);        std::cout << "  Code phase verification: " << cp_expected << " chips "
                  << (std::abs(cp_expected - ts.code_phase_chips) < 0.001 ? "[PASS]" : "[FAIL]")
                  << "\n\n";
    }

    // ── Test 3: Klobuchar delay ───────────────────────────────────────────────
    {
        IonoModel iono;
        iono.valid    = true;
        iono.alpha[0] = 4.6566e-9;  iono.alpha[1] = 1.4901e-8;
        iono.alpha[2] =-5.9605e-8;  iono.alpha[3] =-5.9605e-8;
        iono.beta[0]  = 90112.0;    iono.beta[1]  = 98304.0;
        iono.beta[2]  =-65536.0;    iono.beta[3]  =-458752.0;

        const double delay = CoordinateMath::klobuchar_delay_m(
            iono, 389000.0, 45.0, 180.0, 17.6, 78.1);
        std::cout << "Klobuchar delay at 45° elevation: "
                  << delay << " m  (expect ~3-8 m in daytime)\n\n";
    }

    std::cout << "=== All tests done ===\n";
    return 0;
}

#endif // TRAJECTORY_MODULE_STANDALONE_TEST
