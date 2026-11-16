// gpssim_nav.h — Ported directly from gpssim.c's proven nav message + IQ synthesis
#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <array>
#include <complex>
#include <vector>
#include <iostream>
#include <iomanip>

// Constants from gpssim.h
#define GPS_CA_SEQ_LEN 1023
#define GPS_N_SBF 5
#define GPS_N_DWRD_SBF 10
#define GPS_N_DWRD ((GPS_N_SBF+1)*GPS_N_DWRD_SBF)
#define GPS_PI 3.1415926535898
#define GPS_SPEED_OF_LIGHT 2.99792458e8
#define GPS_CARR_FREQ 1575.42e6
#define GPS_CODE_FREQ 1.023e6

// Sin/cos lookup tables from gpssim.c (512 entries, amplitude ±250)
static const int sinTable512[] = {
   2,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,74,77,80,83,86,89,91,94,
   97,100,103,105,108,111,114,116,119,122,125,127,130,132,135,138,140,143,145,148,150,153,155,157,160,162,164,167,169,171,173,176,
   178,180,182,184,186,188,190,192,194,196,198,200,202,204,205,207,209,210,212,214,215,217,218,220,221,223,224,225,227,228,229,230,
   232,233,234,235,236,237,238,239,240,241,241,242,243,244,244,245,245,246,247,247,248,248,248,249,249,249,249,250,250,250,250,250,
   250,250,250,250,250,249,249,249,249,248,248,248,247,247,246,245,245,244,244,243,242,241,241,240,239,238,237,236,235,234,233,232,
   230,229,228,227,225,224,223,221,220,218,217,215,214,212,210,209,207,205,204,202,200,198,196,194,192,190,188,186,184,182,180,178,
   176,173,171,169,167,164,162,160,157,155,153,150,148,145,143,140,138,135,132,130,127,125,122,119,116,114,111,108,105,103,100,97,
   94,91,89,86,83,80,77,74,71,68,65,62,59,56,53,50,47,44,41,38,35,32,29,26,23,20,17,14,11,8,5,2,
   -2,-5,-8,-11,-14,-17,-20,-23,-26,-29,-32,-35,-38,-41,-44,-47,-50,-53,-56,-59,-62,-65,-68,-71,-74,-77,-80,-83,-86,-89,-91,-94,
   -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
   -178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
   -232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
   -250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
   -230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
   -176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100,-97,
   -94,-91,-89,-86,-83,-80,-77,-74,-71,-68,-65,-62,-59,-56,-53,-50,-47,-44,-41,-38,-35,-32,-29,-26,-23,-20,-17,-14,-11,-8,-5,-2
};
static const int cosTable512[] = {
   250,250,250,250,250,249,249,249,249,248,248,248,247,247,246,245,245,244,244,243,242,241,241,240,239,238,237,236,235,234,233,232,
   230,229,228,227,225,224,223,221,220,218,217,215,214,212,210,209,207,205,204,202,200,198,196,194,192,190,188,186,184,182,180,178,
   176,173,171,169,167,164,162,160,157,155,153,150,148,145,143,140,138,135,132,130,127,125,122,119,116,114,111,108,105,103,100,97,
   94,91,89,86,83,80,77,74,71,68,65,62,59,56,53,50,47,44,41,38,35,32,29,26,23,20,17,14,11,8,5,2,
   -2,-5,-8,-11,-14,-17,-20,-23,-26,-29,-32,-35,-38,-41,-44,-47,-50,-53,-56,-59,-62,-65,-68,-71,-74,-77,-80,-83,-86,-89,-91,-94,
   -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
   -178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
   -232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
   -250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
   -230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
   -176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100,-97,
   -94,-91,-89,-86,-83,-80,-77,-74,-71,-68,-65,-62,-59,-56,-53,-50,-47,-44,-41,-38,-35,-32,-29,-26,-23,-20,-17,-14,-11,-8,-5,-2,
   2,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,74,77,80,83,86,89,91,94,
   97,100,103,105,108,111,114,116,119,122,125,127,130,132,135,138,140,143,145,148,150,153,155,157,160,162,164,167,169,171,173,176,
   178,180,182,184,186,188,190,192,194,196,198,200,202,204,205,207,209,210,212,214,215,217,218,220,221,223,224,225,227,228,229,230,
   232,233,234,235,236,237,238,239,240,241,241,242,243,244,244,245,245,246,247,247,248,248,248,249,249,249,249,250,250,250,250,250
};

// ── countBits from gpssim.c ──
static inline unsigned long gps_countBits(unsigned long v) {
    unsigned long c = v;
    c = ((c >> 1) & 0x55555555) + (c & 0x55555555);
    c = ((c >> 2) & 0x33333333) + (c & 0x33333333);
    c = ((c >> 4) & 0x0F0F0F0F) + (c & 0x0F0F0F0F);
    c = ((c >> 8) & 0x00FF00FF) + (c & 0x00FF00FF);
    c = ((c >>16) & 0x0000FFFF) + (c & 0x0000FFFF);
    return c;
}

// ── computeChecksum from gpssim.c — bit-exact copy ──
static unsigned long gps_computeChecksum(unsigned long source, int nib) {
    unsigned long bmask[6] = {
        0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
        0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };
    unsigned long d = source & 0x3FFFFFC0UL;
    unsigned long D29 = (source>>31)&0x1UL;
    unsigned long D30 = (source>>30)&0x1UL;
    if (nib) {
        if ((D30 + gps_countBits(bmask[4] & d)) % 2) d ^= (0x1UL<<6);
        if ((D29 + gps_countBits(bmask[5] & d)) % 2) d ^= (0x1UL<<7);
    }
    unsigned long D = d;
    if (D30) D ^= 0x3FFFFFC0UL;
    D |= ((D29 + gps_countBits(bmask[0] & d)) % 2) << 5;
    D |= ((D30 + gps_countBits(bmask[1] & d)) % 2) << 4;
    D |= ((D29 + gps_countBits(bmask[2] & d)) % 2) << 3;
    D |= ((D30 + gps_countBits(bmask[3] & d)) % 2) << 2;
    D |= ((D30 + gps_countBits(bmask[4] & d)) % 2) << 1;
    D |= ((D29 + gps_countBits(bmask[5] & d)) % 2);
    D &= 0x3FFFFFFFUL;
    return D;
}

// Forward declaration
struct Ephemeris;
struct KlobucharParams;

// ── GpsSimChannel: per-satellite state, ported from gpssim.c channel_t ──
struct GpsSimChannel {
    int prn = 0;
    int ca[GPS_CA_SEQ_LEN];      // C/A code (0/1)
    double f_carr = 0.0;         // carrier frequency offset (Hz)
    double f_code = GPS_CODE_FREQ; // code frequency (Hz)
    double carr_phase = 0.0;     // [0..1) carrier phase
    double code_phase = 0.0;     // [0..1023) code phase
    unsigned long sbf[5][GPS_N_DWRD_SBF]; // subframe templates
    unsigned long dwrd[GPS_N_DWRD];       // encoded words for current 30s frame
    int iword = 0;
    int ibit = 0;
    int icode = 0;
    int dataBit = 1;   // ±1
    int codeCA = 1;    // ±1
    double rho0_range = 0.0;   // Previous pseudorange [m] (gpssim chan->rho0.range)
    double rho0_gps_sec = 0.0; // GPS TOW when rho0_range was measured
    double g0_sec = 0.0;       // Nav data reference time (30 s aligned, gpssim chan->g0)

    // Generate C/A code for this PRN (from gpssim.c)
    void generateCAcode() {
        static const int delay[33] = {
            0,5,6,7,8,17,18,139,140,141,251,252,254,255,256,257,258,
            469,470,471,472,473,474,509,512,513,514,515,516,859,860,861,862
        };
        if (prn < 1 || prn > 32) return;
        int g1[GPS_CA_SEQ_LEN], g2[GPS_CA_SEQ_LEN];
        int r1[10], r2[10];
        for (int i = 0; i < 10; i++) { r1[i] = -1; r2[i] = -1; }
        for (int i = 0; i < GPS_CA_SEQ_LEN; i++) {
            g1[i] = r1[9]; g2[i] = r2[9];
            int s1 = r1[2]*r1[9]; int s2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];
            for (int j = 9; j > 0; j--) { r1[j] = r1[j-1]; r2[j] = r2[j-1]; }
            r1[0] = s1; r2[0] = s2;
        }
        for (int i = 0; i < GPS_CA_SEQ_LEN; i++) {
            ca[i] = (1-g1[i]*g2[(i+delay[prn])%GPS_CA_SEQ_LEN])/2;
        }
    }

    // ── eph2sbf: exact port from gpssim.c ──
    void eph2sbf(const Ephemeris& eph, const KlobucharParams& iono);

    // ── generateNavMsg: exact port from gpssim.c ──
    void generateNavMsg(int week, double sec, bool init) {
        int iwrd, isbf;
        unsigned long wn = (unsigned long)(week % 1024);
        // Align to 30-second boundary (gpssim.c generateNavMsg → chan->g0)
        g0_sec = (double)(((unsigned long)(sec+0.5))/30UL) * 30.0;
        unsigned long tow = ((unsigned long)g0_sec)/6UL;
        unsigned long sbfwrd, prevwrd;
        int nib;

        if (init) {
            prevwrd = 0UL;
            for (iwrd = 0; iwrd < GPS_N_DWRD_SBF; iwrd++) {
                sbfwrd = sbf[4][iwrd];
                if (iwrd == 1) sbfwrd |= ((tow & 0x1FFFFUL) << 13);
                sbfwrd |= (prevwrd << 30) & 0xC0000000UL;
                nib = ((iwrd==1)||(iwrd==9)) ? 1 : 0;
                dwrd[iwrd] = gps_computeChecksum(sbfwrd, nib);
                prevwrd = dwrd[iwrd];
            }
        } else {
            for (iwrd = 0; iwrd < GPS_N_DWRD_SBF; iwrd++) {
                dwrd[iwrd] = dwrd[GPS_N_DWRD_SBF * GPS_N_SBF + iwrd];
                prevwrd = dwrd[iwrd];
            }
        }

        for (isbf = 0; isbf < GPS_N_SBF; isbf++) {
            tow++;
            for (iwrd = 0; iwrd < GPS_N_DWRD_SBF; iwrd++) {
                sbfwrd = sbf[isbf][iwrd];
                if ((isbf==0) && (iwrd==2)) sbfwrd |= (wn & 0x3FFUL) << 20;
                if (iwrd == 1) sbfwrd |= ((tow & 0x1FFFFUL) << 13);
                sbfwrd |= (prevwrd << 30) & 0xC0000000UL;
                nib = ((iwrd==1)||(iwrd==9)) ? 1 : 0;
                dwrd[(isbf+1)*GPS_N_DWRD_SBF + iwrd] = gps_computeChecksum(sbfwrd, nib);
                prevwrd = dwrd[(isbf+1)*GPS_N_DWRD_SBF + iwrd];
            }
        }

        // Do NOT reset iword/ibit/icode/dataBit here — gpssim.c leaves them
        // unchanged; computeCodePhase() sets all counters from pseudorange.
    }
};

// end of gpssim_nav.h

