// tx_gpssim.cpp — GPS IQ streaming transmitter for B210
//
// Supports TWO modes:
//   -f FILE   : load and play a single IQ file (original behaviour)
//   -f -      : STREAMING mode — reads raw int16 IQ from stdin/pipe continuously
//               → ZERO GAP between chunks when fed via named pipe
//
// Build: g++ -std=c++17 -O2 -o tx_gpssim tx_gpssim.cpp $(pkg-config --cflags --libs uhd) -lpthread

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/stream.hpp>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/stat.h>

static volatile bool g_stop = false;
void sig_handler(int) { g_stop = true; }

int main(int argc, char** argv)
{
    std::string filename  = "gpssim.bin";
    double      gain      = 75.0;
    double      samp_rate = 2600000.0;
    bool        phase1_only = false;
    std::string args;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "-f" && i+1 < argc) filename     = argv[++i];
        else if (a == "-g" && i+1 < argc) gain         = std::stod(argv[++i]);
        else if (a == "-s" && i+1 < argc) samp_rate    = std::stod(argv[++i]);
        else if (a == "--phase1-only")    phase1_only  = true;
        else if (a == "-a" && i+1 < argc) args         = argv[++i];
    }

    std::signal(SIGINT, sig_handler);

    bool streaming = (filename == "-");

    // Auto-detect named pipe/FIFO — ftell() returns garbage on FIFOs
    if (!streaming && filename != "-") {
        struct stat st;
        if (stat(filename.c_str(), &st) == 0 && S_ISFIFO(st.st_mode)) {
            streaming = true;
            std::cout << "[TX] Detected FIFO/pipe — switching to streaming mode.\n";
        }
    }

    // ── Init USRP (done ONCE, stays open across all chunks) ──────────────
    std::cout << "[TX] Initializing B210 (single session — no gaps)...\n";
    auto usrp = uhd::usrp::multi_usrp::make(args);
    usrp->set_tx_rate(samp_rate);
    usrp->set_tx_freq(uhd::tune_request_t(1575.42e6));
    usrp->set_tx_gain(gain);
    usrp->set_tx_antenna("TX/RX");

    while (!usrp->get_tx_sensor("lo_locked").to_bool() && !g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "[TX] LO locked. TX @ 1575.42 MHz, " << samp_rate/1e6
              << " Msps, gain=" << gain << " dB\n";

    auto tx_stream = usrp->get_tx_stream(uhd::stream_args_t("fc32"));

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = false;

    const float scale = 1.0f / 32767.0f;
    const size_t BLOCK = 2600;  // 1ms per send — aligned to C/A code period

    if (streaming) {
        // ── STREAMING MODE: read int16 IQ from stdin, transmit continuously ─
        // Feed via named pipe: cat chunk1.bin chunk2.bin ... > /tmp/gps.pipe
        // Zero gap between chunks — EKF never resets position!
        std::cout << "[TX] STREAMING mode — reading from stdin (pipe)\n"
                  << "[TX] Feed chunks: cat chunk*.bin > /tmp/gps.pipe\n"
                  << "[TX] Ctrl+C to stop.\n\n";

        std::vector<int16_t> raw(BLOCK * 2);
        std::vector<std::complex<float>> iq(BLOCK);
        size_t total_samples = 0;

        while (!g_stop) {
            size_t got = fread(raw.data(), sizeof(int16_t) * 2, BLOCK, stdin);
            if (got == 0) {
                if (feof(stdin)) { std::cout << "\n[TX] End of stream.\n"; break; }
                continue;
            }
            for (size_t i = 0; i < got; i++)
                iq[i] = {raw[2*i]*scale, raw[2*i+1]*scale};
            tx_stream->send(iq.data(), got, md);
            md.start_of_burst = false;
            total_samples += got;

            if (total_samples % (size_t)(samp_rate * 10) < BLOCK) {
                double secs = total_samples / samp_rate;
                std::cout << "[TX] Streaming: " << (int)secs << "s transmitted\n";
            }
        }
    } else {
        // ── FILE MODE: load and play a single IQ file ─────────────────────
        FILE* f = fopen(filename.c_str(), "rb");
        if (!f) { std::cerr << "Cannot open " << filename << "\n"; return 1; }
        fseek(f, 0, SEEK_END);
        size_t file_bytes = ftell(f);
        fseek(f, 0, SEEK_SET);

        const size_t n_samples = file_bytes / (sizeof(int16_t) * 2);
        const double file_dur  = n_samples / samp_rate;

        std::cout << "[TX] File    : " << filename << "\n"
                  << "[TX] Samples : " << n_samples << "  (" << file_dur << " s)\n"
                  << "[TX] Transmitting...\n";

        std::vector<int16_t> raw(BLOCK * 2);
        std::vector<std::complex<float>> iq_buf(BLOCK);
        size_t total = 0;

        while (!g_stop && total < n_samples) {
            size_t want = std::min(BLOCK, n_samples - total);
            size_t got  = fread(raw.data(), sizeof(int16_t)*2, want, f);
            if (got == 0) break;
            for (size_t i = 0; i < got; i++)
                iq_buf[i] = {raw[2*i]*scale, raw[2*i+1]*scale};
            tx_stream->send(iq_buf.data(), got, md);
            md.start_of_burst = false;
            total += got;
        }
        fclose(f);
        std::cout << "[TX] File playback complete.\n";
    }

    md.end_of_burst = true;
    tx_stream->send("", 0, md);
    std::cout << "[TX] Stopped.\n";
    return 0;
}
