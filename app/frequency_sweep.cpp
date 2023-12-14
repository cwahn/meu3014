#include <signal.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

#include "hal.hpp"

using namespace efp;
using namespace std::chrono;

// What has to be done in sweeping voltage to get velocity data
// 0 ~ 3.3 and 3.3 ~ 0
// rise time is assumed to be < 10 ms -> each voltage data will be collected for 10 times of it i.e. 100ms
constexpr int period_per_freq_us = 100 * 1000;

// Step of voltage should be around 0.1 %
constexpr int cmd_step_num = 1000;
constexpr double cmd_rel_step = 1. / static_cast<double>(cmd_step_num);

constexpr double min_freq_hz = 100;
constexpr double max_freq_hz = 1000;

// HAL input

void signal_handler(int signum)
{
    info("Signal {} received in main thread", signum);

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    info("Terminating program.");
    exit(signum);
}

int main()
{
    // Block all signals for this thread (main thread)
    sigset_t set;
    sigfillset(&set);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    // Spawn threads
    Logger::set_log_level(LogLevel::Trace);

    info("Starting MEU3014 2nd project.");

    debug("Initializing PIGPIO.");
    if (gpioInitialise() < 0)
    {
        fatal("Failed to initialize PIGPIO.");
        return -1;
    }

    debug("Setting HALs.");
    auto &encoder = Encoder::instance();
    auto &motor = DcMotor::instance();

    // Unblocking signals in the main thread
    sigemptyset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Register signal handler for main thread
    signal(SIGINT, signal_handler);

    const Vector<double> cmds_hz =
        from_function(cmd_step_num, [](size_t i)
                      { return i * cmd_rel_step * (max_freq_hz - min_freq_hz) + min_freq_hz; });

    Vector<Vector<int>> posss_pulse{};

    const auto collect_poss_at_freq = [&](double cmd_hz)
    {
        debug("Collecting data of {} Hz", cmd_hz);

        Vector<int> poss_pulse{};
        poss_pulse.reserve(1.5 * period_per_freq_us);

        const auto start_time_us = now_us();

        while (now_us() < start_time_us + period_per_freq_us)
        {
            poss_pulse.push_back(encoder());
            // todo maybe other periodic
            std::this_thread::sleep_for(milliseconds(1));
        }

        posss_pulse.push_back(poss_pulse);
    };

    for_each(collect_poss_at_freq, cmds_hz);

    // for_each([&](double cmd_hz, double vel_pulse_s)
    //          {
    //     info("voltage_v: {}, velocity_pulse_per_sec: {}", cmd_hz, vel_pulse_s);
    //     std::this_thread::sleep_for(milliseconds(1)); },
    //          cmds_v, velocities_pulse_per_s);

    

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(1000));
    return 0;
}
