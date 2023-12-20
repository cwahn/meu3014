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

// Step of voltage should be around 0.1 %
constexpr double max_voltage_v = 3.3;

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

    // Unblocking signals in the main thread
    sigemptyset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Register signal handler for main thread
    signal(SIGINT, signal_handler);

    periodic_while(
        microseconds(500000),
        [](microseconds time)
        { return time < seconds(10); },
        [](microseconds time)
        { info("Periodic time check: {} us", time.count()); });

    // Clean up
    debug("Terminatig PIGPIO.");
    gpioTerminate();

    info("Good bye");
    std::this_thread::sleep_for(milliseconds(1000));
    return 0;
}
