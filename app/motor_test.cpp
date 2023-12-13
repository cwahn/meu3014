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

constexpr int motor_0_gpio = 17;
constexpr int motor_1_gpio = 27;

void signal_handler(int signum) {
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
    if(gpioInitialise() < 0)
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

    while(true)
    {
        const double voltage_cmd = now_us()/1000000 * 0.1;
        motor(voltage_cmd);
        info("Voltage command: {} V", voltage_cmd);

        std::this_thread::sleep_for(milliseconds(1000));
    }

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
