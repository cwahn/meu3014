#include <iostream>
#include <thread>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

int main()
{
    using namespace efp;
    using namespace std::chrono;

    Logger::set_log_level(LogLevel::Trace);

    info("Starting MEU3014 2nd project.");

    debug("Initializing PIGPIO.");
    if(gpioInitialise() < 0)
    {        
        fatal("Failed to initialize PIGPIO.");
        return -1;
    }

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
