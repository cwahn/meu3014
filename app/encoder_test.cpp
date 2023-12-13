#include <signal.h>

#include <iostream>
#include <thread>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

using namespace efp;
using namespace std::chrono;

constexpr int encoder_a_gpio = 23;
constexpr int encoder_b_gpio = 24;

// For debouncing
int encoder_pulse_count = 0;
int a_level = 0;
int b_level = 0;

void signal_handler(int signum) {
    info("Signal {} received in main thread", signum);

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    info("Terminating program.");
    exit(signum);
}

void isr_encoder_a(int gpio, int level, uint32_t tick)
{
    trace("ISR for encoder a called.");
    const int a_level = gpioRead(encoder_a_gpio);
    const int b_level = gpioRead(encoder_b_gpio);

    if (a_level == PI_HIGH)
	{
		if (b_level == PI_LOW)
			encoder_pulse_count++; 
		else
			encoder_pulse_count--;
	}
	else
	{
		if (b_level == PI_LOW)
			encoder_pulse_count--;
		else
			encoder_pulse_count++;
	}

    info("Encoder pulse count: {}", encoder_pulse_count);
}

void isr_encoder_b(int gpio, int level, uint32_t tick)
{
    trace("ISR for encoder b called.");
    const int a_level = gpioRead(encoder_a_gpio);
    const int b_level = gpioRead(encoder_b_gpio);

    if (b_level == PI_HIGH)
	{
		if (a_level == PI_LOW)
			encoder_pulse_count--; 
		else
			encoder_pulse_count++;
	}
	else
	{
		if (a_level == PI_LOW)
			encoder_pulse_count++;
		else
			encoder_pulse_count--;
	}

    info("Encoder pulse count: {}", encoder_pulse_count);
}


int main()
{
    // Block all signals for this thread (main thread)
    sigset_t set;
    sigfillset(&set);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    // Spawn threads

    // Unblocking signals in the main thread
    sigemptyset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Register signal handler for main thread
    signal(SIGINT, signal_handler);

    Logger::set_log_level(LogLevel::Trace);

    info("Starting MEU3014 2nd project.");

    debug("Initializing PIGPIO.");
    if(gpioInitialise() < 0)
    {        
        fatal("Failed to initialize PIGPIO.");
        return -1;
    }

    debug("Setting GPIO.");
    gpioSetMode(encoder_a_gpio, PI_INPUT);
    gpioSetMode(encoder_b_gpio, PI_INPUT);

    gpioSetPullUpDown(encoder_a_gpio, PI_PUD_UP);
    gpioSetPullUpDown(encoder_b_gpio, PI_PUD_UP);

    gpioSetISRFunc(encoder_a_gpio, EITHER_EDGE, 0, isr_encoder_a);
    gpioSetISRFunc(encoder_b_gpio, EITHER_EDGE, 0, isr_encoder_b);

    while(true)
    {
        info("Encoder pulse count: {}", encoder_pulse_count);

        std::this_thread::sleep_for(milliseconds(2000));
    }

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
