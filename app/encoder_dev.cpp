#include <signal.h>

#include <iostream>
#include <thread>
#include <mutex>
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

void isr_encoder_a(int gpio, int level, uint32_t tick);
void isr_encoder_b(int gpio, int level, uint32_t tick);

class Encoder{
    public: 

    static Encoder & instance()
    {
        static Encoder inst;
        return inst;
    }

    void isr_a()
    {
        const int enc_a = gpioRead(encoder_a_gpio);
        const int enc_b = gpioRead(encoder_b_gpio);

        std::lock_guard<std::mutex> lock(mtx_);

        if (enc_a == 1)
            enc_b == 0 ? ++enc_pos_ : --enc_pos_;
        else
            enc_b == 0 ? --enc_pos_ : ++enc_pos_;

        trace("Encoder pulse count: {}", enc_pos_);
    }

    void isr_b()
    {
        const int enc_a = gpioRead(encoder_a_gpio);
        const int enc_b = gpioRead(encoder_b_gpio);

        std::lock_guard<std::mutex> lock(mtx_);

        if (enc_b == 1)
            enc_a == 0 ? --enc_pos_ : ++enc_pos_;
        else
            enc_a == 0 ? ++enc_pos_ : --enc_pos_;

        trace("Encoder pulse count: {}", enc_pos_);
    }

    int operator()()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return enc_pos_;
    }

    private:
    Encoder()
    {   
        gpioSetMode(encoder_a_gpio, PI_INPUT);
        gpioSetMode(encoder_b_gpio, PI_INPUT);

        gpioSetPullUpDown(encoder_a_gpio, PI_PUD_UP);
        gpioSetPullUpDown(encoder_b_gpio, PI_PUD_UP);

        gpioSetISRFunc(encoder_a_gpio, EITHER_EDGE, 0, isr_encoder_a);
        gpioSetISRFunc(encoder_b_gpio, EITHER_EDGE, 0, isr_encoder_b);
    }

    std::mutex mtx_;
    int enc_pos_;
};

void isr_encoder_a(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_a();
}

void isr_encoder_b(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_b();
}

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

    debug("Setting GPIO.");
    
    auto &encoder = Encoder::instance();

    // Unblocking signals in the main thread
    sigemptyset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Register signal handler for main thread
    signal(SIGINT, signal_handler);

    while(true)
    {
        info("Encoder pulse count: {}", encoder());

        std::this_thread::sleep_for(milliseconds(2000));
    }

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
