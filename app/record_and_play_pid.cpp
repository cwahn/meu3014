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

constexpr microseconds play_time = microseconds(5000000); // 5 sec
constexpr microseconds cycle_time = microseconds(1000); // 1 ms

template <typename Duration>
class PIDController {
public:
    // Cycle time will be converted as seconds
    PIDController(Duration cycle_time, double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), 
          cycle_time_s_(std::chrono::duration_cast<microseconds>(cycle_time)/1000000.),
          integral_(0.0), previous_error_(0.0) {}

    double operator()(double error) {
        integral_ += error * cycle_time_;
        double derivative = (error - previous_error_) / cycle_time_;
        previous_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

private:
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain
    double cycle_time_s_; // Time interval between each PID compute call

    double integral_; // Integral sum
    double previous_error_; // Previous error value for derivative term
};

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

    info("Recording reference position");
    Vector<int> ref_poss{};
    ref_poss.resize(play_time.count() / cycle_time.count());

    periodic_while(
        cycle_time,
        [](milliseconds loop_time)
        { return loop_time < play_time; },
        [](milliseconds loop_time)
        { 
            const int idx = loop_time / cycle_time;
            ref_poss[idx] = encoder();
        }
    )

    // todo correction

    PIDController pid{cycle_time, 1, 0, 0};

    info("Play back ref position")
    periodic_while(
        cycle_time,
        [](milliseconds loop_time)
        { return loop_time < play_time; },
        [](milliseconds loop_time)
        { 
            const int ref_pos = ref_poss[loop_time / cycle_time];
            const int error = encoder() - ref_pos;
            motor(pid(error));
        }
    )
    
    debug("Terminatig PIGPIO.");
    gpioTerminate();

    info("Good bye");
    std::this_thread::sleep_for(milliseconds(1000)); 
    return 0;
}
