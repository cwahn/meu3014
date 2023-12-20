#include <signal.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

#include "hal.hpp"
#include "common.hpp"

using namespace efp;
using namespace std::chrono;

constexpr microseconds play_time = microseconds(10000000); 
constexpr microseconds cycle_time = microseconds(1000); 

template <typename Duration>
class PIDController {
public:
    // Cycle time will be converted as seconds
    PIDController(Duration cycle_time, double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), 
          cycle_time_s_(std::chrono::duration_cast<microseconds>(cycle_time).count()/1000000.),
          integral_(0.0), previous_error_(0.0) {}

    double operator()(double error) {
        integral_ += error * cycle_time_s_;
        double derivative = (error - previous_error_) / cycle_time_s_;
        previous_error_ = error;

        const double p_term = kp_ * error;
        const double i_term = ki_ * integral_;
        const double d_term = kd_ * derivative;

        trace("error: {} i_error: {}, d_error: {}", error, integral_, derivative);
        trace("p_term: {} i_term: {}, d_term: {}", p_term, i_term, d_term);
        
        return p_term + i_term + d_term;
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

template<size_t n>
class StaticFrictionCompensator
{
public:
    StaticFrictionCompensator(double static_friction_v)
    : static_friction_v_(static_friction_v)
    {
    }

    double operator()(int pos, double cmd_v)
    {
        poss_.push_back(pos);
        if (cmd_v > 0.1 || cmd_v < -0.1)
        {
            auto rel_poss = map([&](int p)
            { return p - poss_[0]; }, poss_);

            if(sum(rel_poss) == 0) // stop
            {
                trace("Compensating static friction");
                return cmd_v > 0 ? cmd_v + static_friction_v_ : cmd_v - static_friction_v_;
            }
        }

        return cmd_v;
    }

private:
    double static_friction_v_;
    Vcb<int, n> poss_;
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

    FILE * log_file = fopen("./record_and_play_pid.log", "w");

    // Spawn threads
    Logger::set_log_level(LogLevel::Trace);
    Logger::set_output(log_file);

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

    Vector<int> ref_poss{};
    ref_poss.resize(play_time.count() / cycle_time.count());
    
    info("Recording reference position");
    motor(0);
    periodic_while(
        cycle_time,
        [](microseconds loop_time)
        { return loop_time < play_time; },
        [&](microseconds loop_time)
        { 
            const int idx = loop_time / cycle_time;
            ref_poss[idx] = encoder();
        }
    );

    // todo correction

    StaticFrictionCompensator<5> sf_comp{0.75};
    PIDController<microseconds> pid{cycle_time, 0.1, 0, 0};

    info("Play back ref position");

    const int current_pos = encoder();
    const int ref_pos_0 = ref_poss[0];

    const auto shifted_ref_poss = map([&](double p)
    { return p - ref_pos_0 + current_pos; }, ref_poss);

    Vector<int> play_back_poss{};
    play_back_poss.resize(play_time.count() / cycle_time.count());

    periodic_while(
        cycle_time,
        [](microseconds loop_time)
        { return loop_time < play_time; },
        [&](microseconds loop_time)
        { 
            const int idx = loop_time / cycle_time;
            const int ref_pos = shifted_ref_poss[idx];
            const int pos = encoder();
            const int error = pos - ref_pos;
            // motor(sf_comp(pos, pid(error)));
            motor(pid(error));
            play_back_poss[idx] = encoder();
        }
    );

    info("ITAE: {}", itae(cycle_time, shifted_ref_poss, play_back_poss)/237);
    
    debug("Terminatig PIGPIO.");
    gpioTerminate();

    info("Good bye");
    std::this_thread::sleep_for(milliseconds(1000)); 
    return 0;
}
