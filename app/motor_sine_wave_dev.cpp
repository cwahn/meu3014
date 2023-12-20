#include <signal.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

// #include "hal.hpp"

using namespace efp;
using namespace std::chrono;

constexpr int motor_0_gpio = 17;
constexpr int motor_1_gpio = 27;

constexpr int carrier_period_us = 20;


class DcMotor
{
public:
    ~DcMotor()
    {
        gpioWrite(motor_0_gpio, 1);
        gpioWrite(motor_1_gpio, 1);
    }

    static DcMotor &instance()
    {
        static DcMotor motor{};
        return motor;
    }

    void operator()(double cmd_v)
    {
        const double bound_cmd_v =  bound_v(-3.3, 3.3, cmd_v);
        const int pwm_cmd = (int)(bound_cmd_v * 255. / 3.3);
        debug("DC motor PWM command: {}", pwm_cmd);

        gpioPWM(motor_0_gpio, pwm_cmd >= 0 ? pwm_cmd : 0);
        gpioPWM(motor_1_gpio, pwm_cmd >= 0 ? 0 : -pwm_cmd);
    }

    void sine_wave(double frequency_hz, double amplitude_v)
    {
        const double period_us = 1000000/frequency_hz;
        const int carrier_num = period_us / carrier_period_us;
        const int pulse_num = carrier_num * 2;
        // pulse_num should be smaller than 10000;

        Vector<gpioPulse_t> pulses;
        pulses.resize(pulse_num);

        for_index([&](size_t i)
        { 
            const double sine = sin(2 * M_PI * i / carrier_num);
            const double duty_ratio = bound_v(0., 1., amplitude_v / 3.3) * sine;

            if(duty_ratio >=0){
                pulses[2*i] = gpioPulse_t{1<<motor_0_gpio, 0, (uint32_t)(duty_ratio * carrier_period_us)}; 
                pulses[2*i + 1] = gpioPulse_t{0, 1<<motor_0_gpio, (uint32_t)((1 - duty_ratio) * carrier_period_us)}; 
            } 
            else 
            {
                pulses[2*i] = gpioPulse_t{1<<motor_1_gpio, 0, (uint32_t)(-duty_ratio * carrier_period_us)}; 
                pulses[2*i + 1] = gpioPulse_t{0, 1<<motor_1_gpio, (uint32_t)(-(1 - duty_ratio) * carrier_period_us)};  
            }
        }, 
        carrier_num);

        gpioWaveClear();
        gpioWaveAddGeneric(length(pulses), pulses.data());
        const int wave_id = gpioWaveCreate();

        if(wave_id >= 0)
        {
            gpioWaveTxStop();
            gpioWaveTxSend(wave_id, PI_WAVE_MODE_REPEAT);
        }
        else 
        {
            warn("PIGPIO wave form generation failed with return {}", wave_id);
        }
    }

private:
    DcMotor()
    {
        gpioSetMode(motor_0_gpio, PI_OUTPUT);
        gpioSetMode(motor_1_gpio, PI_OUTPUT);
        gpioWrite(motor_0_gpio, 1);
        gpioWrite(motor_1_gpio, 1);
    }
};

// HAL input

int now_us()
{
    static auto init_ = steady_clock::now();
    return duration_cast<microseconds>(steady_clock::now() - init_).count();
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

    debug("Setting HALs.");
    // auto &encoder = Encoder::instance();
    auto &motor = DcMotor::instance();

    // Unblocking signals in the main thread
    sigemptyset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Register signal handler for main thread
    signal(SIGINT, signal_handler);

    while(true)
    {
        // const double voltage_cmd = now_us()/1000000 * 0.1;
        // motor(voltage_cmd);

        // info("Voltage command: {} V", voltage_cmd);

        // 50 %
        motor.sine_wave(600, 1.5);

        // gpioWrite(motor_0_gpio, 1);
        // gpioWrite(motor_1_gpio, 0);

        std::this_thread::sleep_for(milliseconds(100));
    }

    debug("Terminatig PIGPIO.");
    gpioTerminate();

    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
