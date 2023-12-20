#ifndef MEU3014_HAL_HPP_
#define MEU3014_HAL_HPP_

#include <thread>
#include <mutex>
#include <fmt/core.h>
#include <cstdio>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

#define ENCODER_A_GPIO 23
#define ENCODER_B_GPIO 24

#define MOTOR_0_GPIO 17
#define MOTOR_1_GPIO 27

#define CARRIER_PERIOD_US 20

// now

int now_us();

// periodic_while
template <typename Duration, typename F, typename G>
void periodic_while(Duration period, const F &keep, const G &task)
{
    using Clock = std::chrono::steady_clock;

    const auto start_timepoint = Clock::now();
    auto next_timepoint = start_timepoint;

    while (true)
    {
        auto now = Clock::now();
        const auto elapsed_time = now - start_timepoint;

        if (!keep(elapsed_time))
            break;

        task(elapsed_time);

        next_timepoint += period;

        std::this_thread::sleep_until(next_timepoint);
        while (now < next_timepoint)
        {
        }
    }
}

// Encoder

void isr_encoder_a(int gpio, int level, uint32_t tick);
void isr_encoder_b(int gpio, int level, uint32_t tick);

class Encoder
{
public:
    static Encoder &instance()
    {
        static Encoder inst;
        return inst;
    }

    void isr_a();
    void isr_b();
    int operator()();

private:
    Encoder();

    std::mutex mtx_;
    int enc_pos_;
};

class DcMotor
{
public:
    ~DcMotor();

    static DcMotor &instance()
    {
        static DcMotor motor{};
        return motor;
    }

    void operator()(double cmd_v);

    void sine_wave(double frequency_hz, double amplitude_v);

private:
    DcMotor();
};

template <typename Sequence>
void print_csv(const std::string &filename, const Sequence &seq)
{
    std::FILE *file = std::fopen(filename.c_str(), "w");

    if (file)
    {
        for (size_t i = 0; i < seq.size(); ++i)
        {
            if (i > 0)
            {
                fmt::print(file, ",");
            }
            fmt::print(file, "{}", seq[i]);
        }
        fmt::print(file, "\n");
        std::fclose(file);
    }
    else
    {
        fmt::print(stderr, "Failed to open the file: {}\n", filename);
    }
}

#endif