#ifndef MEU3014_HAL_HPP_
#define MEU3014_HAL_HPP_

#include <thread>
#include <mutex>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

#define ENCODER_A_GPIO 23
#define ENCODER_B_GPIO 24

#define MOTOR_0_GPIO 17
#define MOTOR_1_GPIO 27

// now

int now_us();

// Encoder

void isr_encoder_a(int gpio, int level, uint32_t tick);
void isr_encoder_b(int gpio, int level, uint32_t tick);

class Encoder{
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

private:
    DcMotor();
};

#endif