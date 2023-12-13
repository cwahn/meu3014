#ifndef MEU3014_PIGPIO_ENCODER_HPP_
#define MEU3014_PIGPIO_ENCODER_HPP_

#include <thread>
#include <mutex>

#include "efp.hpp"
#include "logger.hpp"
#include "pigpio.h"

#define ENCODER_A_GPIO 23;
#define ENCODER_B_GPIO 24;

void isr_encoder_a(int gpio, int level, uint32_t tick);
void isr_encoder_b(int gpio, int level, uint32_t tick);

class Encoder{
public: 
    static Encoder & instance();
    void isr_a();
    void isr_b();
    int operator()();

 private:
    Encoder();

    std::mutex mtx_;
    int enc_pos_;
};

#endif