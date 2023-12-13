#include "../include/encoder.hpp"

using namespace efp;
using namespace std::chrono;

static Encoder & instance()
{
    static Encoder inst;
    return inst;
}

void isr_a()
{
    const int enc_a = gpioRead(ENCODER_A_GPIO);
    const int enc_b = gpioRead(ENCODER_B_GPIO);

    std::lock_guard<std::mutex> lock(mtx_);

    if (enc_a == 1)
        enc_b == 0 ? ++enc_pos_ : --enc_pos_;
    else
        enc_b == 0 ? --enc_pos_ : ++enc_pos_;

    trace("Encoder pulse count: {}", enc_pos_);
}

void Encoder::isr_b()
{
    const int enc_a = gpioRead(ENCODER_A_GPIO);
    const int enc_b = gpioRead(encoder_b_gpio);

    std::lock_guard<std::mutex> lock(mtx_);

    if (enc_b == 1)
        enc_a == 0 ? --enc_pos_ : ++enc_pos_;
    else
        enc_a == 0 ? ++enc_pos_ : --enc_pos_;

    trace("Encoder pulse count: {}", enc_pos_);
}

int Encoder::operator()()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return enc_pos_;
}

    Encoder::Encoder()
{   
    gpioSetMode(ENCODER_A_GPIO, PI_INPUT);
    gpioSetMode(encoder_b_gpio, PI_INPUT);

    gpioSetPullUpDown(ENCODER_A_GPIO, PI_PUD_UP);
    gpioSetPullUpDown(encoder_b_gpio, PI_PUD_UP);

    gpioSetISRFunc(ENCODER_A_GPIO, EITHER_EDGE, 0, isr_encoder_a);
    gpioSetISRFunc(encoder_b_gpio, EITHER_EDGE, 0, isr_encoder_b);
}

void isr_encoder_a(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_a();
}

void isr_encoder_b(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_b();
}