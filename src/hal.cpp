#include "../include/hal.hpp"

using namespace efp;
using namespace std::chrono;

int now_us()
{
    static auto init_ = steady_clock::now();
    return duration_cast<microseconds>(steady_clock::now() - init_).count();
}

void Encoder::isr_a()
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
    const int enc_b = gpioRead(ENCODER_B_GPIO);

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
    gpioSetMode(ENCODER_B_GPIO, PI_INPUT);

    gpioSetPullUpDown(ENCODER_A_GPIO, PI_PUD_UP);
    gpioSetPullUpDown(ENCODER_B_GPIO, PI_PUD_UP);

    gpioSetISRFunc(ENCODER_A_GPIO, EITHER_EDGE, 0, isr_encoder_a);
    gpioSetISRFunc(ENCODER_B_GPIO, EITHER_EDGE, 0, isr_encoder_b);
}

void isr_encoder_a(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_a();
}

void isr_encoder_b(int gpio, int level, uint32_t tick)
{
    Encoder::instance().isr_b();
}

// DcMotor

DcMotor::~DcMotor()
{
    gpioWrite(MOTOR_0_GPIO, 1);
    gpioWrite(MOTOR_1_GPIO, 1);
}

void DcMotor::operator()(double cmd_v)
{
    const double bound_cmd_v =  bound_v(-3.3, 3.3, cmd_v);
    const int pwm_cmd = (int)(bound_cmd_v * 255. / 3.3);
    trace("DC motor PWM command: {}", pwm_cmd);

    gpioPWM(MOTOR_0_GPIO, pwm_cmd >= 0 ? pwm_cmd : 0);
    gpioPWM(MOTOR_1_GPIO, pwm_cmd >= 0 ? 0 : -pwm_cmd);
}


DcMotor::DcMotor()
{
    gpioSetMode(MOTOR_0_GPIO, PI_OUTPUT);
    gpioSetMode(MOTOR_1_GPIO, PI_OUTPUT);
    gpioWrite(MOTOR_0_GPIO, 1);
    gpioWrite(MOTOR_1_GPIO, 1);
}

void DcMotor::sine_wave(double frequency_hz, double amplitude_v)
{
    const double period_us = 1000000/frequency_hz;
    const int carrier_num = period_us / CARRIER_PERIOD_US;
    const int pulse_num = carrier_num * 2;
    // pulse_num should be smaller than 10000;

    Vector<gpioPulse_t> pulses;
    pulses.resize(pulse_num);

    for_index([&](size_t i)
    { 
        const double sine = sin(2 * M_PI * i / carrier_num);
        const double duty_ratio = bound_v(0., 1., amplitude_v / 3.3) * sine;

        if(duty_ratio >=0){
            pulses[2*i] = gpioPulse_t{1<<MOTOR_0_GPIO, 0, (uint32_t)(duty_ratio * CARRIER_PERIOD_US)}; 
            pulses[2*i + 1] = gpioPulse_t{0, 1<<MOTOR_0_GPIO, (uint32_t)((1 - duty_ratio) * CARRIER_PERIOD_US)}; 
        } 
        else 
        {
            pulses[2*i] = gpioPulse_t{1<<MOTOR_1_GPIO, 0, (uint32_t)(-duty_ratio * CARRIER_PERIOD_US)}; 
            pulses[2*i + 1] = gpioPulse_t{0, 1<<MOTOR_1_GPIO, (uint32_t)(-(1 - duty_ratio) * CARRIER_PERIOD_US)};  
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
