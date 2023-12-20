#ifndef MEU3014_COMMON_HPP_
#define MEU3014_COMMON_HPP_

#include <chrono>

#include "efp.hpp"

template <typename Duration, typename As, typename Bs>
double itae(Duration dt, const As & refs, const Bs& datas)
{
    using namespace efp;

    const double dt_s = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.;

    const auto dt_ss = from_function(length(refs), [&](size_t i)
    { return i * dt_s ; });

    return sum(map([&](double t, double r, double d)
    { return t * abs(d - r); }, dt_ss, refs, datas));
}

#endif