#include <iostream>
#include <thread>
#include <chrono>

#include "efp.hpp"
#include "logger.hpp"

int main(){
    using namespace efp;
    using namespace std::chrono;

    info("Starting MEU3014 2nd project...");
    std::this_thread::sleep_for(milliseconds(5));
    return 0;
}
