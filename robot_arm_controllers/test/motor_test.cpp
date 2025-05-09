#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include "robot_arm_controllers/arm/pca9685_comm.h"

using namespace PiPCA9685;


int main() {
    PCA9685 pwm("/dev/i2c-1", 0x40);
    pwm.set_pwm_freq(50.0);

    int channel;
    double ms;
    std::vector<int> channels;
    std::vector<double> pulses;

    std::cout << "Enter channel and ms pairs (e.g., '0 1.0'), then Ctrl+D to run:\n";

    while (std::cin >> channel >> ms) {
        channels.push_back(channel);
        pulses.push_back(ms);
    }

    for (size_t i = 0; i < channels.size(); ++i) {
        std::cout << "Moving ch " << channels[i] << " servo to " << pulses[i] << "ms..." << std::endl;
        pwm.set_pwm_ms(channels[i], pulses[i]);
    }


    std::cout << "Integration test complete." << std::endl;
    return 0;
}
