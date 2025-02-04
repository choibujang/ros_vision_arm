#include "deli_arm/include/pca9685_comm.h"
#include <iostream>

int main() {
    PCA9685 pca;

    int current_pulse = pca.get_pwm(0);

    std::cout << current_pulse << std::endl;
}