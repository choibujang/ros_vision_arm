#include "../include/pca9685_comm.h"
#include <iostream>
#include <unistd.h>

int main() {
    PiPCA9685::PCA9685 pca;

    pca.set_pwm_freq(50);

    int min_pulse = 102; 
    int max_pulse = 512; 

    int current_pulse = pca.get_pwm(0);

    std::cout << current_pulse << std::endl;

    int target_pulse = min_pulse + (45.0 / 180.0) * (max_pulse - min_pulse);
    
    pca.set_pwm(0, 0, target_pulse);

    current_pulse = pca.get_pwm(0);

    std::cout << current_pulse << std::endl;

    target_pulse = min_pulse + (90.0 / 180.0) * (max_pulse - min_pulse);

    sleep(1);

    pca.set_pwm(0, 0, target_pulse);

    current_pulse = pca.get_pwm(0);

    std::cout << current_pulse << std::endl;
}