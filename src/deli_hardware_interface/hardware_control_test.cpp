#include "deli_hardware_interface/include/deli_hardware_interface/Constants.h"
#include "deli_hardware_interface/include/deli_hardware_interface/I2CPeripheral.h"
#include "deli_hardware_interface/include/deli_hardware_interface/pca9685_comm.h"

PiPCA9685::PCA9685 pca;

void set_servo_angle(int channel, double angle) {
    int min_pulse = 102; 
    int max_pulse = 512; 

    int pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse);
    
    pca.set_pwm(channel, 0, pulse);
}


int main() {
    pca.set_pwm_freq(50.0);

    set_servo_angle(0,30);
    set_servo_angle(0,90);
}