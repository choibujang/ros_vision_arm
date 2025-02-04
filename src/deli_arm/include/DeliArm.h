#ifndef DELIARM_H
#define DELIARM_H

#include "deli_arm/pca9685.h"
#include <cmath>

class DeliArm {
    void setJointGoal(int num, double angle) { joints[num] = angle; };

    void writeRasp(std::vector<double>& joints);

    void home();
    void calcIK(double target_x, double target_y, double target_z);
    void moveToPosition();

private:
    std::vector<double> links = {};
    std::vector<double> joints = {};
    std::vector<double> real_joints = {};

    PiPCA9685::PCA9685 pca;
};
#endif 