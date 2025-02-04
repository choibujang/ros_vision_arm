#ifndef DELIARM_H
#define DELIARM_H

#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "../include/pca9685_comm.h"

class DeliArm {
public:
    void setGoalJoint(int num, double angle) { goal_joints[num] = angle; };
    std::vector<double> getGoalJoint() { return goal_joints; }
    void writeRasp(std::vector<double> joints);

    void home();
    void calcIK(double target_x, double target_y, double target_z);
    void moveToPosition();

private:
    std::vector<double> links = {100, 104, 293}; // mm, 손목1 103 110
    // 0번 조인트: 10 앞에서봤을때 왼쪽 90 중간 170 오른쪽
    // 1번 조인트: 170 앞 90 중간 10 뒤
    // 2번 조인트: 10 앞 90 중간 170 뒤
    // 3번 조인트: 170 앞 90 중간 10 뒤
    // 4번 조인트: 120 수평
    // 5번 조인트: 작을수록 손가락 펴짐
    std::vector<double> joints = {0,0,0,0,0,0};
    std::vector<double> goal_joints = {0,0,0,170.0,120.0,100.0};
    std::vector<double> base_joints = {90.0,50.0,40.0,170.0,100.0,90.0};

    PiPCA9685::PCA9685 pca;
};
#endif 