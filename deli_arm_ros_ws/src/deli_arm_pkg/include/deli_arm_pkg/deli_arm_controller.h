#ifndef DELIARM_H
#define DELIARM_H

#include <cmath>
#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "pca9685_comm.h"

class DeliArmController {
public:
    DeliArmController() {
        pca.set_pwm_freq(50);

        int min_pulse = 102; 
        int max_pulse = 512; 

        int duration = 2000;
        int interval = 20;
        int num_steps = duration / interval; 

        int target_pulse;

        target_pulse = min_pulse + (95 / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(0, 0, target_pulse);
        current_joints[0] = 95;   

        target_pulse = min_pulse + (close_gripper / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(5, 0, target_pulse);
        current_joints[5] = close_gripper;

        target_pulse = min_pulse + (fixed_joint4 / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(4, 0, target_pulse);
        current_joints[4] = fixed_joint4;

        target_pulse = min_pulse + (neat_321joints[3] / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(3, 0, target_pulse);
        current_joints[3] = neat_321joints[3];

        target_pulse = min_pulse + (neat_321joints[2] / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(2, 0, target_pulse);
        current_joints[2] = neat_321joints[2];       

        target_pulse = min_pulse + (neat_321joints[1] / 180.0) * (max_pulse - min_pulse);
        pca.set_pwm(1, 0, target_pulse);
        current_joints[1] = neat_321joints[1];       

        std::cout << "Deli arm initialized" << std::endl;
    }

    double convertJoint(int joint_num, double ik_angle) { 
        double converted_angle;
        if (joint_num == 0) {
            converted_angle = ik_angle + joints_map[joint_num];
        } else if (joint_num == 1) {
            converted_angle = -1 * (ik_angle - joints_map[joint_num]);
        } else if (joint_num == 2) {
            converted_angle = ik_angle + joints_map[joint_num];
        } else if (joint_num == 3) {
            converted_angle = -1 * (ik_angle - joints_map[joint_num]);
        } else {
            std::cout << "invalid joint_num" << std::endl;
            return std::nan("");
        }

        if (converted_angle < 10.0 || converted_angle > 170.0) {
            std::cout << "invalid angle" << std::endl;
            return std::nan("");
        } else {
            return converted_angle;
        }
    }

    std::vector<double> calcIK(std::vector<double> pick_target_pos);
    void writeRasp(int joint_num, double angle);
    void move321JointsToNeatPos();
    void openGripper() { writeRasp(5, open_gripper); }
    void closeGripper() { writeRasp(5, close_gripper); }
    void moveBaseLink(double angle) { writeRasp(0, angle); }
    void move321Joints(std::vector<double> goal_joints);

private:
    std::vector<double> links = {100, 104, 293}; // mm, 손목1 103 110
    // 0번 조인트: 10 앞에서봤을때 왼쪽 90 중간 170 오른쪽
    // 1번 조인트: 170 앞 90 중간 10 뒤
    // 2번 조인트: 10 앞 90 중간 170 뒤
    // 3번 조인트: 170 앞 90 중간 10 뒤
    // 4번 조인트: 120 수평
    // 5번 조인트: open: 100, close: 80
    std::map<int, double> joints_map = {
        {0, 95},    // 앞 x, 오른쪽 y, 위 z. 10이 -y 쪽
        {1, 180},   // 앞 y, 위 x. 170일 때 y축과 평행
        {2, 90},      // 90일 때 1번 joint와 일직선, 10이 아래쪽
        {3, 90}     // 90일 때 2번 joint와 일직선
    };

    std::vector<double> current_joints = {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")};
    std::map<int, double> neat_321joints = {
        {1, 50},
        {2, 30},
        {3, 160}
    };

    double fixed_joint4 = 120.0;
    double open_gripper = 90.0;
    double close_gripper = 106.0;

    PiPCA9685::PCA9685 pca;
};
#endif 