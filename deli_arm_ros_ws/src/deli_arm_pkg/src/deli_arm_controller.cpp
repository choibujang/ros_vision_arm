#include "deli_arm_pkg/deli_arm_controller.h"

std::vector<double> DeliArmController::calcIK(std::vector<double> pick_target_pos) {
    double target_x = pick_target_pos[0];
    double target_y = pick_target_pos[1];
    double target_z = pick_target_pos[2];
    std::vector<double> ik_result = {
        std::nan(""),
        std::nan(""),
        std::nan(""),
        std::nan("")
    };
    std::cout << "(target_x, target_y, target_z) = " << target_x << ", " << target_y << ", " << target_z << std::endl;
    if (target_y == 0.0) {
        if (target_x <= 0.0) {
            std::cout << "invalid location" << std::endl;
            return {};
        } else {
            ik_result[0] = convertJoint(0, 0.0);
        }
    } else {
        double theta0 = std::atan2(target_y, target_x) * (180.0 / M_PI);
        std::cout << "theta0: " << theta0 << std::endl;
        ik_result[0] = convertJoint(0, theta0);
    }

    double px = sqrt(pow(target_x, 2) + pow(target_y, 2));
    double py = target_z - links[0];

    std::cout << "px: " << px << std::endl;
    std::cout << "py: " << py << std::endl;
    

    std::cout << "(link1 + link2) ^ 2: " << pow((links[1] + links[2]), 2) << std::endl;
    std::cout << "(px^2) + (py^2): " << (pow(px, 2) + pow(py, 2)) << std::endl;
    double numerator = pow((links[1] + links[2]), 2) - (pow(px, 2) + pow(py, 2));
    std::cout << "numerator: " << numerator << std::endl;
    double denominator = (pow(px, 2) + pow(py, 2)) - (pow(links[1] - links[2], 2));
    std::cout << "denominator: " << denominator << std::endl;

    double theta2_positive = 2 * atan2(sqrt(numerator), sqrt(denominator));
    double theta2_negative = -2 * atan2(sqrt(numerator), sqrt(denominator));
    std::cout << "theta2_positive: " << theta2_positive << std::endl;
    std::cout << "theta2_negative: " << theta2_negative << std::endl;

    double term1 = atan2(py, px);
    std::cout << "term1: " << term1 << std::endl;
    double term2 = atan2(links[2] * sin(theta2_negative), links[1] + links[2] * cos(theta2_negative));
    std::cout << "term2: " << term2 << std::endl;

    double theta1 = term1 - term2;
    std::cout << "theta1: " << theta1 << std::endl;

    theta1 = theta1 * (180.0 / M_PI);
    theta2_negative = theta2_negative * (180.0 / M_PI);

    ik_result[1] = convertJoint(1, theta1);
    ik_result[2] = convertJoint(2, (theta2_negative / 3) * 2);
    ik_result[3] = convertJoint(3, theta2_negative / 3);

    return ik_result;
}

// void DeliArmController::writeRasp(int joint_num, double angle) {
//     int min_pulse = 102; 
//     int max_pulse = 512; 

//     int duration = 2000;
//     int interval = 20;
//     int num_steps = duration / interval; 

//     int current_pulse = pca.get_pwm(joint_num);
//     int target_pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse);

//     for (int step = 0; step <= num_steps; step++) {
//         float t = (float)step / num_steps;
//         float ease_ratio = t * t * (3 - 2 * t);
//         int smooth_pulse = current_pulse + (target_pulse - current_pulse) * ease_ratio;
//         pca.set_pwm(joint_num, 0, smooth_pulse);
//         std::this_thread::sleep_for(std::chrono::milliseconds(interval));  // 1000ms (1초) 대기
//     }
    
// }

void DeliArmController::writeRasp(int joint_num, double angle) {
    int min_pulse = 102; 
    int max_pulse = 512; 

    int base_duration_per_degree = 50;
    int interval = 20;

    int current_pulse = pca.get_pwm(joint_num);
    int target_pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse);
    
    int angle_delta = abs((target_pulse - current_pulse) * 180.0 / (max_pulse - min_pulse));
    int duration = angle_delta * base_duration_per_degree; 
    int num_steps = std::max(1, duration / interval); 

    for (int step = 0; step <= num_steps; step++) {
        float t = (float)step / num_steps;
        float ease_ratio = t * t * (3 - 2 * t);
        int smooth_pulse = current_pulse + (target_pulse - current_pulse) * ease_ratio;
        pca.set_pwm(joint_num, 0, smooth_pulse);
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));  
    }
}


void DeliArmController::move321JointsToNeatPos() {
    std::cout << "!move321JointsToNeatPos entered!" << std::endl;
    for (const auto& pair : neat_321joints) {
        writeRasp(pair.first, pair.second);
    }
    for (int i = 0; i < 6; i++) {
        std::cout << "joint " << i << " pwm: " << pca.get_pwm(i) << std::endl;
    }
}

void DeliArmController::move321JointsToCameraPos() {
    std::cout << "!move321JointsToCameraPos entered!" << std::endl;
    for (const auto& pair : camera_321joints) {
        writeRasp(pair.first, pair.second);
    }
    for (int i = 0; i < 6; i++) {
        std::cout << "joint " << i << " pwm: " << pca.get_pwm(i) << std::endl;
    }
}


void DeliArmController::move321Joints(std::vector<double> goal_joints) {
    std::cout << "!move321Joints entered!" << std::endl;
    for (int i = goal_joints.size()-1; i > 0; i--) {
        std::cout << i << ": " << goal_joints[i] << ", ";
        writeRasp(i, goal_joints[i]);
    }
    std::cout << std::endl;
    for (int i = 0; i < 6; i++) {
        std::cout << "joint " << i << " pwm: " << pca.get_pwm(i) << std::endl;
    }
}
