#include "../include/DeliArm.h"

void DeliArm::home() {
    // home_angles = [90, 90, 90, 150, ?, ?]
    // ready_angles = [90, 50, 40, 170, ?, ?]
    writeRasp(base_joints);
}


void DeliArm::calcIK(double x, double y, double z) {
    std::cout << "(x, y, z) = " << x << ", " << y << ", " << z << std::endl;
    if (y == 0.0) {
        if (x <= 0.0) {
            std::cout << "invalid location" << std::endl;
            // goal_joint = {};
            return;
        } else {
            setGoalJoint(0, 90.0);
        }
    } else {
        double theta0 = std::atan2(y, x) * (180.0 / M_PI);
        std::cout << "theta0: " << theta0 << std::endl;
        setGoalJoint(0, theta0);
    }

    double px = sqrt(pow(x, 2) + pow(y, 2));
    double py = z - links[0];

    double numerator = pow(links[1] + links[2], 2) - (pow(px, 2) + pow(py, 2));
    std::cout << "numerator: " << numerator << std::endl;
    double denominator = (pow(px, 2) + pow(py, 2)) - (pow(links[1] - links[2], 2));
    std::cout << "denominator: " << denominator << std::endl;

    double theta2_positive = 2 * atan2(sqrt(numerator), sqrt(denominator));
    double theta2_negative = -2 * atan2(sqrt(numerator), sqrt(denominator));
    std::cout << "theta2_positive: " << theta2_positive << std::endl;
    std::cout << "theta2_negative: " << theta2_negative << std::endl;

    double term1 = atan2(py, px);
    std::cout << "term1: " << term1 << std::endl;
    double term2 = atan2(links[2] * sin(theta2_positive), links[1] + links[2] * cos(theta2_positive));
    std::cout << "term2: " << term2 << std::endl;

    double theta1 = term1 - term2;
    std::cout << "theta1: " << theta1 << std::endl;

    setGoalJoint(1, 180 - theta1);
    setGoalJoint(2, 90 - theta2_positive);
}

void DeliArm::writeRasp(std::vector<double> joints) {
    int min_pulse = 102; 
    int max_pulse = 512; 

    int duration = 6000;
    int interval = 40;
    int num_steps = duration / interval; 

    std::vector<double> prev_joints = base_joints;
    std::ifstream file_in("joints_data.txt");
    if (file_in.is_open()) {
        std::string line;
        std::getline(file_in, line);
        std::istringstream iss(line);
        for (int i = 0; i < 6 && iss >> prev_joints[i]; i++);
        file_in.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

    for (int i = 0; i < joints.size(); i++) {
        int current_pulse = pca.get_pwm(i);
        if (current_pulse == 0) {
            current_pulse = min_pulse + (prev_joints[i] / 180.0) * (max_pulse - min_pulse);
        }
        int target_pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);

        for (int step = 0; step <= num_steps; step++) {
            float t = (float)step / num_steps;
            float ease_ratio = t * t * (3 - 2 * t);
            int smooth_pulse = current_pulse + (target_pulse - current_pulse) * ease_ratio;
            pca.set_pwm(i, 0, smooth_pulse);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));  // 1000ms (1초) 대기

        }
    }

    std::ofstream file("joints_data.txt");
    if (file.is_open()) {
        for (double joint : joints) {
            file << joint << " ";
        }
        file << std::endl;
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

}

void DeliArm::moveToPosition() {


    // for (int i = 0; i < joints.size(); i++) {
    //     int pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);
    //     int target_pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);
        
    
    //     pca.set_pwm(i, 0, pulse);
    // }

}