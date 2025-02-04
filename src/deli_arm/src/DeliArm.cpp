void DeliArm::home() {
    // home_angles = [90, 90, 90, 150, ?, ?]
    // ready_angles = [90, 50, 40, 170, ?, ?]
    writeRasp({90,50,40,170,90,50});
}


void DeliArm::calcJoints(double x, double y, double z) {
    if (y == 0.0) {
        if (x <= 0.0) {
            setJoint(0, 180.0);
        } else {
            setJoint(0, 0.0);
        }
    } else {
        setJoint(0, (90-(std::atan(x / y) * (180.0 / M_PI))));
    }
}

void DeliArm::writeRasp(std::vector<double>& joints) {
    int min_pulse = 102; 
    int max_pulse = 512; 

    int duration = 1000;
    int interval = 20;
    int num_steps = duration / interval; 

    for (int i = 0; i < joints.size(); i++) {
        int current_pulse = pca.get_pwm(i);
        int target_pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);

        for (int step = 0; step <= num_steps; step++) {
            float t = (float)step / num_steps;
            float ease_ratio = t * t * (3 - 2 * t);
            int smooth_pulse = current_pulse + (target_pulse - current_pulse) * ease_ratio;
            pca.set_pwm(i, 0, smooth_pulse);
            delay(interval);
        }
    }
}

void DeliArm::moveToPosition() {


    for (int i = 0; i < joints.size(); i++) {
        int pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);
        int target_pulse = min_pulse + (joints[i] / 180.0) * (max_pulse - min_pulse);
        
    
        pca.set_pwm(i, 0, pulse);
    }

}