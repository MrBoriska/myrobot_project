#include "myrobot_custom_slam/ieskf_slam.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

std::string readKernel() {
    std::ifstream file("src/myrobot_custom_slam/kernels/scan_matching.cl");
    if (!file.is_open()) {
        std::cerr << "Could not open kernel file." << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

int main() {
    std::cout << "Starting IESKF Test..." << std::endl;

    std::string kernel_src = readKernel();
    if(kernel_src.empty()) return 1;

    IESKF slam(kernel_src);

    // Simulate IMU data
    for (int i = 0; i < 100; ++i) {
        ImuData imu;
        imu.timestamp = i * 0.01; // 100 Hz
        imu.acc = Eigen::Vector3d(0.1, 0, 9.81); // slight forward acceleration
        imu.gyro = Eigen::Vector3d(0, 0, 0.1); // slight rotation
        slam.predict(imu);
    }

    State st = slam.getState();
    std::cout << "State after predict: " << st.pos.transpose() << std::endl;

    // Simulate Lidar scan
    std::vector<Point2D> scan;
    for (int i = 0; i < 360; ++i) {
        float angle = i * M_PI / 180.0;
        scan.push_back({5.0f * std::cos(angle), 5.0f * std::sin(angle)});
    }

    slam.update(scan);

    st = slam.getState();
    std::cout << "State after update: " << st.pos.transpose() << std::endl;

    if (std::isnan(st.pos.x()) || std::isnan(st.pos.y()) || std::isnan(st.pos.z())) {
        std::cerr << "Test failed: NaN in state." << std::endl;
        return 1;
    }

    std::cout << "Test passed!" << std::endl;
    return 0;
}
