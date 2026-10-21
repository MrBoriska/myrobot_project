#ifndef IESKF_SLAM_HPP
#define IESKF_SLAM_HPP

#include <Eigen/Dense>
#include <vector>

#define CL_TARGET_OPENCL_VERSION 200
#include <CL/cl.h>

struct Point2D {
    float x;
    float y;
};

struct ImuData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct State {
    Eigen::Vector3d pos; // x, y, z
    Eigen::Matrix3d rot;
    Eigen::Vector3d vel;
    Eigen::Vector3d bg; // gyro bias
    Eigen::Vector3d ba; // acc bias
};

class IESKF {
public:
    IESKF(const std::string& kernel_source);
    ~IESKF();

    void predict(const ImuData& imu);
    void update(const std::vector<Point2D>& scan);

    State getState() const { return state_; }

private:
    State state_;
    Eigen::Matrix<double, 15, 15> cov_;
    double last_imu_time_ = -1.0;
    Eigen::Vector3d g_;

    std::vector<Point2D> local_map_; // Simple point cloud map

    // OpenCL
    cl_context cl_ctx_;
    cl_command_queue cl_queue_;
    cl_program cl_prog_;
    cl_kernel cl_kernel_;

    // Persistent OpenCL buffers
    cl_mem map_mem_ = nullptr;
    size_t map_mem_capacity_ = 0;

    void initOpenCL(const std::string& kernel_source);
};

#endif // IESKF_SLAM_HPP
