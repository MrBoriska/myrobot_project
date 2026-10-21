#include "myrobot_custom_slam/ieskf_slam.hpp"
#include <iostream>
#include <cmath>

IESKF::IESKF(const std::string& kernel_source) {
    state_.pos = Eigen::Vector3d::Zero();
    state_.rot = Eigen::Matrix3d::Identity();
    state_.vel = Eigen::Vector3d::Zero();
    state_.bg = Eigen::Vector3d::Zero();
    state_.ba = Eigen::Vector3d::Zero();
    cov_ = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;
    g_ = Eigen::Vector3d(0, 0, -9.81);

    initOpenCL(kernel_source);
}

IESKF::~IESKF() {
    if (cl_kernel_) clReleaseKernel(cl_kernel_);
    if (cl_prog_) clReleaseProgram(cl_prog_);
    if (cl_queue_) clReleaseCommandQueue(cl_queue_);
    if (cl_ctx_) clReleaseContext(cl_ctx_);
}

void IESKF::initOpenCL(const std::string& kernel_source) {
    cl_uint num_platforms;
    clGetPlatformIDs(0, nullptr, &num_platforms);
    if (num_platforms == 0) {
        std::cerr << "No OpenCL platforms found." << std::endl;
        return;
    }

    std::vector<cl_platform_id> platforms(num_platforms);
    clGetPlatformIDs(num_platforms, platforms.data(), nullptr);

    cl_device_id device_id = nullptr;
    for (auto plat : platforms) {
        cl_uint num_devices;
        clGetDeviceIDs(plat, CL_DEVICE_TYPE_ALL, 0, nullptr, &num_devices);
        if (num_devices > 0) {
            std::vector<cl_device_id> devices(num_devices);
            clGetDeviceIDs(plat, CL_DEVICE_TYPE_ALL, num_devices, devices.data(), nullptr);
            device_id = devices[0];
            break;
        }
    }

    if (!device_id) {
        std::cerr << "No OpenCL devices found." << std::endl;
        return;
    }

    cl_int err;
    cl_ctx_ = clCreateContext(nullptr, 1, &device_id, nullptr, nullptr, &err);
    cl_queue_ = clCreateCommandQueue(cl_ctx_, device_id, 0, &err);

    const char* src = kernel_source.c_str();
    cl_prog_ = clCreateProgramWithSource(cl_ctx_, 1, &src, nullptr, &err);
    clBuildProgram(cl_prog_, 1, &device_id, nullptr, nullptr, nullptr);

    cl_kernel_ = clCreateKernel(cl_prog_, "scan_match", &err);
}

static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

void IESKF::predict(const ImuData& imu) {
    if (last_imu_time_ < 0) {
        last_imu_time_ = imu.timestamp;
        return;
    }

    double dt = imu.timestamp - last_imu_time_;
    last_imu_time_ = imu.timestamp;

    Eigen::Vector3d acc_unbias = imu.acc - state_.ba;
    Eigen::Vector3d gyro_unbias = imu.gyro - state_.bg;

    // Simple Euler integration
    state_.pos += state_.vel * dt + 0.5 * (state_.rot * acc_unbias + g_) * dt * dt;
    state_.vel += (state_.rot * acc_unbias + g_) * dt;

    Eigen::Vector3d w = gyro_unbias * dt;
    double theta = w.norm();
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if (theta > 1e-6) {
        Eigen::Vector3d w_hat = w / theta;
        Eigen::Matrix3d K = skewSymmetric(w_hat);
        dR = Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * K * K;
    }
    state_.rot = state_.rot * dR;

    // Simplified error-state covariance propagation
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3,3>(0,6) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(6,3) = -state_.rot * skewSymmetric(acc_unbias) * dt;
    cov_ = F * cov_ * F.transpose(); // Add noise matrix Q here in a full implementation
}

void IESKF::update(const std::vector<Point2D>& scan) {
    if (!cl_kernel_ || scan.empty()) return;

    if (local_map_.empty()) {
        // Initialize map with first scan
        float initial_yaw = std::atan2(state_.rot(1,0), state_.rot(0,0));
        float cos_yaw = std::cos(initial_yaw);
        float sin_yaw = std::sin(initial_yaw);
        for(const auto& pt : scan) {
            local_map_.push_back({
                (float)state_.pos.x() + pt.x * cos_yaw - pt.y * sin_yaw,
                (float)state_.pos.y() + pt.x * sin_yaw + pt.y * cos_yaw
            });
        }
        return;
    }

    // --- Iterated Error-State Kalman Filter (simplified 1-iteration) ---
    size_t num_pts = scan.size();
    size_t map_size = local_map_.size();

    cl_int err;

    // Allocate host memory buffers (Mali g31 optimized zero-copy)
    Point2D* mapped_scan = (Point2D*)clEnqueueMapBuffer(cl_queue_,
        clCreateBuffer(cl_ctx_, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, sizeof(Point2D) * num_pts, nullptr, &err),
        CL_TRUE, CL_MAP_WRITE, 0, sizeof(Point2D) * num_pts, 0, nullptr, nullptr, &err);
    std::copy(scan.begin(), scan.end(), mapped_scan);
    cl_mem scan_mem = clCreateBuffer(cl_ctx_, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(Point2D) * num_pts, mapped_scan, &err);

    std::vector<float> errors(num_pts, 0.0f);
    std::vector<float> grad_x(num_pts, 0.0f);
    std::vector<float> grad_y(num_pts, 0.0f);

    cl_mem err_mem = clCreateBuffer(cl_ctx_, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * num_pts, errors.data(), &err);
    cl_mem grad_x_mem = clCreateBuffer(cl_ctx_, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * num_pts, grad_x.data(), &err);
    cl_mem grad_y_mem = clCreateBuffer(cl_ctx_, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * num_pts, grad_y.data(), &err);

    // Re-allocate map buffer if size increased (persistent mapping)
    if (map_size > map_mem_capacity_) {
        if (map_mem_) clReleaseMemObject(map_mem_);
        map_mem_capacity_ = map_size + 1000; // Overallocate to avoid frequent realloc
        map_mem_ = clCreateBuffer(cl_ctx_, CL_MEM_READ_ONLY, sizeof(Point2D) * map_mem_capacity_, nullptr, &err);
    }
    clEnqueueWriteBuffer(cl_queue_, map_mem_, CL_FALSE, 0, sizeof(Point2D) * map_size, local_map_.data(), 0, nullptr, nullptr);

    float yaw = std::atan2(state_.rot(1,0), state_.rot(0,0));
    float pose[3] = {(float)state_.pos.x(), (float)state_.pos.y(), yaw};
    cl_mem pose_mem = clCreateBuffer(cl_ctx_, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 3, pose, &err);

    clSetKernelArg(cl_kernel_, 0, sizeof(cl_mem), &scan_mem);
    clSetKernelArg(cl_kernel_, 1, sizeof(cl_mem), &err_mem);
    clSetKernelArg(cl_kernel_, 2, sizeof(cl_mem), &grad_x_mem);
    clSetKernelArg(cl_kernel_, 3, sizeof(cl_mem), &grad_y_mem);
    clSetKernelArg(cl_kernel_, 4, sizeof(cl_mem), &pose_mem);
    clSetKernelArg(cl_kernel_, 5, sizeof(cl_mem), &map_mem_);
    int m_size = map_size;
    clSetKernelArg(cl_kernel_, 6, sizeof(int), &m_size);

    size_t global_work_size = num_pts;
    clEnqueueNDRangeKernel(cl_queue_, cl_kernel_, 1, nullptr, &global_work_size, nullptr, 0, nullptr, nullptr);
    clFinish(cl_queue_); // Wait for completion since we use CL_MEM_USE_HOST_PTR mapped memory

    clReleaseMemObject(scan_mem);
    clReleaseMemObject(err_mem);
    clReleaseMemObject(grad_x_mem);
    clReleaseMemObject(grad_y_mem);
    clReleaseMemObject(pose_mem);

    // Compute Jacobians H and residual
    Eigen::MatrixXd H(num_pts, 3);
    Eigen::VectorXd res(num_pts);

    for (size_t i = 0; i < num_pts; ++i) {
        float lx = scan[i].x;
        float ly = scan[i].y;

        // Jacobian wrt yaw: (-lx*sin - ly*cos, lx*cos - ly*sin)
        float dx_dyaw = -lx * std::sin(yaw) - ly * std::cos(yaw);
        float dy_dyaw =  lx * std::cos(yaw) - ly * std::sin(yaw);

        H(i, 0) = grad_x[i];
        H(i, 1) = grad_y[i];
        H(i, 2) = grad_x[i] * dx_dyaw + grad_y[i] * dy_dyaw;
        res(i) = errors[i];
    }

    // Kalman update step: delta_x = (H^T R^-1 H + P^-1)^-1 H^T R^-1 res
    Eigen::Matrix3d HTH = H.transpose() * H;

    // Simplified covariance representation (P^-1) mapped to 3D space [x, y, yaw]
    Eigen::Matrix3d P_inv = Eigen::Matrix3d::Identity() * 0.1;
    Eigen::Vector3d delta = (HTH + P_inv).inverse() * H.transpose() * res;

    // Apply error-state correction to nominal state
    state_.pos.x() -= delta(0);
    state_.pos.y() -= delta(1);

    double new_yaw = yaw - delta(2);
    Eigen::Matrix3d dR_z;
    dR_z << std::cos(-delta(2)), -std::sin(-delta(2)), 0,
            std::sin(-delta(2)),  std::cos(-delta(2)), 0,
            0, 0, 1;

    // Correct orientation while preserving roll/pitch from IMU
    state_.rot = state_.rot * dR_z;

    // --- Mapping Phase ---
    // Update local map with newly registered points (simple accumulation for demonstration)
    if (local_map_.size() < 10000) { // Limit map size to prevent memory explosion in this basic implementation
        float c_yaw = std::cos(new_yaw);
        float s_yaw = std::sin(new_yaw);
        for(size_t i = 0; i < num_pts; ++i) {
            // Only add points that were far from existing map points to avoid density explosion
            if(errors[i] > 0.1f) {
                local_map_.push_back({
                    (float)state_.pos.x() + scan[i].x * c_yaw - scan[i].y * s_yaw,
                    (float)state_.pos.y() + scan[i].x * s_yaw + scan[i].y * c_yaw
                });
            }
        }
    }
}
