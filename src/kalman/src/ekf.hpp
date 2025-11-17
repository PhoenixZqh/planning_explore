#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

inline Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q)
{
    Eigen::Vector3d rpy;
    double          sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double          cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    rpy[0]                    = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    // 检查奇异性
    if (std::abs(sinp) >= 1)
        rpy[1] = std::copysign(M_PI / 2.0, sinp);
    else
        rpy[1] = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    rpy[2]           = std::atan2(siny_cosp, cosy_cosp);

    return rpy;  // [Roll, Pitch, Yaw]
}

class EKF
{
private:
    /**
     * @brief 确保状态向量中的四元数归一化
     */
    void normalizeQuaternionState()
    {
        // x = [px, py, pz, qx, qy, qz, qw, vx, vy, vz]^T
        // 四元数在 x[3] 到 x[6]
        Eigen::Quaterniond q_est(x_(6), x_(3), x_(4), x_(5));  // w, x, y, z
        q_est.normalize();
        x_(3) = q_est.x();
        x_(4) = q_est.y();
        x_(5) = q_est.z();
        x_(6) = q_est.w();
    }

public:
    // 状态量维度：10
    static constexpr int N = 10;
    // 观测值维度：7 (Odometry: px, py, pz, qx, qy, qz, qw)
    static constexpr int M = 7;

    Eigen::Matrix<double, N, 1> x_;
    Eigen::Matrix<double, N, N> P_;
    Eigen::Matrix<double, N, N> Q_;
    Eigen::Matrix<double, M, M> R_;

    /**
     * @brief 构造函数
     */
    EKF()
    {
        x_.setZero();

        P_.setIdentity();
        P_ *= 1.0;

        // --- 优化后的 Q (过程噪声) ---
        Q_.setIdentity();
        Q_.block<3, 3>(0, 0) *= 0.001;   // Position (p)
        Q_.block<4, 4>(3, 3) *= 0.0001;  // Quaternion (q) 保持较低，假设缓慢变化
        Q_.block<3, 3>(7, 7) *= 0.005;   // Velocity (v) 略微增大，允许速度变化

        // --- 优化后的 R (测量噪声) ---
        R_.setIdentity();
        R_.block<3, 3>(0, 0) *= 0.01;  // Position measurement noise
        R_.block<4, 4>(3, 3) *= 0.1;   // Quaternion measurement noise
    }

    /**
     * @brief 预测步骤
     * * @param dt 时间间隔
     */
    void predict(double dt)
    {
        // --- 1. 计算 F_k ---
        Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
        F.block<3, 3>(0, 7)           = Eigen::Matrix3d::Identity() * dt;

        // --- 2. 状态预测 ---
        // p_k+1 = p_k + v_k * dt
        x_.head<3>() = x_.head<3>() + x_.tail<3>() * dt;
        // q_k+1 = q_k, v_k+1 = v_k

        normalizeQuaternionState();

        // --- 3. 协方差预测 ---
        P_ = F * P_ * F.transpose() + Q_;
    }

    /**
     * @brief 更新步骤
     * * @param z 观测向量 [px, py, pz, qx, qy, qz, qw]
     */
    void update(const Eigen::Matrix<double, M, 1>& z_raw)
    {
        // --- 0. 归一化测量四元数 ---
        Eigen::Quaterniond q_meas(z_raw(6), z_raw(3), z_raw(4), z_raw(5));
        q_meas.normalize();

        // --- 1. 当前状态四元数 ---
        Eigen::Quaterniond q_est(x_(6), x_(3), x_(4), x_(5));  // w, x, y, z
        q_est.normalize();

        // --- 2. 计算四元数误差：δq = q_meas * q_est.conjugate() ---
        Eigen::Quaterniond q_error = q_meas * q_est.conjugate();

        // 确保误差四元数 w >= 0（避免符号翻转）
        if (q_error.w() < 0)
        {
            q_error.coeffs() = -q_error.coeffs();
        }

        // --- 3. 将四元数误差转为旋转向量（李代数 so(3)）---
        Eigen::Vector3d delta_theta;
        double          w   = q_error.w();
        Eigen::Vector3d xyz = q_error.vec();

        if (w >= 1.0)
        {
            delta_theta.setZero();  // 几乎无旋转
        }
        else
        {
            double theta = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));
            if (std::abs(theta) < 1e-6)
            {
                delta_theta = xyz;
            }
            else
            {
                delta_theta = theta * xyz / std::sin(theta / 2.0);
            }
        }

        // --- 4. 构造观测残差 y（6维：位置 + 旋转误差）---
        Eigen::Matrix<double, 6, 1> y;
        y.head<3>() = z_raw.head<3>() - x_.head<3>();  // 位置残差
        y.tail<3>() = delta_theta;                     // 姿态残差（so(3)）

        // --- 5. 观测雅可比 H (6 x 10) ---
        Eigen::Matrix<double, 6, 10> H = Eigen::Matrix<double, 6, 10>::Zero();
        H.block<3, 3>(0, 0)            = Eigen::Matrix3d::Identity();  // dp/dp
        H.block<3, 4>(3, 3)            = Eigen::Matrix<double, 3, 4>::Zero();
        H.block<3, 3>(3, 3).setIdentity();  // d(delta_theta)/d(qx,qy,qz,qw) ≈ I (近似)

        // --- 6. 测量噪声 R (6x6) ---
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
        R.block<3, 3>(0, 0) *= 0.01;  // 位置噪声
        R.block<3, 3>(3, 3) *= 0.05;  // 旋转误差噪声（可调）

        // --- 7. 卡尔曼增益 ---
        Eigen::Matrix<double, 6, 6>  S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, 10, 6> K = P_ * H.transpose() * S.inverse();

        // --- 8. 状态更新 ---
        Eigen::Matrix<double, 10, 1> delta_x = K * y;

        // 位置和速度直接加
        x_.head<3>() += delta_x.head<3>();
        x_.tail<3>() += delta_x.tail<3>();

        // 四元数：指数映射更新 q = q ⊕ δθ
        Eigen::Vector3d    delta_theta_update = delta_x.segment<3>(3);
        double             theta              = delta_theta_update.norm();
        Eigen::Quaterniond delta_q;
        if (theta < 1e-8)
        {
            delta_q = Eigen::Quaterniond(1, 0, 0, 0);
        }
        else
        {
            Eigen::Vector3d axis       = delta_theta_update / theta;
            double          half_theta = theta / 2.0;
            delta_q =
                Eigen::Quaterniond(std::cos(half_theta), std::sin(half_theta) * axis.x(), std::sin(half_theta) * axis.y(), std::sin(half_theta) * axis.z());
        }
        Eigen::Quaterniond q_new = delta_q * q_est;
        q_new.normalize();

        x_(3) = q_new.x();
        x_(4) = q_new.y();
        x_(5) = q_new.z();
        x_(6) = q_new.w();

        // --- 9. 协方差更新 ---
        Eigen::Matrix<double, 10, 10> I = Eigen::Matrix<double, 10, 10>::Identity();
        P_                              = (I - K * H) * P_;
    }

    const Eigen::Matrix<double, N, 1>& getState() const
    {
        return x_;
    }

    void setInitialState(const Eigen::Vector3d& p_init, const Eigen::Quaterniond& q_init)
    {
        x_.setZero();
        x_.head<3>() = p_init;
        x_(3)        = q_init.x();
        x_(4)        = q_init.y();
        x_(5)        = q_init.z();
        x_(6)        = q_init.w();
        x_.tail<3>().setZero();
        normalizeQuaternionState();  // 确保初始状态归一化
    }
};