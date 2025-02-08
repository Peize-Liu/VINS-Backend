#include "gtsam_factor/gtsam_factors.hpp"
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace CustomGTSAMFactors{
    
ProjectionOneFrameTwoCamFactor::ProjectionOneFrameTwoCamFactor(
    const gtsam::SharedNoiseModel& noise_model,
    gtsam::Key pose_i_key, gtsam::Key pose_j_key, 
    gtsam::Key depth_key, gtsam::Key td_key,
    const gtsam::Point3& pts_i, const gtsam::Point3& pts_j,
    const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
    double td, bool use_unit_sphere = false
) : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, double, double>(
        noise_model, pose_i_key, pose_j_key, depth_key, td_key
    ),
    pts_i_(pts_i), pts_j_(pts_j), td_i_(td), td_j_(td),
    use_unit_sphere_(use_unit_sphere) 
{
    // 初始化速度向量（z分量为0）
    velocity_i_ = (gtsam::Vector3() << velocity_i.x(), velocity_i.y(), 0.0).finished();
    velocity_j_ = (gtsam::Vector3() << velocity_j.x(), velocity_j.y(), 0.0).finished();

    // 单位球面误差基向量计算
    if (use_unit_sphere_) {
        const gtsam::Vector3 a = pts_j_.normalized();
        gtsam::Vector3 tmp(0, 0, 1);
        if (a.isApprox(tmp, 1e-6)) tmp << 1, 0, 0;
        
        const gtsam::Vector3 b1 = (tmp - a * a.dot(tmp)).normalized();
        const gtsam::Vector3 b2 = a.cross(b1);
        tangent_base_.row(0) = b1.transpose();
        tangent_base_.row(1) = b2.transpose();
    }
};

gtsam::Vector ProjectionOneFrameTwoCamFactor::evaluateError(
    const gtsam::Pose3& T_cam1_imu,
    const gtsam::Pose3& T_cam2_imu,
    const double& inv_depth,
    const double& td,
    boost::optional<gtsam::Matrix&> H1 = boost::none,  // d(residual)/d(T_cam1_imu)
    boost::optional<gtsam::Matrix&> H2 = boost::none,  // d(residual)/d(T_cam2_imu)
    boost::optional<gtsam::Matrix&> H3 = boost::none,  // d(residual)/d(inv_depth)
    boost::optional<gtsam::Matrix&> H4 = boost::none   // d(residual)/d(td)
){
    using gtsam::Matrix3;
    using gtsam::Matrix23;
    using gtsam::Matrix36;
    using gtsam::Vector3;

    // -------------------- 1. 时间补偿 --------------------
    const gtsam::Point3 pts_i_td = pts_i_ - (td - td_i_) * velocity_i_;
    const gtsam::Point3 pts_j_td = pts_j_ - (td - td_j_) * velocity_j_;

    // -------------------- 2. 坐标变换 --------------------
    gtsam::Point3 p_cam1, p_imu, p_cam2;
    Matrix36 H_transform_from, H_transform_to;

    // 左相机坐标系 -> IMU坐标系
    if (H1 || H3 || H4) {
        p_imu = T_cam1_imu.transformFrom(pts_i_td / inv_depth, H_transform_from);
    } else {
        p_imu = T_cam1_imu.transformFrom(pts_i_td / inv_depth);
    }

    // IMU坐标系 -> 右相机坐标系
    if (H2 || H3 || H4) {
        p_cam2 = T_cam2_imu.transformTo(p_imu, H_transform_to);
    } else {
        p_cam2 = T_cam2_imu.transformTo(p_imu);
    }

    // -------------------- 3. 残差计算 --------------------
    gtsam::Vector2 residual;
    Matrix23 J_proj;

    if (use_unit_sphere_) {
        const double norm = p_cam2.norm();
        const Vector3 p_norm = p_cam2 / norm;
        const Vector3 p_j_norm = pts_j_td.normalized();

        // 计算单位球面误差
        residual = tangent_base_ * (p_norm - p_j_norm);

        // 投影雅可比
        if (H1 || H2 || H3 || H4) {
            const Matrix3 J_norm = (Matrix3::Identity() - p_norm * p_norm.transpose()) / norm;
            J_proj = tangent_base_ * J_norm;
        }
    } else {
        // 平面投影误差
        const double z_inv = 1.0 / p_cam2.z();
        residual = (gtsam::Vector2() << p_cam2.x() * z_inv, p_cam2.y() * z_inv).finished() 
                    - pts_j_td.head<2>();

        // 投影雅可比
        if (H1 || H2 || H3 || H4) {
            J_proj << z_inv, 0.0, -p_cam2.x() * z_inv * z_inv,
                        0.0, z_inv, -p_cam2.y() * z_inv * z_inv;
        }
    }

    // -------------------- 4. 雅可比计算 --------------------
    if (H1 || H2 || H3 || H4) {
        // 4.1 左相机位姿导数 (H1)
        if (H1) {
            *H1 = J_proj * H_transform_from;
        }

        // 4.2 右相机位姿导数 (H2)
        if (H2) {
            *H2 = J_proj * H_transform_to;
        }

        // 4.3 逆深度导数 (H3)
        if (H3) {
            const Vector3 J_depth = -H_transform_from.leftCols<3>() * pts_i_td / (inv_depth * inv_depth);
            *H3 = J_proj * J_depth.head<2>();
        }

        // 4.4 时间偏移导数 (H4)
        if (H4) {
            const Vector3 J_td = H_transform_to.rightCols<3>() * T_cam2_imu.rotation().matrix() * velocity_j_
                                - H_transform_from.leftCols<3>() * T_cam1_imu.rotation().matrix() * velocity_i_ / inv_depth;
            *H4 = J_proj * J_td.head<2>();
        }
    }
    return residual;
};
} // namespace CustomGTSAMFactors