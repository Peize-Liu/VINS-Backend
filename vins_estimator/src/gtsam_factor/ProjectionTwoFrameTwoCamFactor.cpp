#include "gtsam_factor/gtsam_factors.hpp"
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace CustomGTSAMFactors{
ProjectionTwoFrameTwoCamFactor::ProjectionTwoFrameTwoCamFactor(const gtsam::SharedNoiseModel& noise_model,
    gtsam::Key pose_i_key, gtsam::Key pose_j_key,
    gtsam::Key ex1_key, gtsam::Key ex2_key,
    gtsam::Key inv_depth_key, gtsam::Key td_key,
    const gtsam::Vector3& pts_i, const gtsam::Vector3& pts_j,
    const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
    double td_i, double td_j, const gtsam::Matrix2& sqrt_info,
    bool unit_sphere): gtsam::NoiseModelFactor6<gtsam::Pose3,gtsam::Pose3,gtsam::Pose3,gtsam::Pose3, double,double>(noise_model,
        {pose_i_key, pose_j_key, ex1_key, ex2_key, inv_depth_key, td_key}),
    pts_i_(pts_i), pts_j_(pts_j),
    velocity_i_(gtsam::Vector3(velocity_i.x(), velocity_i.y(), 0)),
    velocity_j_(gtsam::Vector3(velocity_j.x(), velocity_j.y(), 0)),
    td_i_(td_i), td_j_(td_j), sqrt_info_(sqrt_info),
    unit_sphere_(unit_sphere) {
    if (unit_sphere_) {
        gtsam::Vector3 a = pts_j_.normalized();
        gtsam::Vector3 tmp(0, 0, 1);
        if (a.isApprox(tmp, 1e-6)) tmp << 1, 0, 0;
        gtsam::Vector3 b1 = (tmp - a * a.dot(tmp)).normalized();
        gtsam::Vector3 b2 = a.cross(b1);
        tangent_base_.row(0) = b1.transpose();
        tangent_base_.row(1) = b2.transpose();
    }
};

gtsam::Vector ProjectionTwoFrameTwoCamFactor::evaluateError(
    const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
    const gtsam::Pose3& ex1, const gtsam::Pose3& ex2,
    const double& inv_depth, const double& td,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3,
    boost::optional<gtsam::Matrix&> H4,
    boost::optional<gtsam::Matrix&> H5,
    boost::optional<gtsam::Matrix&> H6) const {

// 时间补偿
gtsam::Vector3 pts_i_td = pts_i_ - (td - td_i_) * velocity_i_;
gtsam::Vector3 pts_j_td = pts_j_ - (td - td_j_) * velocity_j_;

// 坐标变换链 ---------------------------------------------------
// 1. 归一化相机坐标 (归一化平面 + 逆深度)
gtsam::Point3 pts_camera_i = pts_i_td / inv_depth;

// 2. 相机i -> IMUi (外参)
gtsam::Matrix36 D_ex1;
gtsam::Point3 pts_imu_i = ex1.transformFrom(pts_camera_i, H3 ? &D_ex1 : 0);

// 3. IMUi -> 世界系 (位姿i)
gtsam::Matrix36 D_pose_i;
gtsam::Point3 pts_w = pose_i.transformFrom(pts_imu_i, H1 ? &D_pose_i : 0);

// 4. 世界系 -> IMUj (位姿j)
gtsam::Matrix36 D_pose_j;
gtsam::Point3 pts_imu_j = pose_j.transformTo(pts_w, H2 ? &D_pose_j : 0);

// 5. IMUj -> 相机j (外参)
gtsam::Matrix36 D_ex2;
gtsam::Point3 pts_camera_j = ex2.transformTo(pts_imu_j, H4 ? &D_ex2 : 0);

// 计算残差 ----------------------------------------------------
gtsam::Vector2 residual;
Eigen::Map<gtsam::Vector2> residual_map(residual.data());

if (unit_sphere_) {
    gtsam::Vector3 norm_j = pts_camera_j.normalized();
    residual_map = tangent_base_ * (norm_j - pts_j_td.normalized());
} else {
    double dep_j = pts_camera_j.z();
    residual_map.head<2>() = (pts_camera_j.head<2>()/dep_j) - pts_j_td.head<2>();
}

// 应用信息矩阵
residual = sqrt_info_ * residual;

// 雅可比计算 --------------------------------------------------
if (H1 || H2 || H3 || H4 || H5 || H6) {
    gtsam::Matrix23 reduce;
    if (unit_sphere_) {
    double norm = pts_camera_j.norm();
    gtsam::Matrix33 norm_jaco = (gtsam::Matrix33::Identity() - 
        pts_camera_j * pts_camera_j.transpose() / (norm * norm)) / norm;
    reduce = tangent_base_ * norm_jaco;
    } else {
    double dep_j = pts_camera_j.z();
    reduce << 1.0/dep_j, 0, -pts_camera_j.x()/(dep_j*dep_j),
                0, 1.0/dep_j, -pts_camera_j.y()/(dep_j*dep_j);
    }
    reduce = sqrt_info_ * reduce;

    // 各参数导数链式法则
    if (H1) {  // pose_i
    gtsam::Matrix36 D_pts_w_pose_i = D_pose_i;
    gtsam::Matrix26 J = reduce * ex2.rotation().transpose() * 
                        pose_j.rotation().transpose() * D_pts_w_pose_i;
    *H1 = J;
    }

    if (H2) {  // pose_j
    gtsam::Matrix36 D_pts_imu_j_pose_j = D_pose_j;
    gtsam::Matrix26 J = reduce * ex2.rotation().transpose() * D_pts_imu_j_pose_j;
    *H2 = J;
    }

    if (H3) {  // ex1
    gtsam::Matrix36 D_pts_imu_i_ex1 = D_ex1;
    gtsam::Matrix26 J = reduce * ex2.rotation().transpose() * 
                        pose_j.rotation().transpose() * 
                        pose_i.rotation().matrix() * D_pts_imu_i_ex1;
    *H3 = J;
    }

    if (H4) {  // ex2
    gtsam::Matrix36 D_pts_camera_j_ex2 = D_ex2;
    *H4 = reduce * D_pts_camera_j_ex2;
    }

    if (H5) {  // inv_depth
    gtsam::Vector3 d_pts_camera_i = -pts_i_td / (inv_depth * inv_depth);
    gtsam::Vector3 d_pts_imu_i = ex1.rotation().matrix() * d_pts_camera_i;
    gtsam::Vector3 d_pts_w = pose_i.rotation().matrix() * d_pts_imu_i;
    gtsam::Vector3 d_pts_imu_j = pose_j.rotation().transpose() * d_pts_w;
    gtsam::Vector3 d_pts_camera_j = ex2.rotation().transpose() * d_pts_imu_j;
    
    gtsam::Vector2 J = (reduce * d_pts_camera_j).head<2>();
    *H5 = J;
    }

    if (H6) {  // td
    gtsam::Vector3 d_pts_i_td = -velocity_i_;
    gtsam::Vector3 d_pts_j_td = velocity_j_;
    
    gtsam::Vector3 d_pts_camera_j = ex2.rotation().transpose() * 
        pose_j.rotation().transpose() * pose_i.rotation().matrix() * 
        ex1.rotation().matrix() * (d_pts_i_td / inv_depth);
    
    gtsam::Vector2 J = (reduce * d_pts_camera_j).head<2>()
     + sqrt_info_ * (d_pts_j_td.head<2>()); //TODO: check is jacobian
    *H6 = J;
    }
}
return residual;
};

}// namespace CustomGTSAMFactors