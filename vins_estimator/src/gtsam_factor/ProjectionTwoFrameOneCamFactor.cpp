#include "gtsam_factor/gtsam_factors.hpp"
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace CustomGTSAMFactors{
using Base = gtsam::NoiseModelFactor;

  /**
   * @brief 构造函数
   * @param pose_i_key 第i帧位姿的Key
   * @param pose_j_key 第j帧位姿的Key
   * @param ex_key     相机外参的Key
   * @param inv_depth_key 逆深度Key  
   * @param td_key     时间偏移Key
   * @param pts_i      第i帧归一化相机坐标
   * @param pts_j      第j帧归一化相机坐标
   * @param velocity_i 第i帧特征速度
   * @param velocity_j 第j帧特征速度
   * @param td_i       第i帧时间偏移
   * @param td_j       第j帧时间偏移
   * @param sqrt_info  信息矩阵平方根
   * @param unit_sphere 是否使用单位球面误差
   */
  ProjectionTwoFrameOneCamFactor::ProjectionTwoFrameOneCamFactor(
      gtsam::Key pose_i_key, gtsam::Key pose_j_key,
      gtsam::Key ex_key, gtsam::Key inv_depth_key, gtsam::Key td_key,
      const gtsam::Point3& pts_i, const gtsam::Point3& pts_j,
      const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
      double td_i, double td_j, const gtsam::Matrix2& sqrt_info,
      bool unit_sphere = false)
      : Base(gtsam::noiseModel::Gaussian::SqrtInformation(sqrt_info), 
            {pose_i_key, pose_j_key, ex_key, inv_depth_key, td_key}),
        pts_i_(pts_i), pts_j_(pts_j),
        velocity_i_(gtsam::Vector3(velocity_i.x(), velocity_i.y(), 0)),
        velocity_j_(gtsam::Vector3(velocity_j.x(), velocity_j.y(), 0)),
        td_i_(td_i), td_j_(td_j), sqrt_info_(sqrt_info),
        unit_sphere_(unit_sphere) {

    // 单位球面误差的正切基初始化
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

gtsam::Vector ProjectionTwoFrameOneCamFactor::evaluateError(
    const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
    const gtsam::Pose3& ex, const double& inv_depth, const double& td,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none){

// ================== 1. 时间补偿 ==================
gtsam::Vector3 pts_i_td = pts_i_ - (td - td_i_) * velocity_i_;
gtsam::Vector3 pts_j_td = pts_j_ - (td - td_j_) * velocity_j_;

// ================== 2. 坐标变换链 ==================
// 2.1 相机i系 -> IMUi系
gtsam::Matrix36 D_ex_imu_i;
gtsam::Point3 pts_camera_i = pts_i_td / inv_depth;
gtsam::Point3 pts_imu_i = ex.transformFrom(pts_camera_i, H3 ? &D_ex_imu_i : 0);

// 2.2 IMUi系 -> 世界系
gtsam::Matrix36 D_pose_i_world;
gtsam::Point3 pts_world = pose_i.transformFrom(pts_imu_i, H1 ? &D_pose_i_world : 0);

// 2.3 世界系 -> IMUj系
gtsam::Matrix36 D_pose_j_imu;
gtsam::Point3 pts_imu_j = pose_j.transformTo(pts_world, H2 ? &D_pose_j_imu : 0);

// 2.4 IMUj系 -> 相机j系
gtsam::Matrix36 D_ex_cam_j;
gtsam::Point3 pts_camera_j = ex.transformTo(pts_imu_j, H3 ? &D_ex_cam_j : 0);

// ================== 3. 残差计算 ==================
gtsam::Vector2 residual;
if (unit_sphere_) {
    gtsam::Vector3 norm_j = pts_camera_j.normalized();
    residual = tangent_base_ * (norm_j - pts_j_td.normalized());
} else {
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j.head<2>()/dep_j) - pts_j_td.head<2>();
}
residual = sqrt_info_ * residual; // 白化残差

// ================== 4. 雅可比计算 ==================
    if (H1 || H2 || H3 || H4 || H5) {
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

        // 4.1 位姿i的雅可比 (H1)
        if (H1) {
        gtsam::Matrix36 J = -reduce * ex.rotation().transpose() * 
                            pose_j.rotation().transpose() * D_pose_i_world;
        *H1 = J;
        }

        // 4.2 位姿j的雅可比 (H2)
        if (H2) {
        gtsam::Matrix36 J = reduce * ex.rotation().transpose() * D_pose_j_imu;
        *H2 = J;
        }

        // 4.3 外参的雅可比 (H3)
        if (H3) {
        gtsam::Matrix36 J_ex = reduce * (D_ex_cam_j - ex.rotation().transpose() * 
            pose_j.rotation().transpose() * pose_i.rotation().matrix() * D_ex_imu_i);
        *H3 = J_ex;
        }

        // 4.4 逆深度的雅可比 (H4)
        if (H4) {
        gtsam::Vector3 d_pts_camera_i = -pts_i_td / (inv_depth * inv_depth);
        gtsam::Vector3 d_pts_imu_i = ex.rotation().matrix() * d_pts_camera_i;
        gtsam::Vector3 d_pts_world = pose_i.rotation().matrix() * d_pts_imu_i;
        gtsam::Vector3 d_pts_imu_j = pose_j.rotation().transpose() * d_pts_world;
        gtsam::Vector3 d_pts_camera_j = ex.rotation().transpose() * d_pts_imu_j;
        *H4 = reduce * d_pts_camera_j.head<2>();
        }

        // 4.5 时间偏移的雅可比 (H5)
        if (H5) {
        gtsam::Vector3 d_pts_i_td = -velocity_i_;
        gtsam::Vector3 d_pts_j_td = velocity_j_;
        
        gtsam::Vector3 d_pts_camera_j = ex.rotation().transpose() * 
            pose_j.rotation().transpose() * pose_i.rotation().matrix() * 
            ex.rotation().matrix() * (d_pts_i_td / inv_depth);
        
        *H5 = reduce * d_pts_camera_j.head<2>() + 
                sqrt_info_ * d_pts_j_td.head<2>();
        }
    }
    return residual;
};

}// namespace CustomGTSAMFactors
