#ifndef GTSAM_FACTORS_HPP
#define GTSAM_FACTORS_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace CustomGTSAMFactors{

// Stereo extrinsic parameter factor
class ProjectionOneFrameTwoCamFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, double, double> {
public:
    ProjectionOneFrameTwoCamFactor(
        const gtsam::SharedNoiseModel& noise_model,
        gtsam::Key pose_i_key, gtsam::Key pose_j_key, 
        gtsam::Key depth_key, gtsam::Key td_key,
        const gtsam::Point3& pts_i, const gtsam::Point3& pts_j,
        const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
        double td, const gtsam:: Matrix2 & sqrt_info, bool use_unit_sphere = false
    );
        
    gtsam::Vector evaluateError(
        const gtsam::Pose3& T_cam1_imu,
        const gtsam::Pose3& T_cam2_imu,
        const double& inv_depth,
        const double& td,
        boost::optional<gtsam::Matrix&> H1 = boost::none,  // d(residual)/d(T_cam1_imu)
        boost::optional<gtsam::Matrix&> H2 = boost::none,  // d(residual)/d(T_cam2_imu)
        boost::optional<gtsam::Matrix&> H3 = boost::none,  // d(residual)/d(inv_depth)
        boost::optional<gtsam::Matrix&> H4 = boost::none   // d(residual)/d(td)
    )const override ;
private:
    gtsam::Point3 pts_i_;          // 左相机归一化坐标
    gtsam::Point3 pts_j_;          // 右相机归一化坐标
    gtsam::Vector3 velocity_i_;    // 左相机特征速度
    gtsam::Vector3 velocity_j_;   // 右相机特征速度
    double td_i_;                  // 左相机时间偏移
    double td_j_;                  // 右相机时间偏移
    gtsam::Matrix23 tangent_base_; // 单位球面投影基向量
    bool use_unit_sphere_;         // 是否使用单位球面误差
    gtsam::Matrix2 sqrt_info_;     // 信息矩阵平方根
};

// Stereo frame project to two camera factor
class ProjectionTwoFrameTwoCamFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, double, double> {
public:
  using Base = gtsam::NoiseModelFactor;
  ProjectionTwoFrameTwoCamFactor(const gtsam::SharedNoiseModel& noise_model,
      gtsam::Key pose_i_key, gtsam::Key pose_j_key,
      gtsam::Key ex1_key, gtsam::Key ex2_key,
      gtsam::Key inv_depth_key, gtsam::Key td_key,
      const gtsam::Vector3& pts_i, const gtsam::Vector3& pts_j,
      const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
      double curt_td, const gtsam::Matrix2& sqrt_info,
      bool unit_sphere = false);

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
      const gtsam::Pose3& ex1, const gtsam::Pose3& ex2,
      const double& inv_depth, const double& td,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none,
      boost::optional<gtsam::Matrix&> H6 = boost::none) const override;
private:
  gtsam::Vector3 pts_i_, pts_j_;
  gtsam::Vector3 velocity_i_, velocity_j_;
  double td_i_, td_j_;
  gtsam::Matrix2 sqrt_info_;
  bool unit_sphere_;
  gtsam::Matrix23 tangent_base_;  // 单位球面误差的正切基
};

// Mono frame project to one camera factor
class ProjectionTwoFrameOneCamFactor : public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, double, double> {
public:
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
  ProjectionTwoFrameOneCamFactor(
    const gtsam::SharedNoiseModel& noise_model,
    gtsam::Key pose_i_key, gtsam::Key pose_j_key,
    gtsam::Key ex_key, gtsam::Key inv_depth_key, gtsam::Key td_key,
    const gtsam::Point3& pts_i, const gtsam::Point3& pts_j,
    const gtsam::Vector2& velocity_i, const gtsam::Vector2& velocity_j,
    double cur_td, const gtsam::Matrix2& sqrt_info,
    bool unit_sphere = false);

  gtsam::Vector evaluateError(
    const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
    const gtsam::Pose3& ex, const double& inv_depth, const double& td,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none) const override;

private:
  gtsam::Point3 pts_i_, pts_j_;
  gtsam::Vector3 velocity_i_, velocity_j_;
  double td_i_, td_j_;
  gtsam::Matrix2 sqrt_info_;
  bool unit_sphere_;
  gtsam::Matrix23 tangent_base_; // 单位球面误差正切基
};

class CustomIMUPreintergration: public gtsam::PreintegratedImuMeasurements{
public:
  CustomIMUPreintergration(boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params,const gtsam::imuBias::ConstantBias& bias):gtsam::PreintegratedImuMeasurements(params, bias){
  };

  void preinteragtion(const gtsam::Vector3& linear_acceleration, const gtsam::Vector3& angular_velocity, double delta_t){
    gtsam::PreintegratedImuMeasurements::integrateMeasurement(linear_acceleration, angular_velocity, delta_t);
    // printf("[IMU preintegration] linear_acceleration: %f %f %f, angular_velocity: %f %f %f, delta_t: %f\n", linear_acceleration(0), linear_acceleration(1), linear_acceleration(2), angular_velocity(0), angular_velocity(1), angular_velocity(2), delta_t);
    acc_buf_.push_back(linear_acceleration);
    gyr_buf_.push_back(angular_velocity);
    dt_buf_.push_back(delta_t);
  };

  void rePreintegration(const gtsam::imuBias::ConstantBias& new_bias){
    gtsam::PreintegratedImuMeasurements::resetIntegrationAndSetBias(new_bias);
    for (int i = 0; i < acc_buf_.size(); i++){
      gtsam::PreintegratedImuMeasurements::integrateMeasurement(acc_buf_.front(), gyr_buf_.front(), dt_buf_.front());
    }
  };

  void resetpreintergration(){
    gtsam::PreintegratedImuMeasurements::resetIntegration();
    acc_buf_.clear();
    gyr_buf_.clear();
    dt_buf_.clear();
  };
protected:
  std::list <gtsam::Vector3> acc_buf_;
  std::list <gtsam::Vector3> gyr_buf_;
  std::list <double> dt_buf_;
};

}// namespace CustomGTSAMFactors

#endif // GTSAM_FACTORS_HPP