#include <gtsam/nonlinear/NonlinearFactor.h>

namespace CustomGTSAMFactors{

// Stereo extrinsic parameter factor
class StereoExtrinsicFactor: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Point3> {
  public:
    StereoExtrinsicFactor() = delete;
    gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, const gtsam::Point3& point, 
        boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none) const override;

  private:
    gtsam::Point3 measurement_;
};

// Stereo reprojection factor
class StereoReprojectionFactor: public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> {
  public:
    StereoReprojectionFactor() = delete;
  private:
    gtsam::Point3 measurement_;
    

};

// Mono reprojection factor
class MonoReprojectionFactor: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> {
  public:
    MonoReprojectionFactor() = delete;
  private:
    gtsam::Point3 measurement_;

};


// This factor is used to estimate the time delay between the IMU and the camera
class TdFactor: public gtsam::NoiseModelFactor1<double> {
  public:
    TdFactor() = delete;

};

} // namespace CustomGTSAMFactors