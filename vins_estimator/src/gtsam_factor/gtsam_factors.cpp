#include "gtsam_factor/gtsam_factors.hpp"
#include <gtsam/nonlinear/NonlinearFactor.h>

StereoExtrinsicFactor::StereoExtrinsicFactor(const gtsam::Point3& point, const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, const gtsam::SharedNoiseModel& model):
    NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Point3>(model, 1), measurement_(point){
    pose1_ = pose1;
    pose2_ = pose2;
}

gtsam::Vector StereoExtrinsicFactor::evaluateError(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, const gtsam::Point3& point, 
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2, boost::optional<gtsam::Matrix&> H3) const {
    gtsam::Point3 point1 = pose1.transformFrom(point);
    gtsam::Point3 point2 = pose2.transformFrom(point);
    gtsam::Vector3 error = point1 - point2;
    if (H1){
        *H1 = gtsam::Matrix36::Zero();
        *H1 = gtsam::I_3x3;
    }
    if (H2){
        *H2 = gtsam::Matrix36::Zero();
        *H2 = -gtsam::I_3x3;
    }
    if (H3){
        *H3 = gtsam::Matrix36::Zero();
        *H3 = gtsam::I_3x3;
    }
    return error;
}
