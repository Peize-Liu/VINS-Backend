//Public definations  sliding window
#ifndef SLIDING_WINDOW_H
#define SLIDING_WINDOW_H

#include <list>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

struct BasicStatus{
    BasicStatus(){
        p.setZero();
        v.setZero();
        R.setIdentity();
        bias_a.setZero();
        bias_g.setZero();
    }
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;
    Eigen::Vector3d bias_a;
    Eigen::Vector3d bias_g;
};

struct IMUMeasurement{
    IMUMeasurement(){
        t = 0;
        linear_acceleration.setZero();
        angular_velocity.setZero();
    }
    IMUMeasurement(double _t, const Eigen::Vector3d &_linear_acceleration, const Eigen::Vector3d &_angular_velocity):
        t(_t), linear_acceleration(_linear_acceleration), angular_velocity(_angular_velocity){
    }
    double t;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

class SldWindowStatus{
public:
    struct params{
        double init_bias_sigma_ = 0.0;
        double gyro_noise_density_ = 0.0;
        double gyro_random_walk_ = 0.0;
        double acc_noise_density_ = 0.0;
        double acc_random_walk_ = 0.0;
        double imu_time_shift_ = 0.0;  // Defined as t_imu = t_cam + imu_shift
    };

    SldWindowStatus(double start_time, params & param);
    ~SldWindowStatus();
    void setStatus(BasicStatus & status);

    double getStartTime(){
        return start_time_;
    }
    
    double getEndTime(){
        return end_time_;
    }

    bool inputIMUMeasurement(double t, const Eigen::Vector3d &linear_acceleration,
                                 const Eigen::Vector3d &angular_velocity);
    // int32_t InputViusalMeasurement(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    
    bool getAllmeasurements(std::list<IMUMeasurement> & imu_measurements){
        imu_measurements = imu_measurements_;
    }

    // add imu preintegration from the next frame to the current frame
    bool heritagePreintegration(std::list<IMUMeasurement> & imu_measurements);
    
    void getBodyPose(Eigen::Matrix<double, 3, 4> &body_pose);

    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> getImuPreintegration(){
        return imu_preintegration_;
    }

private:
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegration_ = nullptr;
    std::list<IMUMeasurement> imu_measurements_;
    BasicStatus status_;
    params param_;

    double start_time_;
    double end_time_;
};

#endif