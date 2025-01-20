#include <list>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <gtsam/navigation/ImuFactor.h>
#include <opencv2/opencv.hpp>

#include "featureTracker/feature_tracker.h"
#include "feature_manager.h"

//Public definations  sliding window

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

    SldWindowStatus(params & param);
    ~SldWindowStatus();
    void setStatus(BasicStatus & status);

    int32_t InputIMUMeasurement(double t, const Eigen::Vector3d &linear_acceleration,
                                 const Eigen::Vector3d &angular_velocity);
    int32_t InputViusalMeasurement(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
private:
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> preintegrated_;
    
    BasicStatus status_;
    params param_;
};


class GTSAMEstimator{
public:
    GTSAMEstimator();
    ~GTSAMEstimator();
    bool initializeWithIMU(std::vector<IMUMeasurement> & imu_measurements, BasicStatus & status);
    
    // frontend
    void inputIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    // this is the frontend stereo image tracking thread
    void processImage(); 

    //this is the backend thread: construct optimization problem and solve it
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void setParameter();
private:
    enum EstimatorStatus{
        InitializeFirstPose = 0,
        Initialize = 1,
        Norminal = 2,
        Failed = 3
    };

    //frontend data buffer
    struct ImageFrame{
        ImageFrame(double _time_stamp, cv::Mat _left_img, cv::Mat _right_img):
            time_stamp(_time_stamp), left_img(_left_img), right_img(_right_img){
        }
        double time_stamp;
        cv::Mat left_img;
        cv::Mat right_img;
    };
    std::mutex raw_image_buffer_mutex_;
    std::list<ImageFrame> image_buffer_;
    int32_t frame_id_ = 0; // this give the unqiue id for each frame


    //frontend
    FeatureTracker feature_tracker_;
    FeatureManager feature_manager_; //We will have frontend feature manager to address lossing tracking; 
                                     //and then a backend feature manager to facilitate the optimization
    
    //frontend thread
    std::thread image_track_thread_;

    //backend

    int32_t getIMUMeasurements(double t0, double t1, std::vector<IMUMeasurement> & imu_measurements);
    

    EstimatorStatus estimator_status_ = InitializeFirstPose;

    std::list<SldWindowStatus> sliding_windows_;

    std::mutex imu_buffer_mutex_;
    std::list<IMUMeasurement> imu_buffer_;

    std::mutex feature_buffer_mutex_;
    std::list<std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>> feature_buffer_;

    BasicStatus latest_status_;

    //configurations
    SldWindowStatus::params sld_params_;

};