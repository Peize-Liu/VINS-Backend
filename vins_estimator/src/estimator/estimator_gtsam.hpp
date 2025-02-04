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

#include "featureTracker/feature_tracker.h"
#include "estimator/stereo_feature_manager.hpp"
#include "estimator/sldwindows_gtsam.hpp"

class GTSAMEstimator{
public:
    struct GTSAMEstimatorParams{
        SldWindowStatus::params sld_params_;
        int32_t sld_window_size = 10;
        int32_t long_track_feature_threshold = 3;
    };

    GTSAMEstimator(GTSAMEstimatorParams &params);
    ~GTSAMEstimator();

    
    bool initializeWithIMU(std::list<IMUMeasurement> & imu_measurements, BasicStatus & status);
    
    // frontend call back
    void inputIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    // this is the frontend stereo image tracking thread
    void processImage(); 

    //this is the backend thread: construct optimization problem and solve it
    void processMeasurements();


    void changeSensorType(int use_imu, int use_stereo);
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void setParameter();
    
    //Debug functions
    void pubTrackImage(cv::Mat &img, double time_stamp);

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

    struct FeatureFrame{
        FeatureFrame(double _time_stamp, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> _features):
            time_stamp(_time_stamp), features(_features){
        }
        double time_stamp;
        std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> features;
    };

    std::mutex imu_buffer_mutex_;
    std::list<IMUMeasurement> imu_buffer_; //this would save imu measurements between two frames

    std::mutex raw_image_buffer_mutex_;
    std::list<ImageFrame> raw_image_buffer_;

    
    std::mutex feature_frame_buffer_mutex_;
    // std::list<std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>> feature_frame_buffer_;
    std::list<FeatureFrame> feature_frame_buffer_;


    int32_t frame_id_ = 0; // this give the unqiue id for each frame

    //frontend
    std::shared_ptr<FeatureTracker> feature_tracker_ = nullptr;

    // FeatureManager feature_manager_; //We will have frontend feature manager to address lossing tracking; 
    //                                  //and then a backend feature manager to facilitate the optimization
    std::shared_ptr<StereoFeatureManager> stereo_feature_manager_ = nullptr;
    //frontend thread
    std::thread image_track_thread_;

    //backend
    EstimatorStatus estimator_status_ = InitializeFirstPose;

    std::deque<SldWindowStatus> sliding_windows_;


    //GTSAM backend parameters
    Eigen::Vector3d acc_bias_;
    Eigen::Vector3d gyro_bias_;

    BasicStatus latest_status_; //TODO may not be used

    //configurations
    GTSAMEstimatorParams params_;

    int32_t getIMUMeasurements(double t0, double t1, std::list<IMUMeasurement>& imu_measurements);
    int32_t removeIMUMeasurementsBefroeTime(double t);
    bool optimzeStatus();

    bool optimzeWithGTSAM();
    bool optimzeWithCeres();//TODO::

    bool setOptimizedStatus();
    bool marginalizeOldStatus();
    bool marginalizeSecondNewStatus();

    bool updateStataus();

};