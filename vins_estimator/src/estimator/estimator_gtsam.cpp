#include <boost/make_shared.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/inference/Symbol.h>

#include "estimator/estimator_gtsam.hpp"
#include "utility/utility.h"
#include "estimator/parameters.h"

GTSAMEstimator::GTSAMEstimator(GTSAMEstimatorParams &params){
    // ROS_INFO("init begins");
    // initThreadFlag = false;
    // clearState();
    feature_tracker_ = std::make_shared<FeatureTracker>();
    if (feature_tracker_ == nullptr){
        printf("[Error] Failed to create feature tracker\n");
        return;
    }
    StereoFeatureManager::StereoFeatureManagerParams stereo_params;
    stereo_params.long_track_threshold = params.long_track_feature_threshold;
    
    stereo_feature_manager_ = std::make_shared<StereoFeatureManager>(stereo_params);

    if (stereo_feature_manager_ == nullptr){
        printf("[Error] Failed to create stereo feature manager\n");
        return;
    }
    
}

GTSAMEstimator::~GTSAMEstimator(){
}


// This function is only a callback function for the IMU data
void GTSAMEstimator::inputIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity){
    if (imu_buffer_mutex_.try_lock()){
        imu_buffer_.push_back(IMUMeasurement(t, linear_acceleration, angular_velocity));
        imu_buffer_mutex_.unlock();
        return ;
    } else {
        printf("[Warning] backend slow: IMU buffer is locked, this would be resulted in IMU data loss\n");
    }

}


//GTSAMEstimator::inputFeature()
// This function inputs stereo images, I suggest to move this function to frontend. It is actually a front end!
// only key frame will be add into feature_buffer_, and feature will be added in to featureManager
// We have to notice that in orignal VINS, 30Hz image is sampled to 15Hz. This would largly reduce the stabilitiy of the system.
// So in the future development, we should consider to maxize the frequency of the image input.
void GTSAMEstimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1){
    if(raw_image_buffer_mutex_.try_lock()){
        raw_image_buffer_.push_back(ImageFrame(t, _img, _img1));
        raw_image_buffer_mutex_.unlock();
    }
}

//this should be a singel thread
void GTSAMEstimator::processImage(){
    if (raw_image_buffer_mutex_.try_lock()){
        if (raw_image_buffer_.empty()){
            raw_image_buffer_mutex_.unlock();
            return;
        }
        ImageFrame image_frame = raw_image_buffer_.front();
        raw_image_buffer_.pop_front();
        raw_image_buffer_mutex_.unlock();

        //process image
        auto features = feature_tracker_->trackImage(image_frame.time_stamp, image_frame.left_img, image_frame.right_img);
        FeatureFrame new_feature_frame(image_frame.time_stamp, features);
        // Debug show track image
        if (SHOW_TRACK){
            cv::Mat imgTrack = feature_tracker_->getTrackImage();
            pubTrackImage(imgTrack, image_frame.time_stamp);
        }
        if (feature_frame_buffer_mutex_.try_lock()){
            feature_frame_buffer_.push_back(new_feature_frame);
            feature_frame_buffer_mutex_.unlock();
        } else {
            printf("[Warning] feature_frame_buffer_mutex_ is locked, this would be resulted in tracking loss\n");
        }
    }
    return;
}

void GTSAMEstimator::processMeasurements(){
    //get feature time stamp
    if (feature_frame_buffer_mutex_.try_lock()){
        if (feature_frame_buffer_.empty()){
            feature_frame_buffer_mutex_.unlock();
            return;
        }
        auto feature_frame = feature_frame_buffer_.front();
        feature_frame_buffer_.pop_front();
        feature_frame_buffer_mutex_.unlock();

        double frame_t = feature_frame.time_stamp;

        int32_t next_frame_in_sldwin = 0; //this is the new frame index in the sliding window

        if (estimator_status_  == InitializeFirstPose){
            //initialize first pose
            BasicStatus first_pose_status;
            std::list<IMUMeasurement> imu_measurements;

            double imu_start_t = imu_buffer_.front().t;
            if (imu_start_t > frame_t){
                printf("[Warning] IMU measurements are not enough for initialization; Wait for more IMU measurements\n");
                return;
            }
            if (getIMUMeasurements(imu_buffer_.front().t, frame_t, imu_measurements) < 2){
                printf("[Warning] IMU measurements are not enough for initialization \n");
                return;
            }

            removeIMUMeasurementsBefroeTime(frame_t);

            if (initializeWithIMU(imu_measurements, first_pose_status)){
                sliding_windows_.clear();
                next_frame_in_sldwin = sliding_windows_.size();
                SldWindowStatus new_status(frame_t, params_.sld_params_);
                new_status.setStatus(first_pose_status);
                sliding_windows_.push_back(new_status); //add first key frame to sliding_windows_
                //then add feature into feature_manager
                stereo_feature_manager_->addFeatures(next_frame_in_sldwin,feature_frame.features, frame_t);

                //triangluate stereo features; here the next_frame_in_sldwin is the index of the new frame in the sliding window
                stereo_feature_manager_->triangulateFeatures(sliding_windows_); // these would go through all features TODO: we should 

                estimator_status_ = Initialize;
            } else {
                printf("[Erro] Initialize first pose with IMU failed\n");
                return;
            }
        
        } else if (estimator_status_ == Initialize){
            // add imu measurements to the latest sliding_windows_. preintegrated_; this design allows always add imu measurements to the latest key frame imu spreintergration
            //if is keyframe
            next_frame_in_sldwin = sliding_windows_.size();
            if (stereo_feature_manager_->isKeyFrame(next_frame_in_sldwin, feature_frame.features)){
                //put imu measturements to the current sliding_windows_.back() and then create a new sliding_window
                std::list<IMUMeasurement> imu_measurements;
                int32_t ret = getIMUMeasurements(sliding_windows_.back().getStartTime(), frame_t, imu_measurements);

                for (auto imu_measurement : imu_measurements){
                    sliding_windows_.back().inputIMUMeasurement(imu_measurement.t, imu_measurement.linear_acceleration, imu_measurement.angular_velocity);
                }
                
                //create new sliding_window status
                stereo_feature_manager_->addFeatures(next_frame_in_sldwin, feature_frame.features, frame_t);
                
                //PnP
                BasicStatus new_status;
                stereo_feature_manager_->initNewFramePoseByPnP(next_frame_in_sldwin, sliding_windows_, new_status);
                SldWindowStatus new_sld_status(frame_t, params_.sld_params_);
                new_sld_status.setStatus(new_status);
                sliding_windows_.push_back(new_sld_status);

                stereo_feature_manager_->triangulateFeatures(sliding_windows_);
                //optimize for initial sliding_windows_
                if (sliding_windows_.size() == params_.sld_window_size){
                    if (optimzeStatus()){
                        estimator_status_ = Norminal;
                        //update status
                    } else {
                        printf("[Warning] Optimization failed\n");
                        return;
                    }
                }
            }
        } else if (estimator_status_ == Norminal){
            // add imu measurements to the latest sliding_windows_. preintegrated_; this design allows always add imu measurements to the latest key frame imu spreintergration
            next_frame_in_sldwin = sliding_windows_.size();
            std::list<IMUMeasurement> interval_imu_measurements;
            auto imu_measurements = getIMUMeasurements(sliding_windows_.back().getStartTime(), frame_t, interval_imu_measurements);
            for (auto imu_measurement : interval_imu_measurements){
                sliding_windows_.back().inputIMUMeasurement(imu_measurement.t, imu_measurement.linear_acceleration, imu_measurement.angular_velocity);
                // sliding_windows_.back().preintegrated_->integrateMeasurement(imu_measurement.linear_acceleration, imu_measurement.angular_velocity, imu_measurement.t);
            }

            //if is keyframe
            bool is_keyframe = stereo_feature_manager_->isKeyFrame(next_frame_in_sldwin,feature_frame.features);
            
            stereo_feature_manager_->addFeatures(next_frame_in_sldwin,feature_frame.features, frame_t);
            // PNP
            BasicStatus new_status;
            stereo_feature_manager_->initNewFramePoseByPnP(next_frame_in_sldwin, sliding_windows_, new_status);
            // triangulate
            stereo_feature_manager_->triangulateFeatures(sliding_windows_);
            //add this frame to back of sliding_windows_
            SldWindowStatus new_sld_status(frame_t,params_.sld_params_);
            new_sld_status.setStatus(new_status);
            sliding_windows_.push_back(new_sld_status);

            //optimize
            optimzeStatus();

            //marginalize
            if (is_keyframe){
                //marginlize the old sliding_windows_
                marginalizeOldStatus();
            } else {
                marginalizeSecondNewStatus();
                //mraginlize the second new sliding_windows_
            }

            //push the outcome the the buffer for publishing
  
        }
    }
}
    

int32_t GTSAMEstimator::getIMUMeasurements(double t_start, double t_end, std::list<IMUMeasurement>& imu_measurements){
    if (imu_buffer_mutex_.try_lock()){
        for (auto it = imu_buffer_.begin(); it != imu_buffer_.end(); it++){
            if (it->t >= t_start && it->t <= t_end){
                imu_measurements.push_back(*it);
            }
        }
        imu_buffer_mutex_.unlock();
    }
    return imu_measurements.size();
}

bool GTSAMEstimator::initializeWithIMU(std::list<IMUMeasurement> & imu_measurements, BasicStatus & status){
    if (imu_measurements.size() < 2){
        printf("[Warning] IMU measurements are not enough for initialization \n");
        //TODO:: make all printf to spdlog or glog
        return false;
    }
    // make sure there is no status in sliding_windows_
    if (!sliding_windows_.empty()){
        printf("[Warning] There should be no status in sliding_windows_  Initialization failed\n");
        return false;
    }

    //Align with gravity
    Eigen::Vector3d averAcc(0, 0, 0);
    for (auto imu_measurement : imu_measurements){
        averAcc += imu_measurement.linear_acceleration;
    }
    averAcc /= imu_measurements.size();
    Eigen::Matrix3d init_R = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(init_R).x();
    init_R = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * init_R;
    
    BasicStatus init_status;
    init_status.R = init_R;

    double time = imu_measurements.back().t;

    //initialize first status
    SldWindowStatus first_status(time, params_.sld_params_);
    first_status.setStatus(init_status);
    //first status add to sliding_windows_
    sliding_windows_.push_back(first_status);
    return true;
}

bool GTSAMEstimator::optimzeStatus(){
    //TODO:: we can have different optimization strategy here 
    optimzeWithGTSAM();
}

bool GTSAMEstimator::optimzeWithGTSAM(){
    
    using gtsam::symbol_shorthand::X ; //pose
    using gtsam::symbol_shorthand::V ; //velocity
    using gtsam::symbol_shorthand::B ; //bias

    using gtsam::symbol_shorthand::L ; //landmark

    gtsam::NonlinearFactorGraph graph;
    //add pose node
    gtsam::Values initial_values;
    for (int i = 0; i < sliding_windows_.size(); i++){
        Eigen::Matrix<double, 3, 4> body_pose;
        sliding_windows_[i].getBodyPose(body_pose);
        gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(body_pose.block<3, 3>(0, 0)), gtsam::Point3(body_pose.block<3, 1>(0, 3)));
        initial_values.insert(X(i), pose);
    }

    //add imu factor //
    for (int i = 0; i < sliding_windows_.size(); i++){
        auto imu_preintegration = sliding_windows_[i].getImuPreintegration();
        gtsam::imuBias::ConstantBias prior_imu_bias(acc_bias_, gyro_bias_);
        // add IMU factor
        if (i < sliding_windows_.size() - 1){
            gtsam::PreintegratedCombinedMeasurements* imu_preintegration = 
                dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(sliding_windows_[i].getImuPreintegration().get());
            gtsam::CombinedImuFactor imu_factor(X(i), V(i), X(i + 1), V(i + 1), B(i), B(i), *imu_preintegration);
            graph.add(imu_factor);
        } 
    }

    //add visual factor
    for(auto feature: stereo_feature_manager_->features_){
        if(feature.second->solve_flag == FeaturePerId::SolveFlag::FEATURE_DEPTH_VALID){
            // if optimize extrinsic parameters of stereo camera
            if (feature.second->is_stereo){
                //stereo observation factor
                // gtsam::GenericStereoFactor
            } else {
                //mono observation factor
            }

        }
    }

    // add prior factor and marginalize factor

    //optimize
}


SldWindowStatus::SldWindowStatus(double start_time, SldWindowStatus::params & param): 
    start_time_(start_time), end_time_(start_time), param_(param){
    status_ = BasicStatus();
    //gtsam preintergration parameters
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(0.0);
    p->accelerometerCovariance = gtsam::I_3x3 * std::pow(param_.acc_noise_density_, 2);
    p->gyroscopeCovariance = gtsam::I_3x3 * std::pow(param_.gyro_noise_density_, 2);
    p->integrationCovariance = gtsam::I_3x3 * 1e-8; //integration noise should be very small TODO: make it configuratble
    p->biasAccCovariance = gtsam::I_3x3 * std::pow(param_.init_bias_sigma_, 2);
    p->biasOmegaCovariance = gtsam::I_3x3 * std::pow(param_.init_bias_sigma_, 2);
    p->biasAccOmegaInt = gtsam::I_6x6 * 1e-5; //integration noise should be very small TODO: make it configuratble

    gtsam::imuBias::ConstantBias prior_imu_bias(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
    
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegration_ = 
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p, prior_imu_bias);
    
    if (imu_preintegration_ == nullptr){
        printf("[Error] imu_preintegration_ is nullptr\n");
    }

}

SldWindowStatus::~SldWindowStatus(){
    if (imu_preintegration_ != nullptr){
        imu_preintegration_.reset();
    }
}

void SldWindowStatus::setStatus(BasicStatus & status){
    status_ = status;
}

bool SldWindowStatus::inputIMUMeasurement(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity){
    if (t <= end_time_){
        printf("[Warning] IMU measurements are not in order\n");
    }
    end_time_ = t;
    // optional
    imu_measurements_.push_back(IMUMeasurement(t, linear_acceleration, angular_velocity));
    
    imu_preintegration_->integrateMeasurement(linear_acceleration, angular_velocity, t);
    return true;
}

bool SldWindowStatus::heritagePreintegration(std::list<IMUMeasurement> & imu_measurements){
    //endtime of the last frame
    for (auto imu_measurement : imu_measurements){
        if (imu_measurement.t > end_time_){
            imu_measurements_.push_back(imu_measurement);
            imu_preintegration_->integrateMeasurement(imu_measurement.linear_acceleration, imu_measurement.angular_velocity, imu_measurement.t);
            end_time_ = imu_measurement.t;
        }
    }
}

void SldWindowStatus::getBodyPose(Eigen::Matrix<double, 3, 4, 0, 3, 4> &body_pose){
    body_pose.block<3, 3>(0, 0) = status_.R;
    body_pose.block<3, 1>(0, 3) = status_.p;
    return;
}


