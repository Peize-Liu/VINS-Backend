#include "estimator_gtsam.hpp"
#include "utility/utility.h"


SldWindowStatus::SldWindowStatus(){

}

SldWindowStatus::~SldWindowStatus(){

}

GTSAMEstimator::GTSAMEstimator(){
    // ROS_INFO("init begins");
    // initThreadFlag = false;
    // clearState();
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
        auto features = feature_tracker_.trackImage(image_frame.time_stamp, image_frame.left_img, image_frame.right_img);
        FeatureFrame new_feature_frame(image_frame.time_stamp, features);
        // Debug show track image
        if (SHOW_TRACK){
            cv::Mat imgTrack = feature_tracker_.getTrackImage();
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

        int32_t new_frame_in_sldwin = 0; //this is the new frame index in the sliding window

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
                new_frame_in_sldwin = sliding_windows_.size();
                sliding_windows_.push_back(SldWindowStatus(frame_t, params_.sld_params_,first_pose_status)); //add first key frame to sliding_windows_
                //then add feature into feature_manager
                stereo_feature_manager_.addFeature(new_frame_in_sldwin,feature_frame);
                feature_manager_.triangulate();
                estimator_status_ = Initialize;
            } else {
                printf("[Erro] Initialize first pose with IMU failed\n");
                return;
            }
        
        } else if (estimator_status_ == Initialize){
            // add imu measurements to the latest sliding_windows_. preintegrated_; this design allows always add imu measurements to the latest key frame imu spreintergration
            //if is keyframe
            if (feature_manager_.isKeyFrame(feature_frame)){
                //put imu measturements to the current sliding_windows_.back() and then create a new sliding_window
                std::list<IMUMeasurement> imu_measurements;
                auto imu_measurements = getIMUMeasurements(sliding_windows_.back().getStartTime(), frame_t, imu_measurements);

                for (auto imu_measurement : imu_measurements){
                    sliding_windows_.back().preintegrated_->integrateMeasurement(imu_measurement.linear_acceleration, imu_measurement.angular_velocity, imu_measurement.t);
                }
                
                new_frame_in_sldwin = sliding_windows_.size()
                //create new sliding_window status
                feature_manager_.addFeature(new_frame_in_sldwin, feature_frame);
                
                //PnP
                BasicStatus new_status;
                feature_manager_.initFramePoseByPnP(new_status);
                feature_manager_.triangulate();


                //add to sliding_windows_
                SldWindowStatus new_sld_status(sld_params_, new_status);
                sliding_windows_.push_back(new_sld_status);

                if (sliding_windows_.size() == sld_params_.window_size){
                    //optimize for initial sliding_windows_
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
            auto imu_measurements = getIMUMeasurements(sliding_windows_.back().status_.time, feature_frame.time_stamp);
            for (auto imu_measurement : imu_measurements){
                sliding_windows_.back().preintegrated_->integrateMeasurement(imu_measurement.linear_acceleration, imu_measurement.angular_velocity, imu_measurement.t);
            }

            //if is keyframe
            is_keyframe = feature_manager_.isKeyFrame(feature_frame);
            feature_manager_.addFeature(feature_frame);
            // PNP
            // triangulate
            //add this frame to back of sliding_windows_

            //optimize
            if (is_keyframe){
                //marginlize the old sliding_windows_
                oldest_sld_status = sliding_windows_.front();

            } else {
                second_new_sld_status = sliding_windows_[sliding_windows_.size() - 2];
                //mraginlize the second new sliding_windows_

            }

            //update status
            
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

    //initialize first status
    SldWindowStatus first_status(sld_params_);
    first_status.setStatus(init_status);

    //first status add to sliding_windows_
    sliding_windows_.push_back(first_status);
    return true;
}

SldWindowStatus::SldWindowStatus(SldWindowStatus::params & param):param_(param){
    status_ = BasicStatus();
    //gtsam preintergration parameters
    preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>();
}

SldWindowStatus::SldWindowStatus(SldWindowStatus::params & param, BasicStatus & status):param_(param), status_(status){
     //gtsam preintergration parameters
    preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>();
}
`


void SldWindowStatus::setStatus(BasicStatus & status){
    status_ = status;
    preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>();
}