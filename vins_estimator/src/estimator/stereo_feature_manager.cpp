// this is the new feature manager realized by std::map
#include "estimator/stereo_feature_manager.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


StereoFeatureManager::StereoFeatureManager(StereoFeatureManagerParams params):
    params_(params){
}

StereoFeatureManager::~StereoFeatureManager(){
}

bool StereoFeatureManager::clearFeatures(){
    features_.clear();
    return true;
}

// sldwin_index is the length of the sliding window
bool StereoFeatureManager::isKeyFrame(int32_t next_sldwin_index ,const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &input_features){
    //check if the new frame is keyframe or not
    //if the new frame has more than long_track_threshold features, it is a keyframe
    double parallax_sum = 0;
    int parallax_num = 0;

    //reset counter
    int last_track_num = 0;
    int last_average_parallax = 0;
    int new_feature_num = 0;
    int long_track_num = 0;

    for (auto &feature : input_features){
        int32_t feature_id = feature.first;
        auto it = features_.find(feature_id);
        if (it == features_.end()){
            //new feature
            new_feature_num++;
        } else {
            //old feature
            last_track_num++;
            if (it->second->feature_per_frame.size() > (params_.long_track_threshold -1)){
                long_track_num++;
            }
        }
    }

    if (next_sldwin_index < 2 || last_track_num < 20 || long_track_num < 40 ||
        new_feature_num > (last_track_num * params_.last_track_threshold)){
        return true;
    }

    //check the paralaax with the last frame
    for (auto & new_feature : input_features){
       auto  feature_iter = features_.find(new_feature.first);
       if (feature_iter != features_.end()){
            if(feature_iter->second->start_frame <= next_sldwin_index - 1 && // has to be observed in the last frame && continuesly observed till the last frame
                (feature_iter->second->start_frame + int32_t(feature_iter->second->feature_per_frame.size()) -1) == next_sldwin_index - 1){
                auto last_frame_observe = feature_iter->second->feature_per_frame[next_sldwin_index - 1 - feature_iter->second->start_frame];
                auto new_frame_observe = new_feature.second[0].second;
                Eigen::Vector3d last_frame_observe_vec(last_frame_observe.point);
                Eigen::Vector3d new_frame_observe_vec = new_frame_observe.head<3>();
                parallax_sum += compensatedParallax2(last_frame_observe_vec, new_frame_observe_vec);
                parallax_num++;
            }
       }
    }
    if (parallax_num == 0){
        return true;
    }
    return (parallax_sum / parallax_num) > params_.min_parallax;
}

bool StereoFeatureManager::addFeatures(int sldwin_index, 
    const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &input_features, double td){
    for (auto &feature : input_features){
        FeaturePerFrame feature_per_frame(feature.second[0].second, td);
        bool is_stereo = false;
        if (feature.second.size() == 2){
            feature_per_frame.rightObservation(feature.second[1].second);
            is_stereo = true;
        }
        int32_t feature_id = feature.first;
        auto it = features_.find(feature_id);

        //define stereo measurements at first observation
        if (it == features_.end()){
            //new feature
            features_[feature_id] = std::make_shared<FeaturePerId>(feature_id, sldwin_index);
            features_[feature_id]->feature_per_frame.push_back(feature_per_frame);
            features_[feature_id]->is_stereo = is_stereo;
        } else {
            //old feature
            it->second->feature_per_frame.push_back(feature_per_frame);
        }
    }
    return true;
}

// first try triangulate with stereo measurements,

void StereoFeatureManager::triangulateFeatures(std::deque<SldWindowStatus> &sliding_windows_pose){
    for (auto & feature : features_){
        if (feature.second->solve_flag == FeaturePerId::SolveFlag::FEATURE_DEPTH_UNINITIALIZED){
            if (feature.second->is_stereo){
                //has stereo measurements
                FeaturePerFrame stereo_frame = feature.second->feature_per_frame[feature.second->stereo_measurements];
                int32_t reference_pose_index = feature.second->start_frame + feature.second->stereo_measurements;
                Eigen::Matrix <double, 3, 4> ego_pose;
                sliding_windows_pose[reference_pose_index].getBodyPose(ego_pose);
                Eigen::Vector3d t_ego = ego_pose.block<3, 1>(0, 3);
                Eigen::Matrix3d R_ego = ego_pose.block<3, 3>(0, 0);

                Eigen::Vector2d point_left = stereo_frame.point.head(2);
                Eigen::Vector2d point_right = stereo_frame.pointRight.head(2);

                Eigen::Matrix3d R_left_cam_ego = R_ego * extrinsics_[0].R;
                Eigen::Vector3d t_left_cam_ego = t_ego + R_ego * extrinsics_[0].t;

                Eigen::Matrix3d R_right_cam_ego = R_ego * extrinsics_[1].R;
                Eigen::Vector3d t_right_cam_ego = t_ego + R_ego * extrinsics_[1].t;

                Eigen::Matrix <double, 3, 4> pose_left = Eigen::Matrix <double, 3, 4>::Zero(); //T_world_cam
                pose_left.block<3, 3>(0, 0) = R_left_cam_ego.transpose();
                pose_left.block<3, 1>(0, 3) = -R_left_cam_ego.transpose() * t_left_cam_ego;

                Eigen::Matrix <double, 3, 4> pose_right = Eigen::Matrix <double, 3, 4>::Zero(); //T_world_cam
                pose_right.block<3, 3>(0, 0) = R_right_cam_ego.transpose();
                pose_right.block<3, 1>(0, 3) = -R_right_cam_ego.transpose() * t_right_cam_ego;

                Eigen::Vector3d point3d;
                triangulateFeature(pose_left, pose_right, point_left, point_right, point3d);

                Eigen::Vector3d point3d_cam = R_ego.transpose() * (point3d - t_ego);

                if (point3d_cam(2) > 0){
                    feature.second->estimated_depth = point3d_cam(2);
                    feature.second->solve_flag = FeaturePerId::SolveFlag::FEATURE_DEPTH_VALID;
                } else {
                    feature.second->estimated_depth = 5.0; //set to a large value
                    feature.second->solve_flag = FeaturePerId::SolveFlag::FEATURE_DEPTH_INVALID; //TODO: may be bug here
                }
            } 
            else if(feature.second->feature_per_frame.size() == 2 || 
                feature.second->feature_per_frame.size() >=params_.long_track_threshold){
                std::list<Eigen::Matrix<double, 3, 4>> poses;
                std::list<Eigen::Vector2d> points;
                Eigen::Vector3d point3d;
                
                int32_t pose_index = 0;
                for (auto & measurement : feature.second->feature_per_frame){
                    int32_t reference_pose_index = feature.second->start_frame + pose_index;
                    Eigen::Matrix <double, 3, 4> ego_pose;
                    sliding_windows_pose[reference_pose_index].getBodyPose(ego_pose);
                    Eigen::Vector3d t_ego = ego_pose.block<3, 1>(0, 3);
                    Eigen::Matrix3d R_ego = ego_pose.block<3, 3>(0, 0);

                    Eigen::Matrix3d R_left_cam_ego = R_ego * extrinsics_[0].R;
                    Eigen::Vector3d t_left_cam_ego = t_ego + R_ego * extrinsics_[0].t;

                    Eigen::Matrix<double, 3, 4> pose_left = Eigen::Matrix<double, 3, 4>::Zero();
                    pose_left.block<3, 3>(0, 0) = R_left_cam_ego.transpose();
                    pose_left.block<3, 1>(0, 3) = -R_left_cam_ego.transpose() * t_left_cam_ego;

                    poses.push_back(pose_left);
                    points.push_back(measurement.point.head(2));
                    pose_index++;
                }
                sdvTriangulateFeature(poses, points, point3d);

                // transform the point to camera frame
                Eigen::Matrix <double, 3, 4> ego_pose;
                sliding_windows_pose[feature.second->start_frame].getBodyPose(ego_pose);
                Eigen::Vector3d t_ego = ego_pose.block<3, 1>(0, 3);
                Eigen::Matrix3d R_ego = ego_pose.block<3, 3>(0, 0);
                Eigen::Vector3d point3d_cam = R_ego.transpose() * (point3d - t_ego);
                if (point3d_cam(2) > 0){
                    feature.second->estimated_depth = point3d_cam(2);
                    feature.second->solve_flag = FeaturePerId::SolveFlag::FEATURE_DEPTH_VALID;
                } else {
                    feature.second->estimated_depth = 5.0; //set to a large value
                    feature.second->solve_flag = FeaturePerId::SolveFlag::FEATURE_DEPTH_INVALID;
                }
            }
        }
    }
}


//initialize the new frame pose by PnP
void StereoFeatureManager::initNewFramePoseByPnP(int32_t next_sldwin_index, std::deque<SldWindowStatus> &sliding_windows_pose,BasicStatus &status){
    if(next_sldwin_index < 1){
        return;
    }
    std::vector<cv::Point2f> pts_2d;
    std::vector<cv::Point3f> pts_3d;

    for (auto &feature : features_){
        if (feature.second->solve_flag == FeaturePerId::SolveFlag::FEATURE_DEPTH_VALID &&  //feature has valid depth
            (feature.second->start_frame + int32_t(feature.second->feature_per_frame.size()) - 1) == next_sldwin_index - 1){ //end frame is the last frame of the feature
            int32_t search_index = feature.second->start_frame;
            Eigen::Matrix<double, 3, 4> pose;
            sliding_windows_pose[search_index].getBodyPose(pose);
            Eigen::Vector3d point3d_in_cam, point3d_in_body, point3d_in_world;
            point3d_in_cam = feature.second->feature_per_frame.front().point.head(3) * feature.second->estimated_depth;
            point3d_in_body = extrinsics_[0].R * point3d_in_cam +  extrinsics_[0].t;
            point3d_in_world = pose.block<3, 3>(0, 0) * point3d_in_body + pose.block<3, 1>(0, 3);
            pts_3d.push_back(cv::Point3f(point3d_in_world(0), point3d_in_world(1), point3d_in_world(2)));

            //new observation
            pts_2d.push_back(cv::Point2f(feature.second->feature_per_frame.back().point(0), feature.second->feature_per_frame.back().point(1)));
        }
    }
    //use last frame pose as the initial guess
    Eigen::Matrix<double, 3, 4> last_frame_pose;
    sliding_windows_pose[next_sldwin_index - 1].getBodyPose(last_frame_pose);
    Eigen::Matrix3d R_w_body = last_frame_pose.block<3, 3>(0, 0);
    Eigen::Vector3d t_w_body = last_frame_pose.block<3, 1>(0, 3);

    Eigen::Matrix3d R_w_cam = R_w_body * extrinsics_[0].R;
    Eigen::Vector3d t_w_cam = t_w_body + R_w_body * extrinsics_[0].t;

    Eigen::Matrix3d optimized_R;
    Eigen::Vector3d optimized_t;

    if (solvePnP(R_w_cam, t_w_cam, pts_3d, pts_2d, optimized_R, optimized_t)){
        status.R = optimized_R;
        status.p = optimized_t;
    }

}
//input R_cam_w and t_cam_w, output R_w_cam and t_w_cam
bool StereoFeatureManager::solvePnP(Eigen::Matrix3d & init_R, Eigen::Vector3d & init_t, 
    std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d, 
    Eigen::Matrix3d &R, Eigen::Vector3d &t){

    init_R = init_R.transpose();
    init_t = -init_R * init_t;
    if (pts_2d.size() < 4){
        printf("[Warning] PnP need at least 4 points\n");
        return false;
    }
    cv::Mat r, rvec, cv_t, D, tmp_r;
    cv::eigen2cv(init_R, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(init_t, cv_t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts_3d, pts_2d, K, D, rvec, cv_t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(cv_t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    t = R * (-T_pnp);

    return true;
}

void StereoFeatureManager::initNewFramePoseByIMU(int32_t next_sldwin_index, BasicStatus &status){

}

void StereoFeatureManager::triangulateFeature(Eigen::Matrix <double, 3, 4> &pose_left, Eigen::Matrix <double, 3, 4> &pose_right, 
      Eigen::Vector2d &point_left, Eigen::Vector2d &point_right, Eigen::Vector3d &point3d){
    //SVD based 
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point_left(0) * pose_left.row(2) - pose_left.row(0);
    design_matrix.row(1) = point_left(1) * pose_left.row(2) - pose_left.row(1);
    design_matrix.row(2) = point_right(0) * pose_right.row(2) - pose_right.row(0);
    design_matrix.row(3) = point_right(1) * pose_right.row(2) - pose_right.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point3d = triangulated_point.head<3>() / triangulated_point(3);
}

//TODO: make SVD triangulation as a unified function
void StereoFeatureManager::sdvTriangulateFeature(std::list<Eigen::Matrix<double, 3, 4>> &pose, std::list<Eigen::Vector2d> &point,
     Eigen::Vector3d &point3d){
    //SVD based 
    Eigen::Matrix<double, Eigen::Dynamic, 4> design_matrix = Eigen::Matrix<double, Eigen::Dynamic, 4>::Zero(2 * pose.size(), 4);
    int svd_idx = 0;
    for (auto &pose_iter : pose){
        Eigen::Vector2d point_iter = point.front();
        point.pop_front();
        design_matrix.row(svd_idx++) = point_iter(0) * pose_iter.row(2) - pose_iter.row(0);
        design_matrix.row(svd_idx++) = point_iter(1) * pose_iter.row(2) - pose_iter.row(1);
    }
    Eigen::Vector4d triangulated_point;
    if (design_matrix.rows() ==4){
        triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    } else if (design_matrix.rows() > 4){
        triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeThinV).matrixV().rightCols<1>();
    }

    point3d = triangulated_point.head<3>() / triangulated_point(3);
    return;
}

double StereoFeatureManager::compensatedParallax2(Eigen::Vector3d & last_frame_observe, Eigen::Vector3d & new_frame_observe){
    double last_u = last_frame_observe(0);
    double last_v = last_frame_observe(1);
    double new_u = new_frame_observe(0);
    double new_v = new_frame_observe(1);
    double d_u = last_u - new_u;
    double d_v = last_v - new_v;

    double ans = 0.0;
    ans = std::max(ans, d_u * d_u + d_v * d_v);
    return ans;
}

void StereoFeatureManager::setDepth(const Eigen::VectorXd &x){

}

void StereoFeatureManager::removeInvalidDepth(){
    for (auto & feature : features_){
        if (feature.second->isDepthValid() == false){
            features_.erase(feature.first);
        }
    }
}


//this function ues in mono camera initialization
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> StereoFeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    for (auto &it : features_)
    {
        if (it.second->start_frame <= frame_count_l && it.second->endFrame() >= frame_count_r)
        {
            Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
            int idx_l = frame_count_l - it.second->start_frame;
            int idx_r = frame_count_r - it.second->start_frame;

            a = it.second->feature_per_frame[idx_l].point;
            b = it.second->feature_per_frame[idx_r].point;

            corres.push_back(std::make_pair(a, b));
        }
    }
    return corres;
}