// this is the new feature manager realized by std::map
#include "vins_estimator/src/estimator/stereo_feature_manager.hpp"

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
bool StereoFeatureManager::isKeyFrame(int32_t sldwin_index ,const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &input_features){
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

    if (sldwin_index < 2 || last_track_num < 20 || long_track_num < 40 ||
        new_feature_num > (last_track_num * params_.last_track_threshold)){
        return true;
    }

    //check the paralaax with the last frame
    for (auto & new_feature : input_features){
       auto  feature_iter = features_.find(new_feature.first);
       if (feature_iter != features_.end()){
            if(feature_iter->second->start_frame <= sldwin_index - 2 && 
                feature_iter->second->start_frame + int32_t(feature_iter->second->feature_per_frame.size()) >= sldwin_index - 1){
                auto last_frame_observe = feature_iter->second->feature_per_frame[sldwin_index - 1 - feature_iter->second->start_frame];
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
        if (feature.second.size() == 2){
            feature_per_frame.rightObservation(feature.second[1].second);
        }
        int32_t feature_id = feature.first;
        auto it = features_.find(feature_id);
        if (it == features_.end()){
            //new feature
            features_[feature_id] = std::make_shared<FeaturePerId>(feature_id, sldwin_index);
            features_[feature_id]->feature_per_frame.push_back(feature_per_frame);
        } else {
            //old feature
            it->second->feature_per_frame.push_back(feature_per_frame);
        }
    }
    return true;
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
            Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero;
            int idx_l = frame_count_l - it.second->start_frame;
            int idx_r = frame_count_r - it.second->start_frame;

            a = it.second->feature_per_frame[idx_l].point;
            b = it.second->feature_per_frame[idx_r].point;

            corres.push_back(std::make_pair(a, b));
        }
    }
    return corres;
}