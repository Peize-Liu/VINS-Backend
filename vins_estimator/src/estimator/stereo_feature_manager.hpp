// this is the new feature manager realized by std::map
#ifndef RS_FEATURE_MANAGER_H
#define RS_FEATURE_MANAGER_H

#include <map>
#include <vector>
#include <deque>
#include <memory>
#include <algorithm>
#include <eigen3/Eigen/Dense>

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Eigen::Vector3d point, pointRight;
    Eigen::Vector2d uv, uvRight;
    Eigen::Vector2d velocity, velocityRight;
    bool is_stereo;
};

class FeaturePerId
{
  public:
    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(FEATURE_DEPTH_UNINITIALIZED){
    }

    ~FeaturePerId(){
    }

    int startFrame(){
        //return first observation in sldwind index
        return start_frame;
    }

    int endFrame(){
        //return last observation in sldwind index
        return start_frame + feature_per_frame.size() - 1;
    }

    bool isOutlier(){
        //check if the feature is outlier
        return solve_flag == FEATURE_DEPTH_INVALID;
    }

    bool isDepthValid(){
        //check if the feature depth is valid
        return solve_flag == FEATURE_DEPTH_VALID;
    }

    typedef enum {
      FEATURE_DEPTH_UNINITIALIZED = 0,
      FEATURE_DEPTH_VALID = 1,
      FEATURE_DEPTH_INVALID = 2,
    } SolveFlag;
    
    const int feature_id;
    int start_frame; //this is the sld_window index of the first frame this feature observed
    std::vector<FeaturePerFrame> feature_per_frame; // and then the feature is observed in the following frames
    int used_num;
    double estimated_depth;
    SolveFlag solve_flag = FEATURE_DEPTH_UNINITIALIZED ; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
};

class StereoFeatureManager{
  public:
    struct StereoFeatureManagerParams{
      int32_t long_track_threshold;
      double last_track_threshold = 0.5; // between [0,1]
      double min_parallax = 0.1; // pix/focal_length
    };
    StereoFeatureManager(StereoFeatureManagerParams params);
    ~StereoFeatureManager();

    //clear all features
    bool clearFeatures(); 
    // check new frame is keyframe or not
    bool isKeyFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &features);
    // add feaures and record the first observation position of the feature in the sliding window
    bool addFeatures(int sldwin_index, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &features, double td);

    // get Corresponding features in two frames 
    // this function will be called in SFM
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //this would set optimized depth to the features ; ceres optimization will call this function
    void setDepth(const Eigen::VectorXd &x); 
    // remove the features that are not valid
    void removeInvalidDepth();
    void clearDepth();
    // get the depth vector of the features
    void triangulateFrame();

    //remove marginalized feature depth and set a new depth
    void removeBackShiftDepth();

    //remove fisrt observation
    void removeBack();

    void removeFront();
    void removeOutlier();

    //check parallax with last frame
    double compensatedParallax2(Eigen::Vector3d & last_frame_observe, Eigen::Vector3d & new_frame_observe);

  private:
    //this function is called by triangulateFrame
    void triangulatePoint();

    std::map <int, std::shared_ptr<FeaturePerId>> features_;

    StereoFeatureManagerParams params_;

};


#endif