#include <unordered_map>
#include <cstdlib>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include "common.h"
#include "json.h"
#include "ImuTypes.h"

struct LidarPoints{
    std_msgs::Header header;
    std::unordered_map<int, float> lidarDepth;
};

class StereoLidar {
    public:
        StereoLidar(ORB_SLAM3::System* pSLAM, Common* comm): mpSLAM(pSLAM), common(comm){};

        /**
        * @brief callback for grabbing stereo images 
        * 
        * @param msgLeft left camera
        * @param msgRight right camera
        */
        void GrabStereoLidar(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);
        void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    
    private:
        Common* common;
        ORB_SLAM3::System* mpSLAM;
        LidarPoints* lidarPoints;
        float timeFrameLidarSync = 10; //Da capire che unità è, TODO: da mettere come parametro
        cv::Mat K;
        int camera_width;
        int camera_height;
        float baseline = 0.12f;
        string frame_input = "";
        string frame_output = "";
};