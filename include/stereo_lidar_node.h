#include <unordered_map>
#include <cstdlib>

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
        void lidar_callback(); //TODO: Aggiungere i parametri
    
    private:
        Common* common;
        ORB_SLAM3::System* mpSLAM;
        LidarPoints* lidarPoints;
        float timeFrameLidarSync = 10; //Da capire che unità è
};