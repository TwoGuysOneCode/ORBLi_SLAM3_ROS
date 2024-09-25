#include "stereo_lidar_node.h"

//TODO: Add pcl_ros

using json = nlohmann::json;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "StereoLidar");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;

    Common* common = new Common(node_handler);
    ORB_SLAM3::System SLAM(common->GetVocFile(), common->GetSettingsFile(), ORB_SLAM3::System::LIDAR_STEREO, common->isPangolinEnabled());
    StereoLidar node(&SLAM, common);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 50);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 50);
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&StereoLidar::GrabStereoLidar,&node,_1,_2));

    //TODO: Aggiungere il subscriber al nodo del lidar
    
    ros::spin();
    // Stop all threads
    ros::shutdown();
    return 0;
}

void StereoLidar::lidar_callback()//Aggiungere pcl
{
    //TODO: prendere i punti del lidar e trasformali in punti nell'immagine con la depth 
    //salva in locale in un unorder_map<std::pair<int, int>, float> i punti da utilizzare
    //Crea una struct che contiene la mappa e il timestamp cosi che i frame possano essere sincronizzati
}

void StereoLidar::GrabStereoLidar(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    //Controllo se i punti sono nel giusto timestamp altrimenti mando una lista vuota
    Sophus::SE3f Tcw;
    if(std::abs( ((long) lidarPoints->header.stamp.toNSec()) - ((long) msgLeft->header.stamp.toNSec())) < timeFrameLidarSync){
        Tcw = mpSLAM->TrackLidarStereo(cv_ptrLeft->image, cv_ptrRight->image, lidarPoints->lidarDepth, cv_ptrLeft->header.stamp.toSec());
    } else {
        std::unordered_map<int, float> empty;
        Sophus::SE3f Tcw = mpSLAM->TrackLidarStereo(cv_ptrLeft->image,cv_ptrRight->image, empty, cv_ptrLeft->header.stamp.toSec());
    }
    Sophus::SE3f Twc = Tcw.inverse();

    ros::Time msg_time = cv_ptrLeft->header.stamp;
    this->common->PublishRosTfTransform(Tcw, common->GetWorldFrameId(), "base_link", msg_time);
    this->common->PublishPositionAsPoseStamped(Tcw, msg_time, true);
    this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
}