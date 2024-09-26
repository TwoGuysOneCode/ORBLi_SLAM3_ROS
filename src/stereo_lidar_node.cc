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

void StereoLidar::lidar_callback(const PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //Converti da PointCloud2 ROS a PointCloud PCL
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Creiamo un listener per ottenere le trasformazioni tf
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Aspetta che la trasformazione sia disponibile
    try {
        //listener.lookupTransform(frame_output, frame_input, ros::Time(0), transform);
        listener.waitForTransform(frame_output, frame_input, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(frame_output, frame_input, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Errore durante l'attesa della trasformazione: %s", ex.what());
        return;
    }

    tf::Transform inverseTransform = transform.inverse();

    // Creazione di una point cloud per i dati trasformati
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Applica la trasformazione da source_frame a target_frame
    try {
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform);
        ROS_INFO("Nuvola di punti trasformata con successo!");
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Errore durante la trasformazione della nuvola di punti: %s", ex.what());
        return;
    }

    //std::vector<cv::Point3d> image_points;
    lidarPoints->header = cloud_msg->header;
    lidarPoints->clear();
    for (const auto& point : transformed_cloud->points) {
        if (point.z > 0){
            cv::Mat pt_3d = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            cv::Mat pt_2d = K * pt_3d;
            // Normalizza per ottenere le coordinate (x, y)
            pt_2d /= pt_2d.at<double>(2);
            if(pt_2d.at<double>(0) >= 0 && pt_2d.at<double>(0) < camera_width)
            {
                if(pt_2d.at<double>(1) >= 0 && pt_2d.at<double>(1) < camera_height){
                    //image_points.push_back(cv::Point3d(pt_2d.at<double>(0), pt_2d.at<double>(1), abs(point.z)));
                    lidarPoint->lidarDepth[((cv::Point3d(pt_2d.at<double>(0) << 16) && 0xFFFF0000) | (pt_2d.at<double>(1) && 0x0000FFFF))] = std::abs(point.z);
                }
            }
        }
    }

    //TODO: Creare il vettore
    //Se vogliamo visualizzare l'immagine
    // for (const auto& pt : image_points) {
    //     //int color = (int)mapValue(pt.z, 0, 30, 255, 100);
    //     //Devi aggiungere dei controlli per vedere che stia nell'immagine
    //     cv::Point2d p (pt.x, pt.y);
    //     cv::Scalar c = getColor(pt.z, minElement.z, maxElement.z);
    //     //ROS_INFO("color: %f, %f, %f", c[0], c[1], c[2]);
    //     cv::circle(img1, p, 3, c, -1);
    // }

    // ROS_INFO("[lidar_to_camera_view]Pub lidar points image");
    // sensor_msgs::ImagePtr ros_left_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img1).toImageMsg();
    // left_camera_lidar_point.publish(ros_left_image);
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