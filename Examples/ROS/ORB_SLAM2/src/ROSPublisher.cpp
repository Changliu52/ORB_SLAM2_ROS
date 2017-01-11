//
// Created by sebastiano on 8/18/16.
//

#include "ROSPublisher.h"
#include "FrameDrawer.h"
#include "Tracking.h"
#include "utils.h"

#include <thread>
#include <sstream>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// #include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <chrono>


#define DEFAULT_OCTOMAP_RESOLUTION 0.1


using namespace ORB_SLAM2;


static bool isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

static const bool IS_BIG_ENDIAN = isBigEndian();

namespace std {
    std::string to_string(const cv::Mat& mat) {
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }
}

template<typename Q>
Q convertToQuaternion(const cv::Mat& rot)
{
    double trace = rot.at<float>(0,0) + rot.at<float>(1,1) + rot.at<float>(2,2);
    double tmp[4];

    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        tmp[3] = s * 0.5;
        s = 0.5 / s;
        tmp[0] = ((rot.at<float>(2,1) - rot.at<float>(1,2)) * s);
        tmp[1] = ((rot.at<float>(0,2) - rot.at<float>(2,0)) * s);
        tmp[2] = ((rot.at<float>(1,0) - rot.at<float>(0,1)) * s);
    } else {
        int i;
        if (rot.at<float>(0, 0) < rot.at<float>(1,1))
            i = rot.at<float>(1,1) < rot.at<float>(2,2) ? 2 : 1;
        else
            i = rot.at<float>(0,0) < rot.at<float>(2,2) ? 2 : 0;
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(rot.at<float>(i,i) - rot.at<float>(j,j) - rot.at<float>(k,k) + 1.0);
        tmp[i] = s * 0.5;
        s = 0.5 / s;
        tmp[3] = (rot.at<float>(k,j) - rot.at<float>(j,k)) * s;
        tmp[j] = (rot.at<float>(j,i) + rot.at<float>(i,j)) * s;
        tmp[k] = (rot.at<float>(k,i) + rot.at<float>(i,k)) * s;
    }

    return {tmp[0], tmp[1], tmp[2], tmp[3]};
}

cv::Mat computeCameraTransform(const cv::Mat& Twc)
{
    cv::Mat ret = cv::Mat::eye(4, 4, CV_32F);

    if(!Twc.empty()) {
        auto Rwc = Twc.rowRange(0,3).colRange(0,3).t();
        ret.rowRange(0,3).colRange(0,3) = Rwc;
        // twc, the position
        ret.rowRange(0,3).col(3) = -Rwc* Twc.rowRange(0, 3).col(3);
    }
    return ret;
}

sensor_msgs::PointCloud2 convertToPCL2(const std::vector<MapPoint*> &map_points)
{
    const std::size_t n_map_points = map_points.size();
    ROS_INFO("sending PointCloud (%lu points)", n_map_points);

    // Kind of a hack, but there aren't much better ways to avoid a copy
    struct point { float x, y, z; };

    std::vector<uint8_t> data_buffer(n_map_points * sizeof(point));
    std::size_t vtop = 0;

    point *dataptr = (point*) data_buffer.data();

    for (MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        dataptr[vtop++] = {
            pos.at<float>(0),
            pos.at<float>(1),
            pos.at<float>(2),
        };
    }

    static const char* const names[3] = { "x", "y", "z" };
    static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    std::vector<sensor_msgs::PointField> fields(3);
    for (int i=0; i < 3; i++) {
        fields[i].name = names[i];
        fields[i].offset = offsets[i];
        fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        fields[i].count = 1;
    }

    sensor_msgs::PointCloud2 msg;
    msg.height = 1;
    msg.width = n_map_points;
    msg.fields = fields;
    msg.is_bigendian = IS_BIG_ENDIAN;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;  // invalid points already filtered out

    return msg;
}

/*
 * Integrates a `map_points` scan originating from camera position `origin` into an `octomap`.
*/
void ROSPublisher::integrateMapPoints(const std::vector<MapPoint*> &map_points, octomap::point3d origin, octomap::OcTree &octomap)
{
    // first convert map points to point cloud
    octomap::Pointcloud cloud;
    for (MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        cloud.push_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }

    // integrate point cloud into octomap
    try {
        // with transformation to camera frame
        tf::StampedTransform transform_in_target_frame;
        tf_listener_.lookupTransform("ORB_base_link", camera_frame_name_, ros::Time(0) , transform_in_target_frame);
        octomap::pose6d frame = octomap::poseTfToOctomap(transform_in_target_frame);

        octomap.insertPointCloud(cloud, origin, frame);
    } catch (tf::TransformException &ex) {
        // without transformation
        octomap.insertPointCloud(cloud, origin);
    }
}


ROSPublisher::ROSPublisher(ORB_SLAM2::Map *map, double frequency,
                std::string map_frame, std::string camera_frame, ros::NodeHandle nh) :
    IMapPublisher(map),
    drawer_(GetMap()),
    nh_(std::move(nh)),
    map_frame_name_(std::move(map_frame)),
    camera_frame_name_(std::move(camera_frame)),
    pub_rate_(frequency),
    lastBigMapChange_(-1),
    octomap_(DEFAULT_OCTOMAP_RESOLUTION)
{
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 5);
    map_updates_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_updates", 2);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("frame", 5);
    status_pub_ = nh_.advertise<std_msgs::String>("status", 5);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 3);
}

/*
 * Either integrates the current GetReferenceMapPoints into the octomap or rebuilds
 * the entire octomap from GetAllMapPoints, if there is a big change in the ORB_SLAM map.
*/
bool ROSPublisher::updateOctoMap()
{
    //std::cout << "GetMap()->GetLastBigChangeIdx() = " << GetMap()->GetLastBigChangeIdx() << std::endl;
    if (GetMap()->GetLastBigChangeIdx() > lastBigMapChange_) {
        // big map change: rebuild map completely
        octomap_.clear(); // TODO: confirm functionality!
        integrateMapPoints(GetMap()->GetAllMapPoints(), octomap::point3d(), octomap_);
        lastBigMapChange_ = GetMap()->GetLastBigChangeIdx();
        //std::cout << "rebuilt map" << std::endl;
        return true;
    } else {
        octomap::point3d origin = {camera_position_.x(), camera_position_.y(), camera_position_.z()};
        //std::cout << camera_position_.x() << " " << camera_position_.y() << " " << camera_position_.z() << std::endl;
        auto t0 = std::chrono::system_clock::now();
        integrateMapPoints(GetMap()->GetReferenceMapPoints(), origin, octomap_);
        auto tn = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
        //std::cout << "integrateMapPoints time: " << dt.count() << " ms" << std::endl;
        return false;
    }
}

void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    SetFinish(false);

    ROS_INFO("ROS publisher started");

    std_msgs::Header map_hdr;
    map_hdr.frame_id = map_frame_name_;

    while (WaitCycleStart()) {
        // only publish map, map updates and camera pose, if camera pose was updated
        // TODO: maybe there is a way to check if the map was updated
        if (isCamUpdated()) {
            // map
            {
                auto msg = convertToPCL2(GetMap()->GetAllMapPoints());
                msg.header = map_hdr;
                map_pub_.publish(msg);
            }
            // map updates
            {
                auto msg = convertToPCL2(GetMap()->GetReferenceMapPoints());
                msg.header = map_hdr;
                map_updates_pub_.publish(msg);
            }
            // camera pose
            cv::Mat xf = computeCameraTransform(GetCameraPose());
            if (!xf.empty()) {
                camera_position_ = { xf.at<float>(0, 3), xf.at<float>(1, 3), xf.at<float>(2, 3) };
                auto orientation = convertToQuaternion<tf::Quaternion>(xf);
                tf::StampedTransform transform(
                    tf::Transform(orientation, camera_position_),
                    ros::Time::now(), map_frame_name_, camera_frame_name_);
                camera_tf_pub_.sendTransform(transform);
                ResetCamFlag();
            }
            // octomap
            {
                updateOctoMap();
                auto t0 = std::chrono::system_clock::now();
                octomap_msgs::Octomap msgOctomap;
                msgOctomap.header.frame_id = map_frame_name_;
                msgOctomap.header.stamp = ros::Time::now();
                if (octomap_msgs::fullMapToMsg(octomap_, msgOctomap))
                {
                    auto tn = std::chrono::system_clock::now();
                    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
                    //std::cout << "msg generation time: " << dt.count() << " ms" << std::endl;
                    t0 = std::chrono::system_clock::now();
                    octomap_pub_.publish(msgOctomap);
                    tn = std::chrono::system_clock::now();
                    dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
                    //std::cout << "msg publish time: " << dt.count() << " ms" << std::endl;
                }
            }
        }
    }

    ROS_INFO("ROS publisher finished");
    SetFinish(true);
}

bool ROSPublisher::WaitCycleStart()
{
    if (!IPublisherThread::WaitCycleStart())
        return false;

    pub_rate_.sleep();
    return true;
}

static const char *stateDescription(Tracking::eTrackingState trackingState)
{
    switch (trackingState) {
    case Tracking::SYSTEM_NOT_READY: return "System not ready";
    case Tracking::NO_IMAGES_YET: return "No images yet";
    case Tracking::NOT_INITIALIZED: return "Not initialized";
    case Tracking::OK: return "Ok";
    case Tracking::LOST: return "Tracking lost";
    }

    return "???";
}

void ROSPublisher::Update(Tracking *tracking)
{
    using namespace cv_bridge;
    static std::mutex mutex;

    if (tracking == nullptr)
        return;

    std_msgs::String status_msg;
    status_msg.data = stateDescription(tracking->mLastProcessedState);
    status_pub_.publish(status_msg);

    drawer_.Update(tracking);

    // TODO: Make sure the camera TF is correctly aligned. See:
    // <http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html>

    std_msgs::Header hdr;
    CvImage cv_img {hdr, "bgr8", drawer_.DrawFrame()};

    auto image_msg = cv_img.toImageMsg();
    image_msg->header = hdr;
    image_pub_.publish(*image_msg);
}

ROSSystemBuilder::ROSSystemBuilder(const std::string& strVocFile,
                const std::string& strSettingsFile,
                ORB_SLAM2::System::eSensor sensor,
                double frequency,
                ros::NodeHandle nh,
                std::string map_frame,
                std::string camera_frame) :
    System::GenericBuilder(strVocFile, strSettingsFile, sensor)
{
    mpPublisher = make_unique<ROSPublisher>(
        GetMap(), frequency, std::move(map_frame), std::move(camera_frame), std::move(nh));
    mpTracker->SetFrameSubscriber(mpPublisher.get());
    mpTracker->SetMapPublisher(mpPublisher.get());
}

// Empty dtor to give a place to the calls to the dtor of unique_ptr members
ROSSystemBuilder::~ROSSystemBuilder() { }

IPublisherThread* ROSSystemBuilder::GetPublisher()
{
    return mpPublisher.get();
}
