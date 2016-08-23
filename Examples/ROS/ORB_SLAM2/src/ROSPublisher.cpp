//
// Created by sebastiano on 8/18/16.
//

#include "ROSPublisher.h"
#include "Map.h"
#include "Tracking.h"

#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

using namespace ORB_SLAM2;


ROSPublisher::ROSPublisher(Map *map, IFrameDrawer *frameDrawer,
                           double frequency, std::string tf_name, std::string ns) :
    map_(map),
    frameDrawer_(frameDrawer)
    nh_(ns),
    map_pub_(nh_.advertise<sensor_msgs::PointCloud>("map_updates", 5)),
    frame_pub_(nh_.advertise<sensor_msgs::Image>("frame", 5)),
    status_pub_(nh_.advertise<std_msgs::String>("status", 5)),
    pub_rate_(frequency),
    tf_name_(std::move(tf_name))
{
    ROS_INFO("Publishing on namespace `%s`/{map_updates, frame, status}", ns.c_str());
}

void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    SetFinish(false);

    ROS_INFO("ROS publisher started");

    while (WaitCycleStart()) {
        sensor_msgs::PointCloud msg;
        msg.header.frame_id = tf_name_;

        const auto &map_points = map_->GetAllMapPoints();
        for (MapPoint *map_point: map_points) {
            if (map_point->isBad())
                continue;

            cv::Mat pos = map_point->GetWorldPos();
            geometry_msgs::Point32 msg_point;
            msg_point.x = pos.at<float>(0);
            msg_point.y = pos.at<float>(1);
            msg_point.z = pos.at<float>(2);
            msg.points.push_back(msg_point);
        }

        map_pub_.publish(msg);
    }

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

    // TODO: Make sure the camera TF is correctly aligned. See:
    // <http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html>

    CvImagePtr cv_img;
    std_msgs::Header hdr;
    hdr.frame_id = tf_name_;

    {
        std::unique_lock<std::mutex> lk{mutex};
        cv_img = CvImagePtr {new CvImage(hdr, "mono8", tracking->mImGray)};
    }

    auto image_msg = cv_img->toImageMsg();
    frame_pub_.publish(*image_msg);
}
