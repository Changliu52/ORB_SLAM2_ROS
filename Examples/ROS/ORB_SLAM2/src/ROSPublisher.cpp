//
// Created by sebastiano on 8/18/16.
//

#include "ROSPublisher.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "Tracking.h"
#include "utils.h"

#include <thread>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

using namespace ORB_SLAM2;


/// TODO Make `ROSPublisher` implement `IMapPublisher`


ROSPublisher::ROSPublisher(ORB_SLAM2::Map *map, double frequency, std::string tf_name, std::string ns) :
    IMapPublisher(map),
    drawer_(GetMap()),
    nh_(ns),
    map_pub_(nh_.advertise<sensor_msgs::PointCloud>("map_updates", 5)),
    frame_pub_(nh_.advertise<sensor_msgs::Image>("frame", 5)),
    status_pub_(nh_.advertise<std_msgs::String>("status", 5)),
    camera_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("pose", 5)),
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

    std_msgs::Header hdr;
    hdr.frame_id = tf_name_;
    
    while (WaitCycleStart()) {
	sensor_msgs::PointCloud msg;
        msg.header = hdr;

        const auto &map_points = GetMap()->GetAllMapPoints();
	ROS_INFO("sending PointCloud (%lu points)", map_points.size());
	
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
	
	cv::Mat xf = GetCameraPose();
	std::stringstream ss;
	ss << xf;
	ROS_INFO("Transform:\n%s\n", ss.str().c_str());
	if (!xf.empty()) {
	    geometry_msgs::PoseStamped pose_msg;
	    pose_msg.header = hdr;
	    pose_msg.pose.position.x = xf.at<double>(0, 3);
	    pose_msg.pose.position.y = xf.at<double>(1, 3);
	    pose_msg.pose.position.z = xf.at<double>(2, 3);
	    double w = sqrt(1.0 + xf.at<double>(0, 0) + xf.at<double>(1, 1) + xf.at<double>(2, 2)) * 0.5;
	    pose_msg.pose.orientation.w = w;
	    pose_msg.pose.orientation.x = (xf.at<double>(2, 1) - xf.at<double>(1, 2)) / (4 * w);
	    pose_msg.pose.orientation.y = (xf.at<double>(0, 2) - xf.at<double>(2, 0)) / (4 * w);
	    pose_msg.pose.orientation.z = (xf.at<double>(1, 0) - xf.at<double>(0, 1)) / (4 * w);
	    camera_pose_pub_.publish(pose_msg);
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
    frame_pub_.publish(*image_msg);
}

ROSSystemBuilder::ROSSystemBuilder(const std::string& strVocFile,
				   const std::string& strSettingsFile,
				   ORB_SLAM2::System::eSensor sensor,
				   double frequency, std::string tf_name, std::string ns) :
    System::GenericBuilder(strVocFile, strSettingsFile, sensor)
{
    mpPublisher = make_unique<ROSPublisher>(GetMap(), frequency, std::move(tf_name), std::move(ns));
    mpTracker->SetFrameSubscriber(mpPublisher.get());
    mpTracker->SetMapPublisher(mpPublisher.get());
}

// Empty dtor to give a place to the calls to the dtor of unique_ptr members
ROSSystemBuilder::~ROSSystemBuilder() { }

IPublisherThread* ROSSystemBuilder::GetPublisher()
{
    return mpPublisher.get();
}


