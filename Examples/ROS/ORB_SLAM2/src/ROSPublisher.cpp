//
// Created by sebastiano on 8/18/16.
//

#include "ROSPublisher.h"
#include "Map.h"

#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

using namespace ORB_SLAM2;

ROSPublisher::ROSPublisher(Map *map, double frequency, std::string tf_name, std::string topic) :
    map_(map),
    pub_(nh_.advertise<sensor_msgs::PointCloud>(std::move(topic), 5)),
    pub_rate_(frequency),
    tf_name_(std::move(tf_name))
{
    ROS_INFO("Publishing map updates on %s", topic.c_str());
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

        pub_.publish(msg);
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
