//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_ROSPUBLISHER_H
#define ORB_SLAM2_ROSPUBLISHER_H

#include "IPublisherThread.h"
#include "IFrameSubscriber.h"
#include "IMapPublisher.h"
#include "FrameDrawer.h"
#include "System.h"
#include "Map.h"

#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <octomap/OcTree.h>

#include <orb_slam2/ORBState.h>

namespace ORB_SLAM2 
{
    class Map;
    class Tracking;
}

class ROSPublisher :
    public ORB_SLAM2::IPublisherThread,
    public ORB_SLAM2::IMapPublisher,
    public ORB_SLAM2::IFrameSubscriber
{
public:
    static constexpr const char *DEFAULT_MAP_FRAME = "/orb_slam2/world";
    static constexpr const char *DEFAULT_MAP_FRAME_ADJUSTED = "/orb_slam2/map";
    static constexpr const char *DEFAULT_CAMERA_FRAME = "/orb_slam2/camera";
    static constexpr const float DEFAULT_OCTOMAP_RESOLUTION = 0.1;
    static constexpr const float STATE_REPUBLISH_WAIT_RATE = 20;  // re-publish state @ 20 Hz

    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(
        ORB_SLAM2::Map *map,
        double frequency,
        ros::NodeHandle nh = ros::NodeHandle());

    virtual void Run() override;
    virtual void Update(ORB_SLAM2::Tracking*);

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::FrameDrawer drawer_;

    // Important: `nh_` goes before the `*_pub_`, because their construction relies on `nh_`!
    ros::NodeHandle nh_;
    std::string map_frame_name_, camera_frame_name_;
    ros::Publisher map_pub_, map_updates_pub_, image_pub_, odom_pub_, state_pub_, state_desc_pub_, octomap_pub_;
    tf::TransformBroadcaster camera_tf_pub_;
    ros::Rate pub_rate_;

    ros::Time last_state_publish_time_;
    orb_slam2::ORBState orb_state_;

    int lastBigMapChange_;
    bool octomap_tf_based_;
    octomap::OcTree octomap_;
    tf::Vector3 camera_position_;

    tf::TransformListener tf_listener_;

    void updateOctoMap();
    void integrateMapPoints(const std::vector<ORB_SLAM2::MapPoint*> &, const octomap::point3d &, const octomap::pose6d &, octomap::OcTree &);
    void publishMap();
    void publishMapUpdates();
    void publishCameraPose();
    void publishOctomap();
    void publishState(ORB_SLAM2::Tracking *tracking);
    void publishImage(ORB_SLAM2::Tracking *tracking);
};

class ROSSystemBuilder : public ORB_SLAM2::System::GenericBuilder {
public:
    ROSSystemBuilder(const std::string& strVocFile,
                     const std::string& strSettingsFile,
                     ORB_SLAM2::System::eSensor sensor,
                     double frequency,
                     ros::NodeHandle nh = ros::NodeHandle(),
                     std::string map_frame = ROSPublisher::DEFAULT_MAP_FRAME,
                     std::string camera_frame = ROSPublisher::DEFAULT_CAMERA_FRAME);
    virtual ~ROSSystemBuilder();
    
    virtual ORB_SLAM2::IPublisherThread* GetPublisher() override;

private:
    std::unique_ptr<ROSPublisher> mpPublisher;
};


#endif //ORB_SLAM2_ROSPUBLISHER_H
