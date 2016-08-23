//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_ROSPUBLISHER_H
#define ORB_SLAM2_ROSPUBLISHER_H

#include "IPublisherThread.h"
#include "IFrameSubscriber.h"

#include <chrono>

#include <ros/ros.h>

namespace ORB_SLAM2 
{
    class Map;
    class Tracking;
    class IFrameDrawer;
}


class ROSPublisher :
    public ORB_SLAM2::IPublisherThread,
    public ORB_SLAM2::IFrameDrawer
{
public:
    static constexpr const char *DEFAULT_TF_NAME = "map";
    static constexpr const char *DEFAULT_NAMESPACE = "ORB_SLAM2";

    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(ORB_SLAM2::Map *map,
                          double frequency,
                          std::string tf_name = DEFAULT_TF_NAME,
                          std::string ns = DEFAULT_NAMESPACE);
    
    virtual void Run() override;

    virtual void Update(ORB_SLAM2::Tracking *tracker) override;

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::Map *map_;
	ORB_SLAM2::IFrameDrawer *frameDrawer_;
    // Important: `nh_` goes before `pub_`, because the construction
    // of `pub_` relies on `nh_`!
    ros::NodeHandle nh_;
    ros::Publisher map_pub_, frame_pub_, status_pub_;
    ros::Rate pub_rate_;
    std::string tf_name_;
};


#endif //ORB_SLAM2_ROSPUBLISHER_H
