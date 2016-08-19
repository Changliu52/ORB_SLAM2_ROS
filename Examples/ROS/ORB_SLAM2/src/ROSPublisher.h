//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_ROSPUBLISHER_H
#define ORB_SLAM2_ROSPUBLISHER_H

#include "IPublisherThread.h"

#include <chrono>

#include <ros/ros.h>

namespace ORB_SLAM2 { class Map; }


class ROSPublisher : public ORB_SLAM2::IPublisherThread
{
public:
    static constexpr const char *DEFAULT_TF_NAME = "map";
    static constexpr const char *DEFAULT_TOPIC = "map_updates";
    
    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(ORB_SLAM2::Map* map,
			  double frequency,
			  std::string tf_name = DEFAULT_TF_NAME,
			  std::string topic = DEFAULT_TOPIC);
    
    virtual void Run() override;

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::Map *map_;
    // Important: `nh_` goes before `pub_`, because the construction
    // of `pub_` relies on `nh_`!
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate pub_rate_;
    std::string tf_name_;
};


#endif //ORB_SLAM2_ROSPUBLISHER_H
