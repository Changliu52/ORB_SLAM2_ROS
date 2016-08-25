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

#include <chrono>
#include <mutex>

#include <ros/ros.h>

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
    static constexpr const char *DEFAULT_TF_NAME = "map";
    static constexpr const char *DEFAULT_NAMESPACE = "ORB_SLAM2";

    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(ORB_SLAM2::Map *map, 
                          double frequency,
                          std::string tf_name = DEFAULT_TF_NAME,
                          std::string ns = DEFAULT_NAMESPACE);
    
    virtual void Run() override;
    virtual void Update(ORB_SLAM2::Tracking*);

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::FrameDrawer drawer_;

    // Important: `nh_` goes before the `*_pub_`, because their construction relies on `nh_`!
    ros::NodeHandle nh_;
    ros::Publisher map_pub_, frame_pub_, status_pub_, camera_pose_pub_;
    ros::Rate pub_rate_;
    std::string tf_name_;
};

class ROSSystemBuilder : public ORB_SLAM2::System::GenericBuilder {
public:
    ROSSystemBuilder(const std::string& strVocFile,
                     const std::string& strSettingsFile,
                     ORB_SLAM2::System::eSensor sensor,
                     double frequency,
                     std::string tf_name = ROSPublisher::DEFAULT_TF_NAME,
                     std::string ns = ROSPublisher::DEFAULT_NAMESPACE);
    virtual ~ROSSystemBuilder();
    
    virtual ORB_SLAM2::IPublisherThread* GetPublisher() override;

private:
    std::unique_ptr<ROSPublisher> mpPublisher;
};


#endif //ORB_SLAM2_ROSPUBLISHER_H
