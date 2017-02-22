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
#include <orb_slam2/ORBState.h>
#include <cv_bridge/cv_bridge.h>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <chrono>


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


ROSPublisher::ROSPublisher(Map *map, double frequency, ros::NodeHandle nh) :
    IMapPublisher(map),
    drawer_(GetMap()),
    nh_(std::move(nh)),
    pub_rate_(frequency),
    lastBigMapChange_(-1),
    octomap_tf_based_(false),
    clear_octomap_(false),
    octomap_(ROSPublisher::DEFAULT_OCTOMAP_RESOLUTION)
{
    // initialize publishers
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 3);
    map_updates_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_updates", 3);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("frame", 5);
    state_pub_ = nh_.advertise<orb_slam2::ORBState>("state", 10);
    state_desc_pub_ = nh_.advertise<std_msgs::String>("state_description", 10);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 3);
    projected_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, 10);
    orb_state_.state = orb_slam2::ORBState::UNKNOWN;

    // initialize parameters
    nh.param<double>("occupancy_projection_min_height", projectionMinHeight_, ROSPublisher::PROJECTION_MIN_HEIGHT);
    // TODO make more params configurable

    octomap_worker_thread_ = std::thread( [this] { octomapWorker(); } );
}

/*
 * Either appends all GetReferenceMapPoints to the pointcloud stash or clears the stash and re-fills it
 * with GetAllMapPoints, in case there is a big map change in ORB_SLAM 2 or all_map_points is set to true.
 */
void ROSPublisher::stashMapPoints(bool all_map_points)
{
    std::vector<MapPoint*> map_points;

    if (all_map_points || GetMap()->GetLastBigChangeIdx() > lastBigMapChange_)
    {
        map_points = GetMap()->GetAllMapPoints();
        lastBigMapChange_ = GetMap()->GetLastBigChangeIdx();

        // TODO: temporarily disabled due to octomap clear() bug
        //clear_octomap_ = true;
    } else {
        map_points = GetMap()->GetReferenceMapPoints();
    }

    pointcloud_map_points_mutex_.lock();
    for (MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        pointcloud_map_points_.push_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    pointcloud_map_points_mutex_.unlock();
}

/*
 * Octomap worker thread function, which has exclusive access to the octomap. Updates and publishes it.
 */
void ROSPublisher::octomapWorker()
{
    static std::chrono::system_clock::time_point this_cycle_time;

    octomap::pose6d frame;
    bool got_tf;
    octomap::point3d origin;

    // wait until ORB_SLAM 2 is up and running
    while (orb_state_.state != orb_slam2::ORBState::OK)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // main thread loop
    while (!isStopped())
    {
        this_cycle_time = std::chrono::system_clock::now();

        origin = {camera_position_.x(), camera_position_.y(), camera_position_.z()};

        // try to get a TF from UAV base to camera (in ORB space)
        try {
            tf::StampedTransform transform_in_target_frame;
            tf_listener_.lookupTransform(ROSPublisher::DEFAULT_BASE_FRAME, ROSPublisher::DEFAULT_CAMERA_FRAME, ros::Time(0), transform_in_target_frame);
            frame = octomap::poseTfToOctomap(transform_in_target_frame);
            got_tf = true;
        } catch (tf::TransformException &ex) {
            frame = octomap::pose6d(0, 0, 0, 0, 0, 0);
            got_tf = false;
        }

        // TODO temporary workaround for buggy octomap
        if (!got_tf) {
            ROS_INFO_STREAM("no TF yet: skipping update to avoid clear()" << std::endl);
            continue;
        }
        //clear_octomap_ |= (got_tf != octomap_tf_based_);

        if (clear_octomap_)
        {
            clear_octomap_ = false; // TODO: mutex?
            // TODO: temporary octomap safety check
            ROS_INFO_STREAM("octomap clear requested, but this shouldn't happen: FAILING" << std::endl);
            assert(false);
            octomap_.clear(); // WARNING: causes ugly segfaults in octomap 1.8.0

            // TODO: test loop-closing properly
            // TODO: if pointcloud is supposed to be a lidar scan result, this is problematic (multiple hits on one beam/previous hits getting overwritten etc.)
        }

        pointcloud_map_points_mutex_.lock();
        octomap_.insertPointCloud(pointcloud_map_points_, origin, frame);
        pointcloud_map_points_.clear();
        pointcloud_map_points_mutex_.unlock();

        octomap_tf_based_ = got_tf;

        publishOctomap();
        publishProjectedMap();

        ROS_INFO_STREAM("Hi this is octomapWorker thread, I just finished one cycle." << std::endl);

        std::this_thread::sleep_until(this_cycle_time + std::chrono::milliseconds((int) (1000. / ROSPublisher::OCTOMAP_RATE)));
    }
}

/*
 * Creates a 2D Occupancy Grid from the Octomap.
 */
void ROSPublisher::octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_DEBUG("Octree min %f %f %f", minX, minY, minZ);
    ROS_DEBUG("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    if (!octree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!octree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    map.info.width = maxKey[0] - minKey[0] + 1;
    map.info.height = maxKey[1] - minKey[1] + 1;

    // might not exactly be min / max:
    octomap::point3d origin =   octree.keyToCoord(minKey, octree.getTreeDepth());
    map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5;
    map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5;

    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height, -1);

    //init with unknown
    for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
      *it = -1;
    }

    // iterate over all keys:
    unsigned i, j;
    for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
    {
        for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
        {
            for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
            { //iterate over height
                octomap::OcTreeNode* node = octree.search(curKey);
                if (node)
                {
                    bool occupied = octree.isNodeOccupied(node);
                    if(occupied) {
                        map.data[map.info.width * j + i] = 100;
                        break;
                    } else {
                        map.data[map.info.width * j + i] = 0;
                    }
                }
            }
        }
    }
}

static const char *stateDescription(orb_slam2::ORBState orb_state)
{
    switch (orb_state.state) {
        case orb_slam2::ORBState::SYSTEM_NOT_READY: return "System not ready";
        case orb_slam2::ORBState::NO_IMAGES_YET: return "No images yet";
        case orb_slam2::ORBState::NOT_INITIALIZED: return "Not initialized";
        case orb_slam2::ORBState::OK: return "OK";
        case orb_slam2::ORBState::LOST: return "Tracking lost";
    }

    return "???";
}

static const orb_slam2::ORBState toORBStateMessage(Tracking::eTrackingState trackingState)
{
    orb_slam2::ORBState state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.state = orb_slam2::ORBState::UNKNOWN;

    switch (trackingState) {
        case Tracking::SYSTEM_NOT_READY: state_msg.state = orb_slam2::ORBState::SYSTEM_NOT_READY;
                                         break;
        case Tracking::NO_IMAGES_YET:    state_msg.state = orb_slam2::ORBState::NO_IMAGES_YET;
                                         break;
        case Tracking::NOT_INITIALIZED:  state_msg.state = orb_slam2::ORBState::NOT_INITIALIZED;
                                         break;
        case Tracking::OK:               state_msg.state = orb_slam2::ORBState::OK;
                                         break;
        case Tracking::LOST:             state_msg.state = orb_slam2::ORBState::LOST;
                                         break;
    }

    return state_msg;
}

/*
 * Publishes ORB_SLAM 2 GetAllMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMap()
{
    if (map_pub_.getNumSubscribers() > 0)
    {
        auto msg = convertToPCL2(GetMap()->GetAllMapPoints());
        msg.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME;
        map_pub_.publish(msg);
    }
}

/*
 * Publishes ORB_SLAM 2 GetReferenceMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMapUpdates()
{
    if (map_updates_pub_.getNumSubscribers() > 0)
    {
        auto msg = convertToPCL2(GetMap()->GetReferenceMapPoints());
        msg.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME;
        map_updates_pub_.publish(msg);
    }
}

/*
 * Publishes ORB_SLAM 2 GetCameraPose() as a TF.
 */
void ROSPublisher::publishCameraPose()
{
    // number of subscribers is unknown to a TransformBroadcaster

    cv::Mat xf = computeCameraTransform(GetCameraPose());
    if (!xf.empty()) {
        camera_position_ = { xf.at<float>(0, 3), xf.at<float>(1, 3), xf.at<float>(2, 3) };
        auto orientation = convertToQuaternion<tf::Quaternion>(xf);
        tf::StampedTransform transform(
            tf::Transform(orientation, camera_position_),
            ros::Time::now(), ROSPublisher::DEFAULT_MAP_FRAME, ROSPublisher::DEFAULT_CAMERA_FRAME);
        camera_tf_pub_.sendTransform(transform);
        ResetCamFlag();
    }
}

/*
 * Publishes the previously built Octomap.
 */
void ROSPublisher::publishOctomap()
{
    if (octomap_pub_.getNumSubscribers() > 0)
    {
        auto t0 = std::chrono::system_clock::now();
        octomap_msgs::Octomap msgOctomap;
        msgOctomap.header.frame_id = octomap_tf_based_ ?
                                     ROSPublisher::DEFAULT_MAP_FRAME_ADJUSTED :
                                     ROSPublisher::DEFAULT_MAP_FRAME;
        msgOctomap.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(octomap_, msgOctomap))   // TODO: full/binary...?
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

/*
 * Publishes the ORB_SLAM 2 tracking state as ORBState int and/or as a description string.
 */
void ROSPublisher::publishState(Tracking *tracking)
{
    if (state_pub_.getNumSubscribers() > 0)
    {
        // publish state as ORBState int
        if (tracking == NULL)
        {
            // re-publish old state
            orb_state_.header.stamp = ros::Time::now();
        } else {
            // get state from tracking
            orb_state_ = toORBStateMessage(tracking->mState);
        }
        state_pub_.publish(orb_state_);
    }
    if (state_desc_pub_.getNumSubscribers() > 0)
    {
        // publish state as string
        std_msgs::String state_desc_msg;
        state_desc_msg.data = stateDescription(orb_state_);
        state_desc_pub_.publish(state_desc_msg);
    }

    last_state_publish_time_ = ros::Time::now();
}

/*
 * Publishes the current ORB_SLAM 2 status image.
 */
void ROSPublisher::publishImage(Tracking *tracking)
{
    if (image_pub_.getNumSubscribers() > 0)
    {
        drawer_.Update(tracking);

        std_msgs::Header hdr;
        cv_bridge::CvImage cv_img {hdr, "bgr8", drawer_.DrawFrame()};

        auto image_msg = cv_img.toImageMsg();
        image_msg->header = hdr;
        image_pub_.publish(*image_msg);
    }
}

/*
 * Creates a 2D Occupancy Grid from the Octomap and publishes it.
 */
void ROSPublisher::publishProjectedMap()
{
  static nav_msgs::OccupancyGrid msgOccupancy;
  if (projected_map_pub_.getNumSubscribers() > 0)
  {
    msgOccupancy.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME_ADJUSTED;
    msgOccupancy.header.stamp = ros::Time::now();

    octomapToOccupancyGrid(octomap_, msgOccupancy, ROSPublisher::PROJECTION_MIN_HEIGHT, std::numeric_limits<double>::max());

    projected_map_pub_.publish(msgOccupancy);

  }
}

void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    SetFinish(false);

    ROS_INFO("ROS publisher started");

    while (WaitCycleStart()) {
        // only publish map, map updates and camera pose, if camera pose was updated
        // TODO: maybe there is a way to check if the map was updated
        if (isCamUpdated()) {
            publishMap();
            publishMapUpdates();
            publishCameraPose();

            stashMapPoints(); // store current reference map points for the octomap worker
        }
        if (ros::Time::now() >= last_state_publish_time_ + ros::Duration(1. / STATE_REPUBLISH_WAIT_RATE))
        {
            publishState(NULL);
        }
    }

    ROS_INFO("ROS publisher finished");
    SetFinish(true);
}

bool ROSPublisher::WaitCycleStart()
{
    //std::cout << "in WaitCycleStart" << std::endl;
    if (!IPublisherThread::WaitCycleStart())
        return false;

    pub_rate_.sleep();
    return true;
}

void ROSPublisher::Update(Tracking *tracking)
{
    static std::mutex mutex;

    if (tracking == nullptr)
        return;

    publishState(tracking);

    // TODO: Make sure the camera TF is correctly aligned. See:
    // <http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html>

    publishImage(tracking);
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
        GetMap(), frequency, std::move(nh));
    mpTracker->SetFrameSubscriber(mpPublisher.get());
    mpTracker->SetMapPublisher(mpPublisher.get());
}

// Empty dtor to give a place to the calls to the dtor of unique_ptr members
ROSSystemBuilder::~ROSSystemBuilder() { }

IPublisherThread* ROSSystemBuilder::GetPublisher()
{
    return mpPublisher.get();
}
