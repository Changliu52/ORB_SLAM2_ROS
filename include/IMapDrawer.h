//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_IMAPDRAWER_H
#define ORB_SLAM2_IMAPDRAWER_H

#include "Map.h"

#include <mutex>

namespace ORB_SLAM2
{

class IMapDrawer
{
public:
    IMapDrawer(Map* map) : mpMap(map) { }

    void SetCurrentCameraPose(const cv::Mat &Tcw);
    cv::Mat GetCameraPose();
    Map *GetMap() { return mpMap; }

private:
    cv::Mat mCameraPose;
    Map* mpMap;
    std::mutex mMutexCamera;
};

}


#endif //ORB_SLAM2_IMAPDRAWER_H
