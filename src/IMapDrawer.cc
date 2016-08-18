//
// Created by sebastiano on 8/18/16.
//

#include "IMapDrawer.h"

using namespace ORB_SLAM2;
using namespace std;

void IMapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

cv::Mat IMapDrawer::GetCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mCameraPose;
}

