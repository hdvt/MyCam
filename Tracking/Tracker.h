#pragma once
#include "define.h"
#include "HungarianAlg.h"
#include "Track.h"
class Tracker
{
public:
    Tracker(const TrackerSetting& settings);
    ~Tracker();
//    void Update(const regions_t& regions, cv::Mat grayFrame);
    void Update(FrameInfo &frameInfo);
 //   void Matching(const regions_t regions, cv::Mat grayFrame);
    tracks_t m_tracks;
private:
    TrackerSetting m_settings;
    size_t m_nextTrackID;
    cv::Mat m_prevFrame;
};