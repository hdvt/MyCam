#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "define.h"
#include "Buff.h"
#include "DetectorFactory.h"
#include "Tracker.h"
#include "VideoProcessing.h"
class TrackingSystem
{
public:
    
    TrackingSystem();
    ~TrackingSystem();
    bool Init(const cv::CommandLineParser& parser);
    //void InitWindows();
    bool InitTracker(const FrameInfo& frameInfo);
    void Start();
    void Tracking(FrameInfo& frameInfo);
    void Detecting(
        std::mutex* frameLock, 
        std::condition_variable* frameCond
        );
    void DrawData(cv::Mat frame);  //
    void DrawData(FrameInfo& frameInfo); 
    void DrawTrack(cv::Mat frame, const Track& track);
    void SetSetting(const cv::CommandLineParser& parser);
private:
    std::unique_ptr<Detector> m_detector;
    std::unique_ptr<Tracker> m_tracker;
    VideoProcessing m_videoProcessor;
    bool m_isInitTracker;
    TrackerSetting m_settings;
    std::shared_ptr<Buff<FrameInfo>> m_frameBuff;
};