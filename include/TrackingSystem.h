#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "define.h"
#include "Buff.h"
#include "Detector.h"
#include "Tracker.h"

class TrackingSystem
{
public:
    
    TrackingSystem(const cv::CommandLineParser& parser);
    ~TrackingSystem();
  //  virtual void InitTracker();
    void InitWindows();
    bool InitTracker(FrameInfo& frameInfo);
    void Process();
    void Detect(FrameInfo& frameInfo);
    void Tracking(FrameInfo& frameInfo);
    void CaptureAndDetecting(
        bool* stopCapture,
        std::mutex* frameLock, 
        std::condition_variable* frameCond
        );
    void DrawData(cv::Mat frame);  //
    void DrawData(FrameInfo& frameInfo); 
    void DrawTrack(cv::Mat frame, const Track& track);

private:
    std::unique_ptr<Detector> m_detector;
    std::unique_ptr<Detector> m_motionDetector;
    std::unique_ptr<Tracker> m_tracker;
    bool m_isInitTracker;
//    FrameInfo m_frames[2];
    TrackerSetting m_settings;
    std::shared_ptr<Buff<FrameInfo>> m_frameBuff;
 //   cv::Rect2d motionROI;
};