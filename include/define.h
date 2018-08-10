#pragma once
#include <vector>
#include <map>
#include <string>
#include <opencv2/opencv.hpp>
namespace tracking
{
    enum class Detectors
    {
        MOTION_MOG2,
        ML_HOGSVM,
        ML_MOTION
    };

    enum class Trackers
    {
        KALMAN_FILTER
    };
    enum KalmanType
    {
        KalmanLinear = 0
    };
}

namespace color
{
   const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
   const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
   const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
   const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0); 
   const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);
}
class Region
{
public:
    Region()
    {

    }
    Region(const cv::Rect& rect) : m_rect(rect)
    {

    }
    cv::Rect m_rect;    
    std::vector<cv::Point2f> m_regions;
};
typedef float track_t;
typedef std::vector<Region> regions_t;
typedef cv::Point_<track_t> Point_t;
typedef std::map<std::string, std::string> config_t;
//std::map<std::string, cv::Scalar> colors;

struct FrameInfo
{
    cv::Mat frame;
    cv::Mat gray;
    regions_t m_regions;
    regions_t m_motionRegions;
    cv::Rect m_motionROI;
    int64 m_dt;

};
struct TrackerSetting
{
    std::string m_inFile;
    std::string m_outFile;
    size_t m_fps;
    config_t detectorSettings;
    track_t distThres = 50;
    track_t m_dt = 0.5;
    track_t m_accelNoiseMag = 0.1;
    tracking::Detectors detectorType = tracking::Detectors::MOTION_MOG2;
    tracking::Trackers trackerType = tracking::Trackers::KALMAN_FILTER;    
    tracking::KalmanType m_kalmanType = tracking::KalmanLinear;
    size_t m_maximumAllowedSkippedFrames = 25;
};