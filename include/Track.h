#pragma once
#include "define.h"
#include "Kalman.h"
class Track
{
public:
    Track(const Region& region,
            tracking::KalmanType kalmanType,
            float deltaTime,
            float accelNoiseMag,
            size_t trackID
            );
    ~Track();
    track_t CalcDist(const Point_t& pt) const;
    track_t CalcDist(const cv::Rect& r) const;
    void CalcDistFPrevios(const Point_t& pt);
    void Update(const Region& region, bool dataCorrect, cv::Mat currFrame);
    Point_t GetPredictedPoint() const;
    cv::Rect GetLastRect() const;
    void PointUpdate(const Point_t& pt, bool dataCorrect, const cv::Size& frameSize);
    size_t m_trackID;
    size_t m_skippedFrames;
    Region m_lastRegion;
    bool isDisplay;
    track_t m_disFromPrevios;

private:
    Point_t m_predictionPoint;
    TKalmanFilter* m_kalman;
    bool m_outOfTheFrame;
};
typedef std::vector<std::unique_ptr<Track>> tracks_t;
