#pragma once
#include "define.h"
#include "Kalman.h"
class Track
{
public:
    Track(  size_t trackID,
            const Region& region,
            std::unique_ptr<GKalmanFilter> kalmanFilter
            );
    ~Track();
  //  track_t CalcDist(const Point_t& pt) const;
 //   track_t CalcDist(const cv::Rect& r) const;
 //   void CalcDistFPrevios(const Point_t& pt);
//    void Update(const Region& region, bool dataCorrect, cv::Mat currFrame);
 //   Point_t GetPredictedPoint() const;
//    void PointUpdate(const Point_t& pt, bool dataCorrect, const cv::Size& frameSize);

    size_t GetTrackID() const; 
    Region GetLastRegion() const;  
    Point_t GetPredictedPoint() const; 
    size_t GetNumOfSkippedFrames(); 
    void Update(const Region& newRegion, bool isCorrectData); 
    void UpdatePoint(const Point_t& newPoint, bool isCorrectData); 
    track_t CalDistFromAPoint(const Point_t& otherPoint) const;
    cv::Rect GetLastRect() const;
    void CalcDT(const Point_t& pt);
    size_t m_numOfSkippedFrames;
    bool isDisplay;

private:
    size_t m_trackID;
    Region m_lastRegion;
    Point_t m_predictedPoint; 
    std::unique_ptr<GKalmanFilter> m_kalman;
    // bool m_outOfTheFrame;
    track_t m_dt;

};
typedef std::vector<std::unique_ptr<Track>> tracks_t;
