#include "Track.h"

Track::Track(const Region& region,
            tracking::KalmanType kalmanType,
            float deltaTime,
            float accelNoiseMag,
            size_t trackID)
    :
      m_trackID(trackID),
      m_skippedFrames(0),
      m_lastRegion(region),
      m_predictionPoint((region.m_rect.tl() + region.m_rect.br()) / 2),
      isDisplay(true),
      m_outOfTheFrame(false)
{
    m_kalman = new TKalmanFilter(kalmanType, m_predictionPoint, deltaTime, accelNoiseMag);
}
Track::~Track()
{

}
track_t Track::CalcDist(const Point_t& pt) const
{
    Point_t diff = m_predictionPoint - pt;
    return sqrtf(diff.x * diff.x + diff.y * diff.y);
}
void Track::CalcDistFPrevios(const Point_t& pt)
{
    Point_t lastPT((m_lastRegion.m_rect.tl() + m_lastRegion.m_rect.br()) / 2);
    Point_t diff = lastPT - pt;
    m_disFromPrevios = sqrtf(diff.x * diff.x + diff.y * diff.y);
} 
track_t Track::CalcDist(const cv::Rect& r) const
{
    std::array<track_t, 4> diff;
    diff[0] = m_predictionPoint.x - m_lastRegion.m_rect.width / 2.f - r.x;
    diff[1] = m_predictionPoint.y - m_lastRegion.m_rect.height / 2.f - r.y;
    diff[2] = static_cast<track_t>(m_lastRegion.m_rect.width - r.width);
    diff[3] = static_cast<track_t>(m_lastRegion.m_rect.height - r.height);

    track_t dist = 0;
    for (size_t i = 0; i < diff.size(); ++i)
    {
        dist += diff[i] * diff[i];
    }
    return sqrtf(dist);
}
void Track::Update(const Region& region, bool dataCorrect, cv::Mat currFrame)
{
    cv::Point pt((region.m_rect.tl() + region.m_rect.br()) / 2);
 //   std::cout << "\nBefore: " << pt.y;
   // std::cout << " - After: " << m_predictionPoint.y;

    if (dataCorrect)
    {
        CalcDistFPrevios(pt);
        isDisplay = true;
        m_lastRegion = region;
    }
    else    
        isDisplay = false;
    PointUpdate(pt, dataCorrect, currFrame.size());
    
}
void Track::PointUpdate(const Point_t& pt, bool dataCorrect, const cv::Size& frameSize)
{
    m_kalman->GetPointPrediction(dataCorrect, m_disFromPrevios/25);
    m_predictionPoint = m_kalman->Update(pt, dataCorrect);
   auto Clamp = [](track_t& v, int hi) -> bool
    {
        if (v < 0)
        {
            v = 0;
            return true;
        }
        else if (hi && v > hi - 1)
        {
			v = static_cast<track_t>(hi - 1);
            return true;
        }
        return false;
    };
    m_outOfTheFrame = false;
    m_outOfTheFrame |= Clamp(m_predictionPoint.x, frameSize.width);
    m_outOfTheFrame |= Clamp(m_predictionPoint.y, frameSize.height); 
}

Point_t Track::GetPredictedPoint() const
{
    return m_predictionPoint;
}
cv::Rect Track::GetLastRect() const
{
    return cv::Rect(
                static_cast<int>(m_predictionPoint.x - m_lastRegion.m_rect.width / 2),
                static_cast<int>(m_predictionPoint.y - m_lastRegion.m_rect.height / 2),
                m_lastRegion.m_rect.width,
                m_lastRegion.m_rect.height);    
}
