#include "Track.h"

Track::Track(size_t trackID,
            const Region& region,
            std::unique_ptr<GKalmanFilter> kalmanFilter
            )
    :
      m_trackID(trackID),
      m_kalman(std::move(kalmanFilter)),
      m_lastRegion(region),
      m_numOfSkippedFrames(0),
      m_predictedPoint((region.m_rect.tl() + region.m_rect.br()) / 2)
{
}
Track::~Track()
{

}

size_t Track::GetTrackID() const
{
    return m_trackID;
} 
Region Track::GetLastRegion() const
{
    return m_lastRegion;
}  
Point_t Track::GetPredictedPoint() const
{
    return m_predictedPoint;
} 
size_t Track::GetNumOfSkippedFrames()
{
    return m_numOfSkippedFrames;
} 


track_t Track::CalDistFromAPoint(const Point_t& otherPoint) const
{
    Point_t diff = m_predictedPoint - otherPoint;
    return sqrtf(diff.x * diff.x + diff.y * diff.y);
}
void Track::CalcDT(const Point_t& pt)
{
    Point_t lastPT((m_lastRegion.m_rect.tl() + m_lastRegion.m_rect.br()) / 2);
    // m_dt[0] = sqrtf((lastPT.x - pt.x) * (lastPT.x - pt.x))/30;
    // m_dt[1] = sqrtf((lastPT.y - pt.y) * (lastPT.y - pt.y))/30;
    // std::cout << m_dt[0] << " - " << m_dt[1] << "\n"; 
    Point_t diff = lastPT - pt;
    m_dt = sqrtf(diff.x * diff.x + diff.y * diff.y)/(25 * (m_numOfSkippedFrames + 1));
    std::cout << m_dt << "\n"; 

} 
void Track::Update(const Region& newRegion, bool isCorrectData)
{
    Point_t newPoint((newRegion.m_rect.tl() + newRegion.m_rect.br()) / 2);
    if (isCorrectData)
    {
        //CalcDT(newPoint);
         isDisplay = true;
        m_lastRegion = newRegion;
    }
   else    
       isDisplay = false;
    UpdatePoint(newPoint, isCorrectData);
    
}
void Track::UpdatePoint(const Point_t& newPoint, bool isCorrectData)
{
    m_kalman->GetPredictedPoint(isCorrectData, m_dt);
    m_predictedPoint = m_kalman->Update(newPoint, isCorrectData);
//    auto Clamp = [](track_t& v, int hi) -> bool
//     {
//         if (v < 0)
//         {
//             v = 0;
//             return true;
//         }
//         else if (hi && v > hi - 1)
//         {
// 			v = static_cast<track_t>(hi - 1);
//             return true;
//         }
//         return false;
//     };
//     m_outOfTheFrame = false;
//     m_outOfTheFrame |= Clamp(m_predictionPoint.x, frameSize.width);
//     m_outOfTheFrame |= Clamp(m_predictionPoint.y, frameSize.height); 
}

cv::Rect Track::GetLastRect() const
{
    return cv::Rect(
                static_cast<int>(m_predictedPoint.x - m_lastRegion.m_rect.width / 2),
                static_cast<int>(m_predictedPoint.y - m_lastRegion.m_rect.height / 2),
                m_lastRegion.m_rect.width,
                m_lastRegion.m_rect.height);    
}
