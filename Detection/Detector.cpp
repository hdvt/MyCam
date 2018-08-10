#include "Detector.h"
#include "MotionDetector.h"
#include "MLDetector.h"
Detector::Detector()
{

}
Detector::~Detector()
{

}

regions_t Detector::GetRegions()
{
    return m_regions;
}
 void Detector::SetObjSize(cv::Size size)
 {
     
 }