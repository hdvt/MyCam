#include "Detector.h"
#include "MotionDetector.h"
#include "MLDetector.h"
Detector::Detector()
{

}
Detector::~Detector()
{

}
void Detector::SetObjSize(cv::Size objSize)
{
    m_minObjSize = objSize;
}
bool Detector::CheckObjSize(const cv::Rect& rect)
{
	if (rect.width >= m_minObjSize.width && rect.height >= m_minObjSize.height)
		return true;

	return false;
}

regions_t Detector::GetRegions()
{
    return m_regions;
}
Detector* Detector::GetDetector(tracking::Detectors detectorType, config_t &settings, FrameInfo& frameInfo)
{
    Detector* detector = nullptr;
    switch(detectorType)
    {
        case tracking::Detectors::MOTION_MOG2:
            detector = new MotionDetector(BackgroundSubtract::BGFG_ALGS::ALG_MOG2, frameInfo.gray);
            //std::cout << "switch";
            break;       
        case tracking::Detectors::ML_HOGSVM:
        case tracking::Detectors::ML_MOTION:
            detector = new MLDetector();
            //std::cout << "switch";
            break;                      
    }
    if (!detector->Init(settings))
    {
        delete detector;
        detector = nullptr;
    }   
    return detector;
}   