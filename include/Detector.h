#pragma once
#include "define.h"
class Detector
{
    public:
    Detector();
    virtual ~Detector();
    virtual bool Init(const config_t &config) = 0;
    virtual void Detect(FrameInfo &frame) = 0;
    virtual void DetectInROI(FrameInfo &frame){}
    regions_t GetRegions();
    void SetObjSize(cv::Size objSize);
    virtual bool CheckObjSize(const cv::Rect& rect);
    static Detector* GetDetector(tracking::Detectors detectorType, config_t &settings, FrameInfo& frameInfo);   

    protected:
    regions_t m_regions;
    cv::Size m_minObjSize;  
};
