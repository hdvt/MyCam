#pragma once
#include "../Core/define.h"
class Detector
{
public:
    Detector();
    virtual ~Detector();
    regions_t GetRegions();
    virtual bool Init(const config_t &config) = 0;
    virtual void Detect(const cv::Mat &image) = 0;
    virtual bool IsObject(const cv::Rect& rect) = 0;
    virtual void SetObjSize(cv::Size size);

protected:
    regions_t m_regions;
};
