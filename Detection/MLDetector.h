#pragma once

#include "Detector.h"
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
class MLDetector : public Detector
{
public:
    MLDetector();
    ~MLDetector();
    virtual bool Init(const config_t& config);
    virtual void Detect(const cv::Mat &image);    
    virtual bool IsObject(const cv::Rect& rect);

private:
    std::vector<dlib::object_detector<dlib::scan_fhog_pyramid<dlib::pyramid_down<1>>>> m_models;   
};