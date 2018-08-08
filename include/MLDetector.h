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
    virtual void Detect(FrameInfo &frameInfo);
    
private:
    typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<1> > image_scanner_type;
    std::vector<dlib::object_detector<image_scanner_type> > m_detectors;
    dlib::array2d<unsigned char> m_cimg;
    std::vector<dlib::rect_detection> m_dets;
};