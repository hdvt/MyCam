#pragma once
#include "Detector.h"
#include "BackgroundSubtraction.h"
class MotionDetector : public Detector
{
    public:
    MotionDetector(BackgroundSubtraction::BGFG_ALGS BSalgs);
    ~MotionDetector();
    virtual bool Init(const config_t &configs);
    virtual void Detect(const cv::Mat &image);
    void DetectContour();
    void SetObjSize(cv::Size size);
    virtual bool IsObject(const cv::Rect &rect);

    private:
    std::unique_ptr<BackgroundSubtraction> m_backgroundSubtractor;
    cv::Mat m_foreGround;
    cv::Size m_minObjSize;  

};