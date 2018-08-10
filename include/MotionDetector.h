#pragma once
#include "Detector.h"
#include "BackgroundSubtract.h"
class MotionDetector : public Detector
{
    public:
    MotionDetector(BackgroundSubtract::BGFG_ALGS BSalgs, cv::Mat& gray);
    ~MotionDetector();
    virtual bool Init(const config_t &configs);
    virtual void Detect(FrameInfo &frame);
    virtual void DetectInROI(FrameInfo &frame);
    virtual bool CheckObjSize(const cv::Rect &r);
    void DetectContour();

    private:
    std::unique_ptr<BackgroundSubtract> m_BSubtractor;
    cv::Mat m_foreGround;

};