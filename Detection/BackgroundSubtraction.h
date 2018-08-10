#pragma once
#include <opencv2/opencv.hpp>
#include "../Core/define.h"
#ifdef USE_OCV_BGFG
#include <opencv2/bgsegm.hpp>
#endif

class BackgroundSubtraction
{
public:
    enum class BGFG_ALGS
    {
        ALG_MOG2
    };
    BackgroundSubtraction(BGFG_ALGS algType);
	~BackgroundSubtraction();
    bool Init(const config_t& config);
    void Subtract(const cv::Mat& image, cv::Mat& foreground);	

private:
	cv::Ptr<cv::BackgroundSubtractor> m_subtractor;
	BGFG_ALGS m_algType;
};
