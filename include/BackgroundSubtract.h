#pragma once
#include <opencv2/opencv.hpp>
#include "define.h"
#ifdef USE_OCV_BGFG
#include <opencv2/bgsegm.hpp>
#endif


class BackgroundSubtract
{
public:
	enum class BGFG_ALGS
	{
        ALG_MOG2
	};

    BackgroundSubtract(BGFG_ALGS algType);
	~BackgroundSubtract();

    bool Init(const config_t& config);

    void Subtract(const cv::Mat& image, cv::Mat& foreground);
	
	BGFG_ALGS m_algType;

private:
	cv::Ptr<cv::BackgroundSubtractor> m_model;
};
