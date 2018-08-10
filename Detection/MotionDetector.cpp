#include "MotionDetector.h"

MotionDetector::MotionDetector(BackgroundSubtraction::BGFG_ALGS BSalgs)
{
    m_backgroundSubtractor = std::make_unique<BackgroundSubtraction>(BSalgs);
}
MotionDetector::~MotionDetector()
{

}

void MotionDetector::SetObjSize(cv::Size size)
{
	m_minObjSize = size;
}

bool MotionDetector::Init(const config_t &configs)
{
    return m_backgroundSubtractor->Init(configs);
}

void MotionDetector::Detect(const cv::Mat &image)
{
   m_backgroundSubtractor->Subtract(image, m_foreGround);
   DetectContour();
}

bool MotionDetector::IsObject(const cv::Rect &rect)
{
	if (rect.width >= m_minObjSize.width && rect.height >= m_minObjSize.height)
		return true;
	return false;
}

void MotionDetector::DetectContour()
{
//	std::cout << "\n" << m_minObjectSize.width << "\n";
	m_regions.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
#if (CV_VERSION_MAJOR < 4)
	cv::findContours(m_foreGround, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point());
#else
    cv::findContours(m_foreGround, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());
#endif
	if (contours.size() > 0)
	{
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::Rect rect = cv::boundingRect(contours[i]);

			if ( IsObject(rect) == true)
			{
				Region region(rect);
				cv::Point2f center(rect.x + 0.5f * rect.width, rect.y + 0.5f * rect.height);
				m_regions.push_back(region);
			}
		}
	}
}

