#include "MotionDetector.h"

MotionDetector::MotionDetector(BackgroundSubtract::BGFG_ALGS BSalgs, cv::Mat& gray)
{
    m_BSubtractor = std::make_unique<BackgroundSubtract>(BSalgs);
    m_foreGround = gray.clone();
}
MotionDetector::~MotionDetector()
{

}
bool MotionDetector::Init(const config_t &configs)
{
    return m_BSubtractor->Init(configs);
}

void MotionDetector::Detect(FrameInfo &frame)
{
   m_BSubtractor->Subtract(frame.gray, m_foreGround);
   DetectContour();
}

bool MotionDetector::CheckObjSize(const cv::Rect &r)
{
	if (r.width >= m_minObjSize.width && r.height >= m_minObjSize.height)
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
			cv::Rect r = cv::boundingRect(contours[i]);

			if ( CheckObjSize(r) == true)
			{
				Region region(r);
				cv::Point2f center(r.x + 0.5f * r.width, r.y + 0.5f * r.height);
				m_regions.push_back(region);
			}
		}
	}
}

