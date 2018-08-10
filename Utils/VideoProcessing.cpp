#include "VideoProcessing.h"
using namespace cv;
VideoProcessing::~VideoProcessing()
{
    m_writer.release();
}
bool VideoProcessing::InitCapture(std::string videoPath)
{
    return m_capture.open(videoPath);
}
cv::Mat VideoProcessing::GetFrame()
{
    m_capture.read(m_currentFrame);
    return m_currentFrame;
} 
bool VideoProcessing::GetState()
{
    return m_isRun;
}
void VideoProcessing::SetState(bool isRun)
{
    m_isRun = isRun;
}

void VideoProcessing::SetROI()
{
    m_ROI = cv::selectROI(m_currentFrame);
}
cv::Rect VideoProcessing::GetROI()
{
    return m_ROI;
}
void VideoProcessing::InitWrite(std::string outFile)
{
    m_writer.open(outFile, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, m_currentFrame.size(), true);
}
void VideoProcessing::Write(cv::Mat &frame)
{
    if (!m_writer.isOpened())
         m_writer.open("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, m_currentFrame.size(), true);
    if (m_writer.isOpened())
    {
        m_writer << frame;
    }  

}
