#include "define.h"
class VideoProcessing
{
public:
    ~VideoProcessing();
    bool InitCapture(std::string videoPath);
    void InitWrite(std::string outFile);
    cv::Mat GetFrame();
    bool GetState();
    void SetState(bool isRun);
    void SetROI();
    cv::Rect GetROI();
    void Write(cv::Mat &frame);
private:
    cv::VideoCapture m_capture;
    cv::Mat m_currentFrame;
    bool m_isRun;
    bool m_isWrire;
    cv::VideoWriter m_writer;
    cv::Rect m_ROI;
};


         