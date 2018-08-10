#include "MLDetector.h"
#include "MotionDetector.h"

class MLMotionDetector : public Detector
{
public:
    MLMotionDetector(std::unique_ptr<Detector> MLDetector, std::unique_ptr<Detector> motionDetector);
    ~MLMotionDetector();
    virtual bool Init(const config_t &config);
    virtual void Detect(const cv::Mat &image);
    virtual bool IsObject(const cv::Rect& rect);
    void SetROI(cv::Rect roi);
private:
    std::unique_ptr<Detector> m_motionDetector;
    std::unique_ptr<Detector> m_MLDetector;
    cv::Rect m_ROI;
};
