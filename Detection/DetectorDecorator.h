#include "Detector.h"

class DetectorDecorator : public Detector
{
// public:
//     DetectorDecorator(std::shared_ptr<Detector> detector)
//     {
//         m_mainDetector = std::move(detector);
//     }
//     virtual ~DetectorDecorator()
//     {

//     }
//     virtual bool Init(const config_t &config)
//     {
//         m_mainDetector->Init(config);
//     }
//     virtual void Detect(const FrameInfo &frameData)
//     {
//         m_mainDetector->Detect(frameData);
//     }
//     virtual bool IsObject(const cv::Rect& rect)
//     {
//         m_mainDetector->IsObject(rect);
//     }

// protected:
//     std::shared_ptr<Detector> m_mainDetector;
};