#include "MLDetector.h"
#include "MotionDetector.h"
#include "MLMotionDetector.h"

class DetectorFactory 
{
public:
    static std::unique_ptr<Detector> GetDetector(tracking::Detectors type, const config_t& config)
    {
        std::unique_ptr<Detector> newDetector(nullptr);
        switch (type)
        {
            case tracking::Detectors::MOTION_MOG2:
            {
                newDetector = std::unique_ptr<Detector>(new MotionDetector(BackgroundSubtraction::BGFG_ALGS::ALG_MOG2));
                break;
            }

            case tracking::Detectors::ML_HOGSVM:
            {
                newDetector = std::unique_ptr<Detector>(new MLDetector());
                break;
            }
            case tracking::Detectors::ML_MOTION:
            {
                newDetector = std::unique_ptr<Detector>(new MLMotionDetector(std::move(std::unique_ptr<Detector>(new MLDetector())),
                                                                             std::move(std::unique_ptr<Detector>(new MotionDetector(BackgroundSubtraction::BGFG_ALGS::ALG_MOG2))))
                                                       );
                break;
            }
        }
        if (!newDetector->Init(config))
            newDetector = std::unique_ptr<Detector>(nullptr);
        return newDetector;       
    }
};