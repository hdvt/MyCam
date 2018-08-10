#include "MLMotionDetector.h"

MLMotionDetector::MLMotionDetector(std::unique_ptr<Detector> MLDetector, std::unique_ptr<Detector> motionDetector) 
    : m_MLDetector(std::move(MLDetector)), m_motionDetector(std::move(motionDetector))
{
}
MLMotionDetector::~MLMotionDetector()
{

}
bool MLMotionDetector::Init(const config_t &config)
{
    std::vector<std::string> paramsConf = { "xROI", "yROI", "wROI", "yROI" };
    auto params = std::make_tuple(1,2,3,4);
    for (size_t i = 0; i < paramsConf.size(); ++i)
    {
        auto conf = config.find(paramsConf[i]);
        if (conf != config.end())
        {
            std::stringstream ss(conf->second);

            switch (i)
            {
            case 0:
                ss >> std::get<0>(params);
                break;
            case 1:
                ss >> std::get<1>(params);
                break;
            case 2:
                ss >> std::get<2>(params);
                break;
            case 3:
                ss >> std::get<3>(params);
                break;
            
            }
        }
    }
   m_ROI.x = std::get<0>(params);
   m_ROI.y = std::get<1>(params);
   m_ROI.width = std::get<2>(params);
   m_ROI.height = std::get<3>(params);
   m_motionDetector->SetObjSize(cv::Size(30, 30));
   return (m_MLDetector->Init(config) && m_motionDetector->Init(config));
}
void MLMotionDetector::Detect(const cv::Mat &image)
{
    m_regions.clear();
    m_MLDetector->Detect(image);
    cv::Mat roiIM = image(m_ROI);
    m_motionDetector->Detect(roiIM);
    const regions_t mlRegs = m_MLDetector->GetRegions();
    regions_t motionRegs = m_motionDetector->GetRegions();  
    for (int i = 0; i < motionRegs.size(); i++)
    {
        motionRegs[i].m_rect.x += m_ROI.x;
        motionRegs[i].m_rect.y += m_ROI.y;
    }  
    m_regions = mlRegs;
    m_regions.insert(m_regions.end(), motionRegs.begin(), motionRegs.end());

}
void MLMotionDetector::SetROI(cv::Rect roi)
{
    m_ROI = roi;
}
bool MLMotionDetector::IsObject(const cv::Rect& rect)
{

}