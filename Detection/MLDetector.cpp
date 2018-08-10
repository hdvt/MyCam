#include "MLDetector.h"
#include <tuple>
MLDetector::MLDetector() 
{
}
MLDetector::~MLDetector()
{
}
bool MLDetector::Init(const config_t& config)
{
    std::vector<std::string> paramsConf = { "carModel", "bikeModel" };
    auto params = std::make_tuple(std::string("../data/model/bike.svm"), std::string("./data/model/car.svm"));
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
            }
        }
    }
    dlib::object_detector<dlib::scan_fhog_pyramid<dlib::pyramid_down<1>>> detect;  
    dlib::deserialize(std::get<0>(params)) >> detect;
    m_models.push_back(detect);
    dlib::deserialize(std::get<1>(params)) >> detect;
    m_models.push_back(detect);
    return true;
}
bool MLDetector::IsObject(const cv::Rect& rect)
{
    return true;
}
void MLDetector::Detect(const cv::Mat &image)
{
    m_regions.clear();
    dlib::array2d<unsigned char> cimg;
    std::vector<dlib::rect_detection> dets;
    dlib::assign_image(cimg, dlib::cv_image<dlib::bgr_pixel>(image));
    evaluate_detectors(m_models, cimg, dets);
    
    for (unsigned long i = 0; i < dets.size(); ++i) 
    {
        cv::Rect roi(dets[i].rect.left(), dets[i].rect.top(), dets[i].rect.right() - dets[i].rect.left(), dets[i].rect.bottom() - dets[i].rect.top());

        if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= image.cols
                && 0 <= roi.y && 0 <= roi.height
                && roi.y + roi.height <= image.rows) 
        {

            Region region(roi);
            cv::Point2f center(roi.x + 0.5f * roi.width, roi.y + 0.5f * roi.height);          
            if (1 == 0)
            {
                 cv::Mat roiIM = image(roi);
                 cv::goodFeaturesToTrack(roiIM, region.m_regions, 500, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);    
                 for (int j = 0; j < region.m_regions.size(); j++)
                 {
                     region.m_regions[j].x += roi.x; 
                     region.m_regions[j].y += roi.y;  
                 }
                    
            }
            m_regions.push_back(region);
       }
        
    }
}
