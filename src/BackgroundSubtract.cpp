#include "BackgroundSubtract.h"

BackgroundSubtract::BackgroundSubtract(BGFG_ALGS algType) : m_algType(algType)
{
//    Init(config);
}

BackgroundSubtract::~BackgroundSubtract()
{

}
bool BackgroundSubtract::Init(const config_t& config)
{
    switch(m_algType)
    {
        case BackgroundSubtract::BGFG_ALGS::ALG_MOG2:
        {    
            std::vector<std::string> paramsConf = { "history", "varThreshold", "detectShadows" };
            auto params = std::make_tuple(500, 16, 1);
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
                    }
                }
            }
            m_model = cv::createBackgroundSubtractorMOG2(std::get<0>(params), std::get<1>(params), std::get<2>(params) != 0).dynamicCast<cv::BackgroundSubtractor>();
            return true;
            break;
        }
    }
    return false;
}

void BackgroundSubtract::Subtract(const cv::Mat& gray, cv::Mat& foreground)
{
     switch (m_algType)
    {   
        case BGFG_ALGS::ALG_MOG2:
        {
            m_model->apply(gray, foreground);
            cv::threshold(foreground, foreground, 200, 255, cv::THRESH_BINARY);
            break;
        }
    }
   //  cv::imshow("Original Foreground", foreground);
    cv::medianBlur(foreground, foreground, 9);
   // for (int i = 0; i < 1; i++)
   //     cv::erode(foreground, foreground,cv::Mat());
    for (int i = 0; i < 5; i++)
         cv::dilate(foreground, foreground,cv::Mat());  
    // cv::medianBlur(foreground, foreground, 3);

    // cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    // cv::dilate(foreground, foreground, dilateElement, cv::Point(-1, -1), 2);

   //cv::imshow("Filtered Foreground", foreground);
}