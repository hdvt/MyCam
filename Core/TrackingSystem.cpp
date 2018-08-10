#include "TrackingSystem.h"

TrackingSystem::TrackingSystem()
{
}
TrackingSystem::~TrackingSystem()
{

}
bool TrackingSystem::Init(const cv::CommandLineParser& parser)
{
    SetSetting(parser);
    // check video file
    if (!m_videoProcessor.InitCapture(m_settings.m_inFile))
    {
        std::cout << "Video can't open!\n";
        return false;
    }
    // Init Buffer
    m_frameBuff = std::make_shared<Buff<FrameInfo>>(5);

    // Set ROI if the detector requires
    if (m_settings.detectorType == tracking::Detectors::ML_MOTION)
    {
        m_videoProcessor.GetFrame();
        m_videoProcessor.SetROI();
        m_settings.detectorSettings["xROI"] = std::to_string(m_videoProcessor.GetROI().x);
        m_settings.detectorSettings["yROI"] = std::to_string(m_videoProcessor.GetROI().y);
        m_settings.detectorSettings["wROI"] = std::to_string(m_videoProcessor.GetROI().width);                
        m_settings.detectorSettings["hROI"] = std::to_string(m_videoProcessor.GetROI().height);
    }
    // Init detector
    m_detector = std::move(DetectorFactory::GetDetector(m_settings.detectorType, m_settings.detectorSettings));
    if (m_detector == nullptr)
    {
        std::cout << "Detector can't init!\n";
        return false;
    }    
    // Init tracker 
    m_tracker = std::make_unique<Tracker>(m_settings);
    if (m_tracker == nullptr)
    {
        std::cout << "Tracker can't init!\n";
        return false;
    } 
    m_videoProcessor.SetState(true);
    return true;
}
void TrackingSystem::SetSetting(const cv::CommandLineParser& parser)
{
    m_settings.m_inFile = parser.get<std::string>(0);
    m_settings.m_outFile = parser.get<std::string>("output_url");
    m_settings.m_dt = 0.5;
    m_settings.distThres = 100;    
    switch (parser.get<int>("detection_type"))
    {
        case 0:
            m_settings.detectorType = tracking::Detectors::MOTION_MOG2;
            m_settings.detectorSettings["history"] = "500";
            m_settings.detectorSettings["varThreshold"] = "16";
            m_settings.detectorSettings["detectShadow"] = "1";
            break;
        case 1:
            m_settings.detectorType = tracking::Detectors::ML_HOGSVM;
            m_settings.detectorSettings["carModel"] = "../data/model/bike.svm";
            m_settings.detectorSettings["bikeModel"] = "../data/model/car.svm";
            break;
        case 2:
            m_settings.detectorType = tracking::Detectors::ML_MOTION;
            m_settings.detectorSettings["carModel"] = "../data/model/bike.svm";
            m_settings.detectorSettings["bikeModel"] = "../data/model/car.svm";
            m_settings.detectorSettings["history"] = "500";
            m_settings.detectorSettings["varThreshold"] = "16";
            m_settings.detectorSettings["detectShadow"] = "1";
            break;
    }

}
bool TrackingSystem::InitTracker(const FrameInfo& frameInfo)
{
    return true;
}

void TrackingSystem::Start()
{
    std::mutex frameLock;
    std::condition_variable frameCond;
    std::thread capDetectThr(std::bind(&TrackingSystem::Detecting, this, &frameLock, &frameCond));
    capDetectThr.detach();
    int k = 0;
    FrameInfo newFrame;
    while (m_videoProcessor.GetState() && k != 27)
    {
        std::cout << "Process: Tracking\n";
        newFrame = m_frameBuff->Take();
        int64 t1 = cv::getTickCount();
        Tracking(newFrame);
        int64 t2 = cv::getTickCount();
        m_settings.m_fps =  cv::getTickFrequency() / (t2 - t1 + newFrame.m_dt);
        DrawData(newFrame.frame); // draw tracks
        //DrawData(newFrame); // just draw detected regions
        cv::imshow("Video track", newFrame.frame);       
        k = cv::waitKey(1);
    }
    m_videoProcessor.SetState(false);
    m_frameBuff->thStop = true;
    m_frameBuff->Release();
    if (capDetectThr.joinable())
    {
        capDetectThr.join();
    }       
    return;

}
void TrackingSystem::Detecting(
        std::mutex* frameLock, 
        std::condition_variable* frameCond
        )
{
    FrameInfo newFrame;
    while (m_videoProcessor.GetState())
    {
        std::cout << "Thread: Capture\n";
        newFrame.frame = m_videoProcessor.GetFrame().clone();
        if (newFrame.frame.empty())
        {
            std::unique_lock<std::mutex> lock(*frameLock);
            std::cerr << "Process: Empty frame \n";
            m_videoProcessor.SetState(false);
            break;
        }
        cv::cvtColor(newFrame.frame, newFrame.gray, cv::COLOR_BGR2GRAY); 
        int64 t1 = cv::getTickCount();
        m_detector->Detect(newFrame.frame);
        const regions_t regs = m_detector->GetRegions();
        newFrame.m_regions.assign(std::begin(regs), std::end(regs));
        int64 t2 = cv::getTickCount();
        newFrame.m_dt = t2 - t1; // cal detection time
        if (!m_videoProcessor.GetState())
           break;
        m_frameBuff->Put(std::move(newFrame));
    }
    m_videoProcessor.SetState(false);
    return;
}        

void TrackingSystem::Tracking(FrameInfo& frameInfo)
{
    // if (m_settings.detectorType == tracking::Detectors::ML_MOTION)
        m_tracker->Update(frameInfo);    
    // else
    //     m_tracker->Update(frameInfo.m_regions, frameInfo.gray);    
}
void TrackingSystem::DrawData(FrameInfo& frameInfo) // draw detected regoins
{   
    for (int i = 0; i < frameInfo.m_regions.size(); i++)
    {
        cv::rectangle(frameInfo.frame, frameInfo.m_regions[i].m_rect, color::SCALAR_BLUE);
    }
    // for (int i = 0; i < frameInfo.m_motionRegions.size(); i++)
    // {
    //     cv::rectangle(frameInfo.frame, frameInfo.m_motionRegions[i].m_rect, color::SCALAR_BLUE);
    // }
}
void TrackingSystem::DrawData(cv::Mat frame) // draw tracks
{   
    std::stringstream fps;
    fps << "FPS: " << m_settings.m_fps;
    cv::putText(frame, fps.str().c_str(), cv::Point(frame.cols/2, 30),cv::FONT_HERSHEY_SIMPLEX, 1, color::SCALAR_RED,2);
    for (const auto& track : m_tracker->m_tracks)
    {
       if (track->isDisplay == true)
        DrawTrack(frame, *track);
    }
 //   m_videoProcessor.Write(frame);
}
void TrackingSystem::DrawTrack(cv::Mat frame, const Track& track)
{
    cv::rectangle(frame, track.GetLastRect(), color::SCALAR_GREEN, 2, CV_AA);
    std::stringstream number;
    number << track.GetTrackID();
    cv::putText(frame, number.str().c_str(), track.GetPredictedPoint(),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0),2);    
}