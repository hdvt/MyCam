#include "TrackingSystem.h"

TrackingSystem::TrackingSystem(const cv::CommandLineParser& parser)
{
    m_settings.m_inFile = parser.get<std::string>(0);
    m_settings.detectorType = (parser.get<int>("detection_type") != 0) ? tracking::Detectors::MOTION_MOG2 : tracking::Detectors::ML_HOGSVM;
    m_frameBuff = std::make_shared<Buff<FrameInfo>>(2);
    m_isInitTracker = false;
    InitWindows();
}
TrackingSystem::~TrackingSystem()
{

}
bool TrackingSystem::InitTracker(FrameInfo& frameInfo)
{
    switch (m_settings.detectorType)
    {
        case tracking::Detectors::MOTION_MOG2:
        {
            m_settings.detectorSettings["history"] = "500";
            m_settings.detectorSettings["varThreshold"] = "16";
            m_settings.detectorSettings["detectShadow"] = "1";
            break;
        }
        case tracking::Detectors::ML_HOGSVM:
        {
            m_settings.detectorSettings["carModel"] = "../data/model/bike.svm";
            m_settings.detectorSettings["bikeModel"] = "../data/model/car.svm";
            break;
        }
    }

//    m_settings.distThres = frameInfo.frame.rows / 20;     
    m_detector = std::unique_ptr<Detector>(Detector::GetDetector(m_settings.detectorType, m_settings.detectorSettings, frameInfo));
    m_detector->SetObjSize(cv::Size(30, 30));
    m_settings.m_dt = 0.5;
    m_settings.distThres = 50;
    m_tracker = std::make_unique<Tracker>(m_settings);
    return true;
}
void TrackingSystem::InitWindows()
{
  //  int sreenWidth = 1928;
   // int sreenHeight = 1080;
   // cv::namedWindow("Video", cv::WINDOW_FREERATIO);        
}
void TrackingSystem::Process()
{
    std::mutex frameLock;
    std::condition_variable frameCond;
    bool stopCapture = false;
    std::thread capDetectThr(std::bind(&TrackingSystem::CaptureAndDetecting, this, &stopCapture, &frameLock, &frameCond));
    capDetectThr.detach();
    const int captureTimeout = 2000;
    {
        std::unique_lock<std::mutex> lock(frameLock);
        auto now = std::chrono::system_clock::now();
        if (frameCond.wait_until(lock, now + std::chrono::milliseconds(captureTimeout)) == std::cv_status::timeout)
        {
            std::cerr << "Process: Init capture timeout!" << std::endl;
            stopCapture = true;
            if (capDetectThr.joinable())
            {
                capDetectThr.join();
            }
            return;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int k = 0;
    FrameInfo newFrame;
    while (!stopCapture && k != 27)
    {
        std::cout << "Process: Tracking\n";
        newFrame = m_frameBuff->Take();
        Tracking(newFrame);
        // DrawData(newFrame);//newFrame.frame);
        // cv::Mat fram2 = newFrame.frame.clone();
        // DrawData(fram2);//newFrame.frame);
        DrawData(newFrame.frame);//newFrame.frame);
        cv::imshow("Video track", newFrame.frame);     
//        cv::imshow("Video detect", newFrame.frame); 
        k = cv::waitKey(100);
    }
    stopCapture = true;
    m_frameBuff->thStop = true;
    m_frameBuff->Release();
    if (capDetectThr.joinable())
    {
        capDetectThr.join();
    }       
    return;

}
void TrackingSystem::CaptureAndDetecting(
        bool* stopCapture, 
        std::mutex* frameLock, 
        std::condition_variable* frameCond
        )
{
    cv::VideoCapture capture(m_settings.m_inFile);
    if (!capture.isOpened())
    {
        *stopCapture = true;
        std::cout << "Can't open " << m_settings.m_inFile << " file\n";
        return;
    }
    frameCond->notify_all();
   // const int trackingTimeout = 2000;
    FrameInfo newFrame;
    while (!*stopCapture)
    {
        std::cout << "Process: Capture\n";
        capture.read(newFrame.frame);
        if (newFrame.frame.empty())
        {
            std::unique_lock<std::mutex> lock(*frameLock);
            std::cerr << "Process: Empty frame \n";
            *stopCapture = true;
            break;
        }
        cv::cvtColor(newFrame.frame, newFrame.gray, cv::COLOR_BGR2GRAY); 
        if (!m_isInitTracker)
        {
            if (!InitTracker(newFrame))
            {
                std::unique_lock<std::mutex> lock(*frameLock);
                std::cerr << "Process: Can't init tracker!\n";
                *stopCapture = true;
                break;
            }
            else
                m_isInitTracker = true;
         }
      //  clock_t t = clock();
         Detect(newFrame);
    //    std::cout << "Detect time: " <<  (clock() - t) / CLOCKS_PER_SEC << "\n";
        if (*stopCapture == true)
            break;
        m_frameBuff->Put(std::move(newFrame));
    }
   // *stopCapture = true;
    return;
}        

void TrackingSystem::Detect(FrameInfo& frameInfo)
{   
    m_detector->Detect(frameInfo);
    const regions_t regs = m_detector->GetRegions();
    frameInfo.m_regions.assign(std::begin(regs), std::end(regs));
}
void TrackingSystem::Tracking(FrameInfo& frameInfo)
{
    m_tracker->Update(frameInfo.m_regions, frameInfo.gray);    
}
void TrackingSystem::DrawData(FrameInfo& frameInfo) // draw without tracking
{   
    for (int i = 0; i < frameInfo.m_regions.size(); i++)
    {
        cv::rectangle(frameInfo.frame, frameInfo.m_regions[i].m_rect, color::SCALAR_BLUE);
       // std::stringstream number;
       //  number << frameInfo.m_regions[i].m_rect.x ;
      //  cv::putText(frame, number.str().c_str(), track.GetPredictedPoint(),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0),2);
    }
}
void TrackingSystem::DrawData(cv::Mat frame)
{   
 
    for (const auto& track : m_tracker->m_tracks)
    {
        if (track->isDisplay == true)
        DrawTrack(frame, *track);
    }
}
void TrackingSystem::DrawTrack(cv::Mat frame, const Track& track)
{
    int resizeCoeff = 1;
    auto ResizeRect = [&](const cv::Rect& r) -> cv::Rect
    {
        return cv::Rect(resizeCoeff * r.x, resizeCoeff * r.y, resizeCoeff * r.width, resizeCoeff * r.height);
    };

    cv::rectangle(frame, ResizeRect(track.GetLastRect()), color::SCALAR_RED, 2, CV_AA);
    std::stringstream number;
    number << track.m_trackID;
    cv::putText(frame, number.str().c_str(), track.GetPredictedPoint(),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0),2);
    for (auto pt : track.m_lastRegion.m_regions)
    {
         cv::circle(frame, cv::Point(cvRound(pt.x), cvRound(pt.y)), 1, color::SCALAR_RED, -1, CV_AA);
    }
}