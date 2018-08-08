#include "Kalman.h"
#include <iostream>
#include <vector>

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
TKalmanFilter::TKalmanFilter(
        tracking::KalmanType type,
        Point_t pt,
        track_t deltaTime, // time increment (lower values makes target more "massive")
        track_t accelNoiseMag
        )
    :
      m_type(type),
      m_initialized(false),
      m_deltaTime(deltaTime),
      m_accelNoiseMag(accelNoiseMag)
{
    m_initialPoints.push_back(pt);
    m_lastPointResult = pt;
}


//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter()
{
}

//---------------------------------------------------------------------------
void TKalmanFilter::CreateLinear(Point_t xy0, Point_t xyv0)
{
    // We don't know acceleration, so, assume it to process noise.
    // But we can guess, the range of acceleration values thich can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: m/s^2)
    // shows, woh much target can accelerate.

    //4 state variables, 2 measurements
    m_linearKalman = std::make_unique<cv::KalmanFilter>(4, 2, 0);
    // Transition cv::Matrix
    m_linearKalman->transitionMatrix = (cv::Mat_<track_t>(4, 4) <<
                                        1, 0, m_deltaTime, 0,
                                        0, 1, 0, m_deltaTime,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1);
  

    // init...
    m_lastPointResult = xy0;
    m_linearKalman->statePre.at<track_t>(0) = xy0.x; // x
    m_linearKalman->statePre.at<track_t>(1) = xy0.y; // y

    m_linearKalman->statePre.at<track_t>(2) = xyv0.x;
    m_linearKalman->statePre.at<track_t>(3) = xyv0.y;

    m_linearKalman->statePost.at<track_t>(0) = xy0.x;
    m_linearKalman->statePost.at<track_t>(1) = xy0.y;

    cv::setIdentity(m_linearKalman->measurementMatrix);

    m_linearKalman->processNoiseCov = (cv::Mat_<track_t>(4, 4) <<
                                       pow(m_deltaTime,4.0)/4.0	,0						,pow(m_deltaTime,3.0)/2.0		,0,
                                       0						,pow(m_deltaTime,4.0)/4.0	,0							,pow(m_deltaTime,3.0)/2.0,
                                       pow(m_deltaTime,3.0)/2.0	,0						,pow(m_deltaTime,2.0)			,0,
                                       0						,pow(m_deltaTime,3.0)/2.0	,0							,pow(m_deltaTime,2.0));


    m_linearKalman->processNoiseCov *= m_accelNoiseMag;

    setIdentity(m_linearKalman->measurementNoiseCov, cv::Scalar::all(0.1));

    setIdentity(m_linearKalman->errorCovPost, cv::Scalar::all(.1));

    m_initialized = true;
}


//---------------------------------------------------------------------------
Point_t TKalmanFilter::GetPointPrediction(bool dataCorrect, track_t dt)
{
    if (m_initialized)
    {
        cv::Mat prediction;

        switch (m_type)
        {
        case tracking::KalmanLinear:
            if (dataCorrect == true && 1 == 2)
            {               
                m_linearKalman->transitionMatrix.at<track_t>(2) = dt;
                m_linearKalman->transitionMatrix.at<track_t>(7) = dt;
            }
            prediction = m_linearKalman->predict();
            break;

        }

        m_lastPointResult = Point_t(prediction.at<track_t>(0), prediction.at<track_t>(1));
    }
    else
    {

    }
    return m_lastPointResult;
}
// ---
void TKalmanFilter::UpdateDelta(track_t dt)
{

}
//---------------------------------------------------------------------------
Point_t TKalmanFilter::Update(Point_t pt, bool dataCorrect)
{
    if (!m_initialized)
    {
        if (m_initialPoints.size() < MIN_INIT_VALS)
        {
            if (dataCorrect)
            {
                m_initialPoints.push_back(pt);
            }
        }
        if (m_initialPoints.size() == MIN_INIT_VALS)
        {
            track_t kx = 0;
            track_t bx = 0;
            track_t ky = 0;
            track_t by = 0;
            get_lin_regress_params(m_initialPoints, 0, MIN_INIT_VALS, kx, bx, ky, by);
            Point_t xy0(kx * (MIN_INIT_VALS - 1) + bx, ky * (MIN_INIT_VALS - 1) + by);
            Point_t xyv0(kx, ky);
          //  std::cout << "\nFirst: " <<  xy0.x << " - " << xy0.y << "\n";
            switch (m_type)
            {
            case tracking::KalmanLinear:
                CreateLinear(xy0, xyv0);
                break;

            }
        }
    }

    if (m_initialized)
    {
        cv::Mat measurement(2, 1, CV_32FC1);
        if (!dataCorrect)
        {
            measurement.at<track_t>(0) = m_lastPointResult.x;  //update using prediction
            measurement.at<track_t>(1) = m_lastPointResult.y;
        }
        else
        {
            measurement.at<track_t>(0) = pt.x;  //update using measurements
            measurement.at<track_t>(1) = pt.y;
        }
        // if (dataCorrect)
        //     std::cout << "\n Real Measure: " << measurement.at<track_t>(0) << " - " << measurement.at<track_t>(1) << "\n";
        // else
        //     std::cout << "\n Predicted Measure: " << measurement.at<track_t>(0) << " - " << measurement.at<track_t>(1) << "\n";

        // Correction
        cv::Mat estimated;
        switch (m_type)
        {
        case tracking::KalmanLinear:
            estimated = m_linearKalman->correct(measurement);
            break;

        }

        m_lastPointResult.x = estimated.at<track_t>(0);   //update using measurements
        m_lastPointResult.y = estimated.at<track_t>(1);
    //    std::cout << "\nEstimated: " <<  m_lastPointResult.x << " - " << m_lastPointResult.y << "\n";
    }
    else
    {
        if (dataCorrect)
        {
            m_lastPointResult = pt;
        }
    }
    return m_lastPointResult;
}
