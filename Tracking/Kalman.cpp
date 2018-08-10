#include "Kalman.h"
#include <iostream>
#include <vector>

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
GKalmanFilter::GKalmanFilter(
        tracking::KalmanType type,
        Point_t pt,
        track_t deltaTime, 
        track_t accelNoiseMag
        )
    :
      m_type(type),
      m_initialized(false),
      m_deltaTime(deltaTime),
      m_accelNoiseMag(accelNoiseMag)
{
    m_initialPoints.push_back(pt);
    m_lastPoint = pt;
}


//---------------------------------------------------------------------------
GKalmanFilter::~GKalmanFilter()
{
}

//---------------------------------------------------------------------------
void GKalmanFilter::CreateLinear(Point_t xy0, Point_t xyv0)
{
    m_linearKalman = std::make_unique<cv::KalmanFilter>(4, 2, 0);
    m_linearKalman->transitionMatrix = (cv::Mat_<track_t>(4, 4) <<
                                        1, 0, m_deltaTime, 0,
                                        0, 1, 0, m_deltaTime,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1);
  

    // init...
    m_lastPoint = xy0;
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
Point_t GKalmanFilter::GetPredictedPoint(bool dataCorrect, track_t dt)
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

        m_lastPoint = Point_t(prediction.at<track_t>(0), prediction.at<track_t>(1));
    }
    else
    {

    }
    return m_lastPoint;
}
//---------------------------------------------------------------------------
Point_t GKalmanFilter::Update(Point_t pt, bool dataCorrect)
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
            measurement.at<track_t>(0) = m_lastPoint.x;  //update using prediction
            measurement.at<track_t>(1) = m_lastPoint.y;
        }
        else
        {
            measurement.at<track_t>(0) = pt.x;  //update using measurements
            measurement.at<track_t>(1) = pt.y;
        }
        cv::Mat estimated;
        switch (m_type)
        {
        case tracking::KalmanLinear:
            estimated = m_linearKalman->correct(measurement);
            break;

        }

        m_lastPoint.x = estimated.at<track_t>(0);   //update using measurements
        m_lastPoint.y = estimated.at<track_t>(1);
    }
    else
    {
        if (dataCorrect)
        {
            m_lastPoint = pt;
        }
    }
    return m_lastPoint;
}
