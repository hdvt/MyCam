#pragma once
#include "define.h"
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>

///
/// \brief The TKalmanFilter class
/// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
///
class TKalmanFilter
{
public:
    TKalmanFilter(tracking::KalmanType type, Point_t pt, track_t deltaTime = 0.2, track_t accelNoiseMag = 0.5);
	~TKalmanFilter();

    Point_t GetPointPrediction(bool dataCorrect, track_t dt = 0);
    Point_t Update(Point_t pt, bool dataCorrect);
    void UpdateDelta(track_t dt);
private:
    tracking::KalmanType m_type;
    std::unique_ptr<cv::KalmanFilter> m_linearKalman;
    std::deque<Point_t> m_initialPoints;
    static const size_t MIN_INIT_VALS = 4;

    Point_t m_lastPointResult;
    bool m_initialized;
    track_t m_deltaTime;
    track_t m_accelNoiseMag;

    void CreateLinear(Point_t xy0, Point_t xyv0);
};

//---------------------------------------------------------------------------
///
/// \brief sqr
/// \param val
/// \return
///
template<class T> inline
T sqr(T val)
{
    return val * val;
}

///
/// \brief get_lin_regress_params
/// \param in_data
/// \param start_pos
/// \param in_data_size
/// \param kx
/// \param bx
/// \param ky
/// \param by
///
template<typename T, typename CONT>
void get_lin_regress_params(
        const CONT& in_data,
        int start_pos,
        int in_data_size,
        T& kx, T& bx, T& ky, T& by)
{
    T m1(0.), m2(0.);
    T m3_x(0.), m4_x(0.);
    T m3_y(0.), m4_y(0.);

    const T el_count = static_cast<T>(in_data_size - start_pos);
    for (int i = start_pos; i < in_data_size; ++i)
    {
        m1 += i;
        m2 += sqr(i);

        m3_x += in_data[i].x;
        m4_x += i * in_data[i].x;

        m3_y += in_data[i].y;
        m4_y += i * in_data[i].y;
    }
    T det_1 = 1. / (el_count * m2 - sqr(m1));

    m1 *= -1.;

    kx = det_1 * (m1 * m3_x + el_count * m4_x);
    bx = det_1 * (m2 * m3_x + m1 * m4_x);

    ky = det_1 * (m1 * m3_y + el_count * m4_y);
    by = det_1 * (m2 * m3_y + m1 * m4_y);
}
