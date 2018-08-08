#include "Tracker.h"

Tracker::Tracker(const TrackerSetting& settings)
    :
       m_settings(settings),
       m_nextTrackID(0)
{

}
Tracker::~Tracker()
{

}
void Tracker::Update(
        const regions_t& regions, 
        cv::Mat grayFrame
        )
{
    size_t N = m_tracks.size();		
    size_t M = regions.size();
    assignments_t assignment(N, -1); 

    if (!m_tracks.empty())
    {
        distMatrix_t Cost(N * M);
        const track_t maxPossibleCost = grayFrame.cols * grayFrame.rows;
        track_t maxCost = 0;
        for (size_t i = 0; i < m_tracks.size(); i++)
        {
            for (size_t j = 0; j < regions.size(); j++)
            {
                auto dist =  m_tracks[i]->CalcDist((regions[j].m_rect.tl() + regions[j].m_rect.br()) / 2);
                Cost[i + j * N] = dist;
                if (dist > maxCost)
                {
                    maxCost = dist;
                }
            }
        }
        AssignmentProblemSolver APS;
        APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] != -1)
            {
                if (Cost[i + assignment[i] * N] > m_settings.distThres)
                {
                    assignment[i] = -1;
                    m_tracks[i]->m_skippedFrames++;
                }
            }
            else
            {
                m_tracks[i]->m_skippedFrames++;
            }
        }

        for (int i = 0; i < static_cast<int>(m_tracks.size()); i++)
        {
            if (m_tracks[i]->m_skippedFrames > m_settings.m_maximumAllowedSkippedFrames)
            {
                m_tracks.erase(m_tracks.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }

    for (size_t i = 0; i < regions.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            m_tracks.push_back(std::make_unique<Track>(regions[i],
                                                    m_settings.m_kalmanType,
                                                    m_settings.m_dt,
                                                    m_settings.m_accelNoiseMag,
                                                    m_nextTrackID++));
        }
    }

    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] != -1) 
        {
            m_tracks[i]->m_skippedFrames = 0;
            m_tracks[i]->Update(regions[assignment[i]], true, grayFrame);
        }
        else				    
        {
            m_tracks[i]->Update(Region(), false, grayFrame);
        }
    }
}
void Tracker::Matching(const regions_t regions, cv::Mat grayFrame)
{

}