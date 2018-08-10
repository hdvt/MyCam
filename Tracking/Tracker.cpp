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
void Tracker::Update(FrameInfo& frameData)
{
    size_t N = m_tracks.size();		
    size_t M = frameData.m_regions.size();
    assignments_t assignment(N, -1); 

    if (!m_tracks.empty())
    {
        distMatrix_t Cost(N * M);
        const track_t maxPossibleCost = frameData.gray.cols * frameData.gray.rows;
        track_t maxCost = 0;
        for (size_t i = 0; i < m_tracks.size(); i++)
        {
            for (size_t j = 0; j < frameData.m_regions.size(); j++)
            {
                auto dist =  m_tracks[i]->CalDistFromAPoint((frameData.m_regions[j].m_rect.tl() + frameData.m_regions[j].m_rect.br()) / 2);
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
                    m_tracks[i]->m_numOfSkippedFrames++;
                }
            }
            else
            {
                m_tracks[i]->m_numOfSkippedFrames++;
            }
        }

        for (int i = 0; i < static_cast<int>(m_tracks.size()); i++)
        {
            if (m_tracks[i]->m_numOfSkippedFrames > m_settings.m_maximumAllowedSkippedFrames)
            {
                m_tracks.erase(m_tracks.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }

    for (size_t i = 0; i < frameData.m_regions.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            m_tracks.push_back(std::make_unique<Track>( m_nextTrackID++,
                                                        frameData.m_regions[i],
                                                        std::move(std::unique_ptr<GKalmanFilter>(new GKalmanFilter(m_settings.m_kalmanType, Point_t((frameData.m_regions[i].m_rect.tl() + frameData.m_regions[i].m_rect.br()) / 2),
                                                                m_settings.m_dt,
                                                                 m_settings.m_accelNoiseMag))) 
                                                        
                                                        ));
        }
    }
    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] != -1) 
        {
            m_tracks[i]->m_numOfSkippedFrames = 0;
            m_tracks[i]->Update(frameData.m_regions[assignment[i]], true);
        }
        else				    
        {
            m_tracks[i]->Update(Region(), false);
        }
    }

}
/*
void Tracker::Update(
        const regions_t& regions, 
        cv::Mat grayFrame
        )
{
    



        size_t realRSize = frameInfo.m_regions.size();
    // for (int j = 0; j < frameInfo.m_motionRegions.size(); j++)
    // {   
    //     if ((frameInfo.m_motionRegions[j].m_rect & frameInfo.m_motionROI).area() > (frameInfo.m_motionRegions[j].m_rect.area()/4))
    //         frameInfo.m_regions.push_back(std::move(frameInfo.m_motionRegions[j]));
    // }
    size_t N = m_tracks.size();		
    size_t M = frameInfo.m_regions.size();
    assignments_t assignment(N, -1); 

    if (!m_tracks.empty())
    {
        distMatrix_t Cost(N * M);
        const track_t maxPossibleCost = frameInfo.gray.cols * frameInfo.gray.rows;
        track_t maxCost = 0;
        for (size_t i = 0; i < m_tracks.size(); i++)
        {
            for (size_t j = 0; j < frameInfo.m_regions.size(); j++)
            {
                auto dist =  m_tracks[i]->CalcDist((frameInfo.m_regions[j].m_rect.tl() + frameInfo.m_regions[j].m_rect.br()) / 2);
                Cost[i + j * N] = dist;
                if (dist > maxCost)
                {
                    maxCost = dist;
                }
            }
        }
        AssignmentProblemSolver APS;
        APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);
        // check in the regions of motion detect
        /*
        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] == -1)
            {
                int minCost = 1000;
                for (int j = 0; j < frameInfo.m_motionRegions.size(); j++)
                {   
                    if ((frameInfo.m_motionRegions[j].m_rect & frameInfo.m_motionROI).area() == 0)
                        continue;
                    auto dist = m_tracks[i]->CalcDist((frameInfo.m_motionRegions[j].m_rect.tl() + frameInfo.m_motionRegions[j].m_rect.br()) / 2);
                    if (dist < m_settings.distThres)
                    {
                        assignment[i] = frameInfo.m_regions.size() + j;
                        break;
                    }
                }inbox
            }
        }
        ----
        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] != -1 && assignment[i] < frameInfo.m_regions.size())
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

    for (size_t i = 0; i < realRSize; ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end() )
        {
            m_tracks.push_back(std::make_unique<Track>(frameInfo.m_regions[i],
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
          //  if (assignment[i] < realRSize)
            m_tracks[i]->Update(frameInfo.m_regions[assignment[i]], true, frameInfo.gray);
          //  else
        //        m_tracks[i]->Update(frameInfo.m_motionRegions[assignment[i] - frameInfo.m_regions.size()], true, frameInfo.gray);

        }
        else				    
        {
            m_tracks[i]->Update(Region(), false, frameInfo.gray);
        }
    }
}

void Tracker::Matching(const regions_t regions, cv::Mat grayFrame)
{

}
*/