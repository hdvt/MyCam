#include <iostream>
#include "TrackingSystem.h"

const char* keys =
{
    "{ @1             |../data/video/sunday.mp4  | movie file | }"
    "{ dt detection_type |0                 | select detector type   | }"
    "{ -s output_url | output.mp4                 | save tracked video   | }"
};

// ----------------------------------------------------------------------

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    TrackingSystem mTracking;
    if (mTracking.Init(parser))
        mTracking.Start();
    cv::destroyAllWindows();
    //exit(0);
    return 0;
}