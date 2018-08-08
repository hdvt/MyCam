#include <iostream>
#include "TrackingSystem.h"

const char* keys =
{
    "{ @1             |../data/sunday.mp4  | movie file | }"
    "{ dt detection_type |0                 | select detector type   | }"
};

// ----------------------------------------------------------------------

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    TrackingSystem mTracking(parser);
    mTracking.Process();
    cv::destroyAllWindows();
    //exit(0);
    return 0;
}