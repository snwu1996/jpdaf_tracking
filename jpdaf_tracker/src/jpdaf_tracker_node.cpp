#include "jpdaf_tracker/jpdaf_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jpdaf_tracker_node");
    JPDAFTracker::RosJPDAFTracker jpdaf_tracker;
    jpdaf_tracker.run();
    return 0;
}
