#ifndef ROS_JPDAF_TRACKER_H
#define ROS_JPDAF_TRACKER
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "vision_msgs/Detection2DArray.h"

#include "jpdaf_tracker/tracker.h"
#include "jpdaf_tracker/global_tracker.h"
#include "jpdaf_tracker/detection.h"


namespace JPDAFTracker
{
    class RosJPDAFTracker
    {
        public:
            RosJPDAFTracker();
            ~RosJPDAFTracker();
            void run();
        private:
            void loadTracker(void);
            void resetTrackerCallback(bool reset);
            void updateTrackerCallback(const vision_msgs::Detection2DArray& detection2d_array_msg);

            ros::NodeHandle nh_;
            ros::Publisher tracked_detection2d_pub_;
            ros::Subscriber untracked_detection2d_sub_;

            ddynamic_reconfigure::DDynamicReconfigure ddr_;
            double detection_probability_;
            double gate_probability_;
            double local_gating_threshold_;
            double local_association_cost_;
            double global_gating_threshold_;
            double global_association_cost_;
            double lambda_constant_;
            double P_1_;
            double P_2_;
            double R_1_1_;
            double R_1_2_;
            double R_2_1_;
            double R_2_2_;
            double dt_;
            int min_acceptance_rate_;
            int max_missed_rate_;

            TrackerParam tracker_param_;
            std::shared_ptr<JPDAFTracker::Tracker> tracker_;

            bool paused_;
    };
}

#endif
