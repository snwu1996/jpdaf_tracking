#include "jpdaf_tracker/jpdaf_tracker.h"


namespace JPDAFTracker
{
    RosJPDAFTracker::RosJPDAFTracker() :
        detection_probability_(1.0f),
        gate_probability_(0.4f),
        local_gating_threshold_(15.0f),
        local_association_cost_(40.0f),
        global_gating_threshold_(0.1f),
        global_association_cost_(50.0f),
        lambda_constant_(2.0f),
        P_1_(10.0f),
        P_2_(10.0f),
        R_1_1_(100.0f),
        R_1_2_(0.0f),
        R_2_1_(0.0f),
        R_2_2_(100.0f),
        dt_(0.4f),
        min_acceptance_rate_(10),
        max_missed_rate_(9)
    {
        // Set up ddynamic reconfigure
        ddr_.registerVariable<double>("detection_probability", &detection_probability_, "", 0.0f, 1.0f);
        ddr_.registerVariable<double>("gate_probability", &gate_probability_, "", 0.0f, 1.0f);
        ddr_.registerVariable<double>("local_gating_threshold_", &local_gating_threshold_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("local_association_cost_", &local_association_cost_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("lambda_constant", &lambda_constant_, "", 0.0f, 10.0f);
        ddr_.registerVariable<double>("P_1", &P_1_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("P_2", &P_2_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("R_1_1", &R_1_1_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("R_1_2", &R_1_2_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("R_2_1", &R_2_1_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("R_2_2", &R_2_2_, "", 0.0f, 100.0f);
        ddr_.registerVariable<double>("dt", &dt_, "", 0.0f, 5.0f);
        ddr_.registerVariable<int>("min_acceptance_rate", &min_acceptance_rate_, "", 0, 100);
        ddr_.registerVariable<int>("max_missed_rate", &max_missed_rate_, "", 0, 100);
        boost::bind(&RosJPDAFTracker::resetTrackerCallback, this, _1);
        // ddr_.registerVariable<bool>("reset_tracker", false,
        //     boost::bind(&RosJPDAFTracker::resetTrackerCallback, this, _1), "");

        loadTrackerParams();

        detection2d_sub_ = nh_.subscribe("detection2d/untracked", 5, &RosJPDAFTracker::updateTrackerCallback, this);
    }

    RosJPDAFTracker::~RosJPDAFTracker() {}

    void RosJPDAFTracker::loadTrackerParams(void)
    {
        tracker_param_.pd = detection_probability_;
        tracker_param_.pg = gate_probability_;
        tracker_param_.g_sigma = local_gating_threshold_;
        tracker_param_.assocCost = local_association_cost_;
        tracker_param_.global_g_sigma = global_gating_threshold_;
        tracker_param_.global_assocCost = global_association_cost_;
        tracker_param_.lambda = lambda_constant_;
        tracker_param_.gamma = lambda_constant_ * 0.000001;
        tracker_param_.target_delta = cv::Point2f(P_1_, P_2_);
        tracker_param_.R << R_1_1_, R_1_2_, R_2_1_, R_2_2_;
        tracker_param_.dt = dt_;
        tracker_param_.min_acceptance_rate = min_acceptance_rate_;
        tracker_param_.max_missed_rate = max_missed_rate_;
    }

    void RosJPDAFTracker::resetTrackerCallback(bool reset)
    {
        ROS_WARN("Resetting tracker");
        reset = false;
    }

    void RosJPDAFTracker::updateTrackerCallback(const vision_msgs::Detection2DArray& detection2d_array_msg)
    {
        ROS_WARN("Updating tracker");
    }

    void RosJPDAFTracker::run() {ros::spin();}

}
