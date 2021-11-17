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
        max_missed_rate_(9),
        paused_(true)
    {
        // Set up ddynamic reconfigure
        ddr_.registerVariable<double>("detection_probability", &detection_probability_, "", 0.0f, 1.0f);
        ddr_.registerVariable<double>("gate_probability", &gate_probability_, "", 0.0f, 1.0f);
        ddr_.registerVariable<double>("local_gating_threshold", &local_gating_threshold_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("local_association_cost", &local_association_cost_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("global_gating_threshold", &global_gating_threshold_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("global_association_cost", &global_association_cost_, "", 1.0f, 100.0f);
        ddr_.registerVariable<double>("lambda_constant", &lambda_constant_, "", 0.0f, 10.0f);
        ddr_.registerVariable<double>("P_1", &P_1_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("P_2", &P_2_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("R_1_1", &R_1_1_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("R_1_2", &R_1_2_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("R_2_1", &R_2_1_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("R_2_2", &R_2_2_, "", 0.0f, 500.0f);
        ddr_.registerVariable<double>("dt", &dt_, "", 0.0f, 5.0f);
        ddr_.registerVariable<int>("min_acceptance_rate", &min_acceptance_rate_, "", 0, 100);
        ddr_.registerVariable<int>("max_missed_rate", &max_missed_rate_, "", 0, 100);
        ddr_.registerVariable<bool>("reset_tracker", false,
            boost::bind(&RosJPDAFTracker::resetTrackerCallback, this, _1), "");
        ddr_.publishServicesTopics();

        loadTracker();
        tracked_detection2d_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("detection2d/tracked", 5);
        untracked_detection2d_sub_ = nh_.subscribe("detection2d/untracked", 5, &RosJPDAFTracker::updateTrackerCallback, this);
    }

    RosJPDAFTracker::~RosJPDAFTracker() {}

    void RosJPDAFTracker::loadTracker(void)
    {
        ROS_WARN("Pausing jpdaf_tracker_node to load tracker");
        paused_ = true;

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

        tracker_ = std::make_shared<JPDAFTracker::GlobalTracker>(tracker_param_);

        ros::Duration(1.0).sleep();
        ROS_WARN("Unpausing jpdaf_tracker_node");
        paused_ = false;
    }

    void RosJPDAFTracker::resetTrackerCallback(bool reset)
    {
        if (!reset) { return; }

        ROS_WARN_STREAM("Resetting tracker with new params:\n"<<
            "\tdetection_probability: " << std::to_string(detection_probability_) << "\n" <<
            "\tgate_probability: " << std::to_string(gate_probability_) << "\n" <<
            "\tlocal_gating_threshold: " << std::to_string(local_gating_threshold_) << "\n" <<
            "\tlocal_association_cost: " << std::to_string(local_association_cost_) << "\n" <<
            "\tglobal_gating_threshold: " << std::to_string(global_gating_threshold_) << "\n" <<
            "\tglobal_association_cost: " << std::to_string(global_association_cost_) << "\n" <<
            "\tlambda_constant: " << std::to_string(lambda_constant_) << "\n" <<
            "\tgamma: " << std::to_string(lambda_constant_ * 0.000001) << "\n" <<
            "\ttarget_delta: [" << std::to_string(P_1_) << " " << std::to_string(P_2_) << "]\n" <<
            "\tR: [[" << std::to_string(R_1_1_) << " " << std::to_string(R_1_2_) << "]\n" <<
            "\t    [" << std::to_string(R_2_1_) << " " << std::to_string(R_2_2_) << "]]\n" <<
            "\tdt: " << std::to_string(dt_) << "\n" <<
            "\tmin_acceptance_rate: " << std::to_string(min_acceptance_rate_) << "\n" <<
            "\tmax_missed_rate: " << std::to_string(max_missed_rate_));

        loadTracker();
    }

    void RosJPDAFTracker::updateTrackerCallback(const vision_msgs::Detection2DArray& untracked_detection2d_array_msg)
    {
        if (paused_) { return; }

        // Update tracker
        std::vector<JPDAFTracker::Detection> detections;
        for (const auto& untracked_detection2d_msg : untracked_detection2d_array_msg.detections)
        {
            int size_x = (int) untracked_detection2d_msg.bbox.size_x;
            int size_y = (int) untracked_detection2d_msg.bbox.size_y;
            int x = (int) untracked_detection2d_msg.bbox.center.x;
            int y = (int) untracked_detection2d_msg.bbox.center.y;
            detections.emplace_back(x, y, size_x, size_y);
        }
        tracker_->track(detections);

        // Pack tracks into Detection2DArray message and publish
        auto tracks = tracker_->tracks();
        vision_msgs::Detection2DArray tracked_detection2d_array_msg;
        tracked_detection2d_array_msg.header = untracked_detection2d_array_msg.header;
        for (const auto track : tracks)
        {
            vision_msgs::Detection2D tracked_detection2d_msg;
            tracked_detection2d_msg.header = untracked_detection2d_array_msg.header;

            const cv::Point& pt = track->getLastPrediction();
            tracked_detection2d_msg.bbox.center.x = pt.x;
            tracked_detection2d_msg.bbox.center.y = pt.y;
            // TODO(snwu): Use the size_x and size_y of last seen detection
            tracked_detection2d_msg.bbox.size_x = 50;
            tracked_detection2d_msg.bbox.size_y = 50;

            vision_msgs::ObjectHypothesisWithPose result_msg;
            result_msg.id = track->getId();
            // TODO(snwu): Assign score relative to track covariance
            result_msg.score = 100.0;
            tracked_detection2d_msg.results.push_back(result_msg);

            tracked_detection2d_array_msg.detections.push_back(tracked_detection2d_msg);
        }
        tracked_detection2d_pub_.publish(tracked_detection2d_array_msg);
    }

    void RosJPDAFTracker::run()
    {
        ros::spin();
    }

}
