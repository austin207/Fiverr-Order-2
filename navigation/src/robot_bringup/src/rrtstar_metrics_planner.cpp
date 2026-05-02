#include <nav2_rrtstar_planner/rrtstar_planner.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <chrono>
#include <fstream>
#include <string>

namespace nav2_rrtstar_metrics_planner {

// Thin wrapper around RRTStar that publishes planning telemetry to
// /planner_metrics after every createPlan() call.
class RRTStarMetricsPlanner : public nav2_rrtstar_planner::RRTStar {
public:
    RRTStarMetricsPlanner() = default;
    ~RRTStarMetricsPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        nav2_rrtstar_planner::RRTStar::configure(parent, name, tf, costmap_ros);
        auto node = parent.lock();
        metrics_pub_ = node->create_publisher<std_msgs::msg::String>("/planner_metrics", 10);
    }

    void activate() override {
        nav2_rrtstar_planner::RRTStar::activate();
        metrics_pub_->on_activate();
    }

    void deactivate() override {
        nav2_rrtstar_planner::RRTStar::deactivate();
        metrics_pub_->on_deactivate();
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override
    {
        long mem_before = readVmRSS();
        auto t0 = std::chrono::high_resolution_clock::now();

        auto path = nav2_rrtstar_planner::RRTStar::createPlan(start, goal);

        auto t1 = std::chrono::high_resolution_clock::now();
        long mem_after = readVmRSS();

        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        long mem_delta_kb = mem_after - mem_before;
        if (mem_delta_kb < 0) mem_delta_kb = 0;

        std_msgs::msg::String out;
        out.data = "{\"PlanTime\":" + std::to_string(ms / 1000.0) +
                   ",\"Mem\":"     + std::to_string(mem_delta_kb) + "}";
        metrics_pub_->publish(out);

        return path;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr metrics_pub_;

    long readVmRSS() {
        std::ifstream f("/proc/self/status");
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("VmRSS:", 0) == 0) {
                long kb;
                std::sscanf(line.c_str(), "VmRSS: %ld", &kb);
                return kb;
            }
        }
        return 0;
    }
};

}  // namespace nav2_rrtstar_metrics_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    nav2_rrtstar_metrics_planner::RRTStarMetricsPlanner,
    nav2_core::GlobalPlanner)
