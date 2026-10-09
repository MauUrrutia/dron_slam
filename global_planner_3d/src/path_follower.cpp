#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/exceptions.hpp>

#include <cmath>
#include <limits>

using std::placeholders::_1;

class PathFollower: public rclcpp::Node
{
private:

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        current_path_ = *msg;
        have_path_ = !current_path_.poses.empty();
        target_idx_ = 0;
        if (!have_path_){
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }
        have_hold_ = false; 
        mission_done_ = false;
        RCLCPP_INFO(this->get_logger(), "Received /global_path with %zu poses",
                                         current_path_.poses.size());    
        
    }


    void timerCallback(){
        if (have_hold_){
            geometry_msgs::msg::PoseStamped sp = hold_sp_;
            sp.header.stamp = this->now();
            sp.header.frame_id = "map";
            nav_setpoint_pub_->publish(sp);

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Publishing HOLD setpoint");
            return;
        }
        if (mission_done_) {
            geometry_msgs::msg::PoseStamped sp = mission_hold_sp_;
            sp.header.stamp = this->now();
            sp.header.frame_id = "map";
            nav_setpoint_pub_->publish(sp);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Mission done: holding final position");
            return;
        }
        if(!have_path_ || current_path_.poses.empty()) return;

        geometry_msgs::msg::TransformStamped tf_map_base;
        try{
            tf_map_base = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF map->base_footprint unavailable: %s", ex.what());
            return;
        }
        const double rx = tf_map_base.transform.translation.x;
        const double ry = tf_map_base.transform.translation.y;
        const double rz = tf_map_base.transform.translation.z;

        while (target_idx_ + 1 < current_path_.poses.size()){
            const auto &wp = current_path_.poses[target_idx_].pose.position;
            const double dx = wp.x - rx;
            const double dy = wp.y - ry;
            const double dz = wp.z - rz;
            const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        

            if (dist < reach_radius_) {
                target_idx_++;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Waypoint reached, advancing to idx=%zu", target_idx_);
            }else{
                break;
            } 
        }

        if (target_idx_ == current_path_.poses.size() - 1) {
            const auto &goal = current_path_.poses.back().pose.position;
            const double gx = goal.x - rx;
            const double gy = goal.y - ry;
            const double gz = goal.z - rz;
            const double gdist = std::sqrt(gx*gx + gy*gy + gz*gz);

            if (gdist < reach_radius_) {
                mission_done_ = true;
                have_path_ = false;  // deja de trackear path
                mission_hold_sp_ = current_path_.poses.back();
                mission_hold_sp_.header.frame_id = "map";
                mission_hold_sp_.header.stamp = this->now();

                RCLCPP_INFO(this->get_logger(),
                    "Goal reached. Switching to mission HOLD.");
                return;
            }
        }

        const size_t publish_idx =
        std::min(target_idx_ + lookahead_points_, current_path_.poses.size() - 1);

        geometry_msgs::msg::PoseStamped sp = current_path_.poses[publish_idx];
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        nav_setpoint_pub_->publish(sp);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Tracking wp %zu (target=%zu)/%zu -> (%.2f, %.2f, %.2f)",
            publish_idx, target_idx_, current_path_.poses.size()-1,
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z);

    }

    void holdCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        hold_sp_ = *msg;
        have_hold_ = true;
        have_path_ = false;
        mission_done_ = false;   
        current_path_.poses.clear();
        target_idx_ = 0;
        RCLCPP_WARN(this->get_logger(),
            "HOLD received at (%.2f, %.2f, %.2f)",
            hold_sp_.pose.position.x,
            hold_sp_.pose.position.y,
            hold_sp_.pose.position.z);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_setpoint_pub_;
    
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path current_path_;
    bool have_path_{false};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    size_t target_idx_{};
    double reach_radius_{0.30};

    size_t lookahead_points_{3};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hold_sub_;

    geometry_msgs::msg::PoseStamped hold_sp_;
    bool have_hold_{false};

    bool mission_done_{false};
    geometry_msgs::msg::PoseStamped mission_hold_sp_;
public:
    PathFollower(): Node("path_follower_6d"){
        path_sub_ = create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&PathFollower::pathCallback, this, _1));
        nav_setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav/setpoint", 10);
        hold_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/nav/hold_setpoint", 10, std::bind(&PathFollower::holdCallback, this, _1));
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::timerCallback, this));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}