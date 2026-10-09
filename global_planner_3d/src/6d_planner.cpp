#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/exceptions.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <cmath>
#include <functional>
#include <cstddef>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <algorithm>

using namespace std::placeholders;

struct Node2D
{
    int ix{}, iy{};
    double g{}, h{};
    int64_t parent_key{-1};

    double f() const { return g + h; }
};


struct Node3D
{
    int ix, iy, iz;
    double g{}, h{};
    int64_t  parent_key{-1}; //No starting parent

    double f() const {return g + h;}    


};


class Nav6DPlanner : public rclcpp::Node
{
private:
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose){
        last_goal_ = *goal_pose;
        have_goal_ = true;
        goal_updated_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Goal received: frame=%s pos=(%.2f, %.2f, %.2f)",
            last_goal_.header.frame_id.c_str(),
            last_goal_.pose.position.x,
            last_goal_.pose.position.y,
            last_goal_.pose.position.z);

        tryPlan();
    }
    void octoMapCallback(const octomap_msgs::msg::Octomap::SharedPtr map){
        last_map_ = map;
        have_map_ = true;
        map_updated_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Octomap received: frame=%s binary=%s size=%zu",
            last_map_->header.frame_id.c_str(),
            last_map_->binary ? "true" : "false",
            last_map_->data.size());

        tryPlan();
    }
    
    void tryPlan(){
        if(!have_map_){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty map still");
            return;
        }
        if(!have_goal_){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty goal still");
            return;
        }
        if(!goal_updated_){
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "Goal not updated, skipping replanning");
            return;
        }
        geometry_msgs::msg::TransformStamped tf_map_base;
        try{
            tf_map_base = tf_buffer_->lookupTransform(
                "map",
                "base_footprint",
                tf2::TimePointZero
            );
        } catch(const tf2::TransformException & ex){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF map->base_footprint unavailable: %s", ex.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Planning trigger: map+goal available, trying TF map->base_footprint");
        geometry_msgs::msg::PoseStamped start;
        start.header.stamp = this->now();
        start.header.frame_id = "map";
        start.pose.position.x = tf_map_base.transform.translation.x;
        start.pose.position.y = tf_map_base.transform.translation.y;
        start.pose.position.z = tf_map_base.transform.translation.z;
        start.pose.orientation = tf_map_base.transform.rotation;

        auto path = planPath(last_map_, start, last_goal_);
        if(!path.poses.empty()){
            path.poses.back().pose.orientation = last_goal_.pose.orientation;
            global_path_pub_->publish(path);
            last_plan_time_ = this->now();
            last_path_ = path;
            have_last_path_ = true;
            goal_updated_ = false; 
            map_updated_ = false;
            replan_fail_count_ = 0;
            blocked_consecutive_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path.poses.size());
        } else {
            replan_fail_count_++;
            RCLCPP_WARN(this->get_logger(), "Empty path (fail_count=%d/%d)",
                        replan_fail_count_, replan_fail_limit_);
            if (replan_fail_count_ >= replan_fail_limit_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Replan failed repeatedly. Consider HOLD/hover fallback.");
                publishHoldAtCurrentPose();
            }
        }
    }

    nav_msgs::msg::Path planPath(const octomap_msgs::msg::Octomap::SharedPtr map,
                             const geometry_msgs::msg::PoseStamped &start_pose,
                             const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        if (goal_pose.header.frame_id != "map") {
            RCLCPP_WARN(this->get_logger(), "Goal frame is '%s', expected 'map'",
                        goal_pose.header.frame_id.c_str());
            return path;
        }

        geometry_msgs::msg::PoseStamped start = start_pose;
        start.header.stamp = this->now();
        start.header.frame_id = "map";

        geometry_msgs::msg::PoseStamped goal = goal_pose;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";

        octomap::AbstractOcTree *tree_abs = octomap_msgs::msgToMap(*map);
        if (!tree_abs) {
            RCLCPP_ERROR(this->get_logger(), "msgToMap failed");
            return path;
        }

        auto *octree = dynamic_cast<octomap::OcTree*>(tree_abs);
        if (!octree) {
            RCLCPP_ERROR(this->get_logger(), "Map is not an OcTree");
            delete tree_abs;
            return path;
        }

        const double res = octree->getResolution();
        RCLCPP_INFO(this->get_logger(), "Octree resolution: %.3f m", res);

        enum class VoxelState { FREE, OCCUPIED, UNKNOWN };
        auto voxelState = [&](double x, double y, double z) -> VoxelState {
            auto *node = octree->search(x, y, z);
            if (!node) return VoxelState::UNKNOWN;
            return octree->isNodeOccupied(node) ? VoxelState::OCCUPIED : VoxelState::FREE;
        };

        // -------- Parámetros de seguridad --------
        const double robot_radius = 0.20;
        const double safety_margin = 0.05;
        const double inflation_radius_xy = robot_radius + safety_margin;
        const int infl_cells_xy = static_cast<int>(std::ceil(inflation_radius_xy / res));

        // Altitud de crucero: puedes tunear o parametrizar
        const double z_start = start.pose.position.z;
        const double z_goal  = goal.pose.position.z;
        const double z_cruise = std::max(z_start, z_goal); // +50cm sobre el más alto

        // Validaciones básicas start/goal en su z real
        if (voxelState(start.pose.position.x, start.pose.position.y, z_start) == VoxelState::OCCUPIED) {
            RCLCPP_WARN(this->get_logger(), "Start is OCCUPIED");
            delete tree_abs;
            return path;
        }
        if (voxelState(goal.pose.position.x, goal.pose.position.y, z_goal) == VoxelState::OCCUPIED) {
            RCLCPP_WARN(this->get_logger(), "Goal is OCCUPIED");
            delete tree_abs;
            return path;
        }

        // Verificar líneas verticales (subida/bajada)
        if (!isVerticalSegmentFree(octree, start.pose.position.x, start.pose.position.y, z_start, z_cruise, res, robot_radius)) {
            RCLCPP_WARN(this->get_logger(), "Vertical segment at START blocked");
            delete tree_abs;
            return path;
        }
        if (!isVerticalSegmentFree(octree, goal.pose.position.x, goal.pose.position.y, z_goal, z_cruise, res, robot_radius)) {
            RCLCPP_WARN(this->get_logger(), "Vertical segment at GOAL blocked");
            delete tree_abs;
            return path;
        }

        // Grid 2D
        Node2D start_n = worldToGrid2D(start.pose, res);
        Node2D goal_n  = worldToGrid2D(goal.pose, res);

        const int64_t start_key = makeKey2D(start_n.ix, start_n.iy);
        const int64_t goal_key  = makeKey2D(goal_n.ix, goal_n.iy);

        // Ocupación inflada 2D evaluada en z_cruise
        std::unordered_map<int64_t, bool> inflated2d_cache;
        inflated2d_cache.reserve(200000);

        auto isInflatedOccupied2D = [&](const Node2D &n) -> bool {
            const int64_t k = makeKey2D(n.ix, n.iy);
            auto it = inflated2d_cache.find(k);
            if (it != inflated2d_cache.end()) return it->second;

            bool occ = false;
            const double cx = (n.ix + 0.5) * res;
            const double cy = (n.iy + 0.5) * res;

            const double z_band_half = 0.10; // tunear
            for (int dx = -infl_cells_xy; dx <= infl_cells_xy; ++dx) {
                for (int dy = -infl_cells_xy; dy <= infl_cells_xy; ++dy) {
                    const double dxy = std::sqrt(double(dx*dx + dy*dy)) * res;
                    if (dxy > inflation_radius_xy) continue;

                    const double wx = cx + dx * res;
                    const double wy = cy + dy * res;

                    bool occ_here = false;
                    for (double zz = z_cruise - z_band_half; zz <= z_cruise + z_band_half; zz += res) {
                        auto st = voxelState(wx, wy, zz);
                        if (st == VoxelState::OCCUPIED) { // OCCUPIED
                            occ_here = true;
                            break;
                        }
                    }
                    if (occ_here) { occ = true; break; }
                }
                if (occ) break;
            }

            inflated2d_cache[k] = occ;
            return occ;
        };

        if (isInflatedOccupied2D(start_n)) {
            RCLCPP_WARN(this->get_logger(), "Start blocked (inflated 2D at z_cruise)");
            delete tree_abs;
            return path;
        }
        if (isInflatedOccupied2D(goal_n)) {
            RCLCPP_WARN(this->get_logger(), "Goal blocked (inflated 2D at z_cruise)");
            delete tree_abs;
            return path;
        }

        auto heuristic = [&](const Node2D &a, const Node2D &b){
            const double dx = double(a.ix - b.ix);
            const double dy = double(a.iy - b.iy);
            return std::sqrt(dx*dx + dy*dy);
        };

        struct CompareNode2D{
            bool operator()(const Node2D &lhs, const Node2D &rhs) const {
                return lhs.f() > rhs.f();
            }
        };

        // 8-conectado (mejor ruta y menos "escalones")
        const std::vector<std::tuple<int,int>> neigh = {
            { 1, 0}, {-1, 0}, {0,  1}, {0, -1},
            { 1, 1}, { 1,-1}, {-1,1}, {-1,-1}
        };

        std::priority_queue<Node2D, std::vector<Node2D>, CompareNode2D> open_set;
        std::unordered_set<int64_t> closed_set;
        std::unordered_map<int64_t, double> best_g;
        std::unordered_map<int64_t, Node2D> came_from;

        start_n.g = 0.0;
        start_n.h = heuristic(start_n, goal_n);
        start_n.parent_key = -1;

        open_set.push(start_n);
        best_g[start_key] = start_n.g;
        came_from[start_key] = start_n;

        bool found = false;
        int64_t found_goal_key = -1;
        const int max_iterations = 150000;
        int iterations = 0;

        while (!open_set.empty()) {
            if (++iterations > max_iterations) {
                RCLCPP_WARN(this->get_logger(), "A* 2D aborted: max_iterations reached");
                break;
            }

            Node2D current = open_set.top();
            open_set.pop();

            const int64_t cur_key = makeKey2D(current.ix, current.iy);
            if (closed_set.count(cur_key)) continue;
            closed_set.insert(cur_key);

            if (cur_key == goal_key) {
                found = true;
                found_goal_key = cur_key;
                break;
            }

            for (const auto &[dx, dy] : neigh) {
                Node2D nb;
                nb.ix = current.ix + dx;
                nb.iy = current.iy + dy;

                if (isInflatedOccupied2D(nb)) continue;

                const int64_t nb_key = makeKey2D(nb.ix, nb.iy);
                if (closed_set.count(nb_key)) continue;

                const double step = std::sqrt(double(dx*dx + dy*dy)); // 1 o sqrt(2)
                // Penaliza desconocido en z_cruise
                const auto nb_pose = gridToWorld2D(nb, res, z_cruise);
                const auto st = voxelState(nb_pose.position.x, nb_pose.position.y, nb_pose.position.z);
                const double unknown_penalty = (st == VoxelState::UNKNOWN) ? 1.0 : 0.0;

                const double tentative_g = current.g + step + unknown_penalty;

                auto it = best_g.find(nb_key);
                if (it == best_g.end() || tentative_g < it->second) {
                    nb.g = tentative_g;
                    nb.h = heuristic(nb, goal_n);
                    nb.parent_key = cur_key;

                    best_g[nb_key] = nb.g;
                    came_from[nb_key] = nb;
                    open_set.push(nb);
                }
            }
        }

        if (!found) {
            RCLCPP_WARN(this->get_logger(), "A* 2D could not find a path");
            delete tree_abs;
            return path;
        }

        // Reconstrucción 2D
        std::vector<Node2D> nodes2d;
        int64_t k = found_goal_key;
        while (k != -1) {
            auto it = came_from.find(k);
            if (it == came_from.end()) break;
            nodes2d.push_back(it->second);
            k = it->second.parent_key;
        }
        std::reverse(nodes2d.begin(), nodes2d.end());

        // Construcción path 3D: vertical inicio -> crucero XY -> vertical final
        path.poses.clear();
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        auto pushPose = [&](double x, double y, double z){
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            ps.pose.position.z = z;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        };

        // 1) vertical inicio
        {
            const double dz = (z_cruise >= z_start) ? res : -res;
            if (std::abs(z_cruise - z_start) < 1e-6) {
                pushPose(start.pose.position.x, start.pose.position.y, z_start);
            } else {
                for (double z = z_start; (dz > 0) ? (z <= z_cruise) : (z >= z_cruise); z += dz) {
                    pushPose(start.pose.position.x, start.pose.position.y, z);
                }
            }
        }

        // 2) tramo XY a z_cruise
        for (const auto &n : nodes2d) {
            auto p = gridToWorld2D(n, res, z_cruise);
            pushPose(p.position.x, p.position.y, p.position.z);
        }

        // 3) vertical final (desde último XY al goal.z)
        {
            // opcional: aterrizar exactamente en x,y del goal
            pushPose(goal.pose.position.x, goal.pose.position.y, z_cruise);

            const double dz = (z_goal >= z_cruise) ? res : -res;
            if (std::abs(z_goal - z_cruise) < 1e-6) {
                pushPose(goal.pose.position.x, goal.pose.position.y, z_goal);
            } else {
                for (double z = z_cruise; (dz > 0) ? (z <= z_goal) : (z >= z_goal); z += dz) {
                    pushPose(goal.pose.position.x, goal.pose.position.y, z);
                }
            }
        }

        // Orientación final = goal
        if (!path.poses.empty()) {
            path.poses.back().pose.orientation = goal.pose.orientation;
        }

        delete tree_abs;
        return path;
    }

    Node3D worldToGrid(const geometry_msgs::msg::Pose &pose, double resolution) const{
        Node3D node;
        
        double scaled_x = pose.position.x / resolution;
        double scaled_y = pose.position.y / resolution;
        double scaled_z = pose.position.z / resolution;
        
        node.ix = floor(scaled_x);
        node.iy = floor(scaled_y);
        node.iz = floor(scaled_z);
        return node;
    }

    geometry_msgs::msg::Pose gridToWorld(const Node3D &node, double resolution) const{
        geometry_msgs::msg::Pose pose;
        
        pose.position.x = (node.ix + 0.5) * resolution;
        pose.position.y = (node.iy + 0.5) * resolution;
        pose.position.z = (node.iz + 0.5) * resolution;
        
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        return pose;
    }
    int64_t makeKey(const int ix, const int iy, const int iz) const{
        
        //patrón clásico llamado hash_combine
        
        std::size_t seed{};
        auto hash_combined = [&seed](std::size_t v){
            seed^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        };

        hash_combined(std::hash<int>{}(ix));
        hash_combined(std::hash<int>{}(iy));
        hash_combined(std::hash<int>{}(iz));

        return static_cast<int64_t>(seed);
    }

    bool isCurrentPathBlocked(){
        if(!have_last_path_ || !have_map_ || !last_map_ || last_path_.poses.empty()) return false;

        octomap::AbstractOcTree *tree_abs = octomap_msgs::msgToMap(*last_map_);
        if(!tree_abs) return false;
        auto *octree = dynamic_cast<octomap::OcTree*>(tree_abs);
        if(!octree){delete tree_abs; return false;}

        const double res = octree->getResolution();
        const int cells = static_cast<int>(std::ceil(path_check_radius_ / res));

        for (const auto &i : last_path_.poses)
        {
            const auto &p = i.pose.position;
            const int cx = static_cast<int>(std::floor(p.x / res));
            const int cy = static_cast<int>(std::floor(p.y / res));
            const int cz = static_cast<int>(std::floor(p.z / res));
            for(int dx=-cells; dx<=cells; ++dx){
                for(int dy=-cells; dy<=cells; ++dy){
                    for(int dz=-cells; dz<=cells; ++dz){
                        const double d = std::sqrt(double(dx*dx + dy*dy + dz*dz)) * res;
                        if(d > path_check_radius_) continue;

                        const double wx = (cx + dx + 0.5) * res;
                        const double wy = (cy + dy + 0.5) * res;
                        const double wz = (cz + dz + 0.5) * res;

                        auto *n = octree->search(wx, wy, wz);
                        if (n && octree->isNodeOccupied(n)) {
                            delete tree_abs;
                            return true;
                        }
                    }
                }
            }   
        }
        delete tree_abs;
        return false;
    }

    void replanTimerCallback(){
        if(!have_goal_ || !have_map_) return;
        if(replan_fail_count_ >= replan_fail_limit_) return;

        const auto now_t = this->now();
        const bool in_cooldown =
            (last_plan_time_.nanoseconds() > 0) &&
            ((now_t - last_plan_time_).seconds() < replan_cooldown_s_);

        // Si hay mapa nuevo, sí permite replanning inmediato
        if(map_updated_){
            goal_updated_ = true;
            tryPlan();
            blocked_consecutive_count_ = 0;
            return;
        }

        // Durante cooldown, ignorar chequeo de bloqueo
        if(in_cooldown) return;

        const bool blocked = isCurrentPathBlocked();
        if (blocked) blocked_consecutive_count_++;
        else blocked_consecutive_count_ = 0;

        if (blocked_consecutive_count_ >= blocked_confirm_cycles_){
            RCLCPP_WARN(this->get_logger(), "Current path blocked (confirmed) -> replanning");
            goal_updated_ = true;
            tryPlan();
            blocked_consecutive_count_ = 0;
        }
    }

    void publishHoldAtCurrentPose()
    {
        geometry_msgs::msg::TransformStamped tf_map_base;
        try{
            tf_map_base = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch(const tf2::TransformException & ex){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "HOLD TF map->base_footprint unavailable: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped hold;
        hold.header.stamp = now();
        hold.header.frame_id = "map";
        hold.pose.position.x = tf_map_base.transform.translation.x;
        hold.pose.position.y = tf_map_base.transform.translation.y;
        hold.pose.position.z = tf_map_base.transform.translation.z;
        hold.pose.orientation = tf_map_base.transform.rotation;

        hold_setpoint_pub_->publish(hold);
    }
    // Helpers 2D (agregar dentro de la clase Nav6DPlanner, en private:)
    int64_t makeKey2D(const int ix, const int iy) const{
        std::size_t seed{};
        auto hash_combine = [&seed](std::size_t v){
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        };
        hash_combine(std::hash<int>{}(ix));
        hash_combine(std::hash<int>{}(iy));
        return static_cast<int64_t>(seed);
    }

    Node2D worldToGrid2D(const geometry_msgs::msg::Pose &pose, double resolution) const{
        Node2D n;
        n.ix = static_cast<int>(std::floor(pose.position.x / resolution));
        n.iy = static_cast<int>(std::floor(pose.position.y / resolution));
        return n;
    }

    geometry_msgs::msg::Pose gridToWorld2D(const Node2D &n, double resolution, double z) const{
        geometry_msgs::msg::Pose p;
        p.position.x = (n.ix + 0.5) * resolution;
        p.position.y = (n.iy + 0.5) * resolution;
        p.position.z = z;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;
        return p;
    }

    bool isVerticalSegmentFree(
        octomap::OcTree *octree,
        double x, double y,
        double z0, double z1,
        double res,
        double radius_xy = 0.20) const
    {
        const double zmin = std::min(z0, z1);
        const double zmax = std::max(z0, z1);

        const int cells_xy = static_cast<int>(std::ceil(radius_xy / res));
        for (double z = zmin; z <= zmax; z += res) {
            for (int dx = -cells_xy; dx <= cells_xy; ++dx) {
                for (int dy = -cells_xy; dy <= cells_xy; ++dy) {
                    const double dxy = std::sqrt(double(dx*dx + dy*dy)) * res;
                    if (dxy > radius_xy) continue;

                    const double wx = x + (dx * res);
                    const double wy = y + (dy * res);

                    auto *node = octree->search(wx, wy, z);
                    if (node && octree->isNodeOccupied(node)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;

    geometry_msgs::msg::PoseStamped last_goal_;
    octomap_msgs::msg::Octomap::SharedPtr last_map_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; 
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    bool have_map_{false};
    bool have_goal_{false};
    bool goal_updated_{false};

    //obstacle avoidance

    rclcpp::TimerBase::SharedPtr replan_timer_;
    bool map_updated_{false};
    bool have_last_path_{false};
    nav_msgs::msg::Path last_path_;

    double replan_rate_hz_{2.0}; //2 Hz 
    double path_check_radius_{0.10}; //inflation radius
    int replan_fail_count_{};
    int replan_fail_limit_{5};
    int blocked_consecutive_count_{0};
    int blocked_confirm_cycles_{3};
    rclcpp::Time last_plan_time_;
    double replan_cooldown_s_{3.0};

    double astar_weight_{1.4};        // 1.0 = A* clásico, >1 acelera
    double planning_margin_m_{3.0};   // margen caja de búsqueda

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hold_setpoint_pub_;


public:
    Nav6DPlanner(): Node("nav_6d_planner"){
        
        RCLCPP_INFO(this->get_logger(), "node initialized");
        
        goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose_3d", 
            10, std::bind(&Nav6DPlanner::goalPoseCallback, this, _1));
    
    
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", 
            rclcpp::QoS(1).transient_local().reliable(), std::bind(&Nav6DPlanner::octoMapCallback, this, _1));
    
        global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        hold_setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav/hold_setpoint", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        replan_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / replan_rate_hz_)),
                                                std::bind(&Nav6DPlanner::replanTimerCallback, this));
    }

};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav6DPlanner>());
    rclcpp::shutdown();
    return 0;
}