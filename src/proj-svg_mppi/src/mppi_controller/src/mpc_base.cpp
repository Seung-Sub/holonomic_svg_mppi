#include "mppi_controller/mpc_base.hpp"
#include <tf2/utils.h>
#include <ros/ros.h>

namespace mppi {
namespace cpu {
    // #### Public functions ####

    MPCBase::MPCBase(const Params::Common& params, const size_t& sample_num)
        : is_reference_path_less_mode_(params.is_reference_path_less_mode),
          thread_num_(params.thread_num),
          prediction_step_size_(static_cast<size_t>(params.prediction_step_size)),
          prediction_interval_(params.prediction_interval),
          lookahead_distance_(params.lookahead_distance),
          q_dist_(params.q_dist),
          q_angle_(params.q_angle),
          collision_weight_(params.collision_weight),
          q_terminal_dist_(params.q_terminal_dist),
          q_terminal_angle_(params.q_terminal_angle)
    {
        // initialize inner variables
        global_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
        local_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
    }

    void MPCBase::set_obstacle_map(const grid_map::GridMap& obstacle_map) { obstacle_map_ = obstacle_map; }

    void MPCBase::set_reference_path(const nav_msgs::Path& reference_path) { reference_path_ = reference_path; }

    void MPCBase::set_single_goal(const geometry_msgs::PoseStamped& goal) { single_goal_ = goal; has_single_goal_ = true; }

    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state) {
        if (is_reference_path_less_mode_) {
            return calc_sample_costs(sampler, init_state, obstacle_map_, reference_path_, &global_state_seq_candidates_, &local_state_seq_candidates_);
        } else {
            return calc_sample_costs(sampler, init_state, obstacle_map_, single_goal_, &global_state_seq_candidates_, &local_state_seq_candidates_);
        }
    }

    std::tuple<StateSeq, double, double> MPCBase::get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const {
        if (is_reference_path_less_mode_) {
            StateSeq global_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            StateSeq local_sate_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            predict_state_seq(control_input_seq, initial_state, &global_state_seq, &local_sate_seq);
            const auto [cost, collision_cost] = state_cost(global_state_seq, local_sate_seq, obstacle_map_, reference_path_);
            return std::make_tuple(global_state_seq, cost - collision_cost, collision_cost);
        } else {
            StateSeq global_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            StateSeq local_sate_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            predict_state_seq(control_input_seq, initial_state, &global_state_seq, &local_sate_seq);
            const auto [cost, collision_cost] = state_cost(global_state_seq, local_sate_seq, obstacle_map_, single_goal_);
            return std::make_tuple(global_state_seq, cost - collision_cost, collision_cost);
        }
    }

    std::pair<std::vector<StateSeq>, std::vector<double>> MPCBase::get_state_seq_candidates(const int& _num_samples,
                                                                                            const std::vector<double>& weights) const {
        if (weights.size() == 0) {
            std::cerr << "weights is empty" << std::endl;
            return std::make_pair(std::vector<StateSeq>(), std::vector<double>());
        }

        const int num_samples = std::min(static_cast<int>(weights.size()), _num_samples);

        std::vector<double> sorted_weights = weights;
        std::sort(sorted_weights.data(), sorted_weights.data() + sorted_weights.size());

        // get indices of top num_samples
        std::vector<int> indices;
        for (int i = 0; i < num_samples; i++) {
            const double weight = sorted_weights[sorted_weights.size() - 1 - i];
            const int index = std::distance(weights.data(), std::find(weights.data(), weights.data() + weights.size(), weight));
            indices.push_back(index);
        }

        std::vector<double> selected_weights(num_samples);
        std::vector<StateSeq> selected_state_seq_candidates(num_samples);
        for (int i = 0; i < num_samples; i++) {
            selected_weights[i] = weights[indices.at(i)];
            selected_state_seq_candidates[i] = global_state_seq_candidates_.at(indices.at(i));
        }

        return std::make_pair(selected_state_seq_candidates, selected_weights);
    }

    std::pair<StateSeq, XYCovMatrices> MPCBase::get_proposed_distribution() const {
        // if (is_localize_less_mode_) {
        //     return calc_state_distribution(local_state_seq_candidates_);
        // } else {
        //     return calc_state_distribution(global_state_seq_candidates_);
        // }
        return calc_state_distribution(global_state_seq_candidates_);
    }

    // #### Private functions ####
    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                                   const State& global_init_state,
                                                                                   const grid_map::GridMap& obstacle_map,
                                                                                   const geometry_msgs::PoseStamped& goal,
                                                                                   StateSeqBatch* global_state_seq_candidates,
                                                                                   StateSeqBatch* local_state_seq_candidates) const {
        std::vector<double> costs(sampler.get_num_samples());
        std::vector<double> collision_costs(sampler.get_num_samples());

        // Rollout for each control sequence
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < sampler.get_num_samples(); i++) {
            // Predict state sequence
            predict_state_seq(sampler.noised_control_seq_samples_[i], global_init_state, &global_state_seq_candidates->at(i),
                              &local_state_seq_candidates->at(i));

            // Calculate cost
            const auto [cost, collision_cost] =
                // state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, reference_path);
                state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, single_goal_);
            costs.at(i) = cost;
            collision_costs.at(i) = collision_cost;
        }
        return std::make_pair(costs, collision_costs);
    }

    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                                    const State& global_init_state,
                                                                                    const grid_map::GridMap& obstacle_map,
                                                                                       const nav_msgs::Path& reference_path,
                                                                                    StateSeqBatch* global_state_seq_candidates,
                                                                                    StateSeqBatch* local_state_seq_candidates) const {
            std::vector<double> costs(sampler.get_num_samples());
            std::vector<double> collision_costs(sampler.get_num_samples());

            // Rollout for each control sequence
    #pragma omp parallel for num_threads(thread_num_)
            for (size_t i = 0; i < sampler.get_num_samples(); i++) {
                // Predict state sequence
                predict_state_seq(sampler.noised_control_seq_samples_[i], global_init_state, &global_state_seq_candidates->at(i),
                                &local_state_seq_candidates->at(i));

                // Calculate cost
                const auto [cost, collision_cost] =
                    state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, reference_path);
                    // state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, single_goal_);
                costs.at(i) = cost;
                collision_costs.at(i) = collision_cost;
            }
            return std::make_pair(costs, collision_costs);
        }

    // Predict both local and global state sequence from control sequence using path
    void MPCBase::predict_state_seq(const ControlSeq& control_seq,
                                    const State& global_init_state,
                                    //const nav_msgs::Path& reference_path,
                                    StateSeq* global_state_seq,
                                    StateSeq* local_state_seq) const {
        // Initialize global state
        global_state_seq->row(0) = global_init_state;

        // Initialize local state
        local_state_seq->row(0) = State::Zero(STATE_SPACE::dim);

        // Extract path points from nav_msgs::Path
        // std::vector<geometry_msgs::PoseStamped> path_poses = path.poses;

        // Assuming that the path is in global frame, we use the first pose as reference
        // const double init_x = global_init_state(STATE_SPACE::x);
        // const double init_y = global_init_state(STATE_SPACE::y);
        // const double init_yaw = global_init_state(STATE_SPACE::yaw);
        
        // Start iterating over the prediction steps
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            // Get control inputs (Vx, Vy, Wz)
            const double v_x = control_seq(i, CONTROL_SPACE::Vx);  // x direction velocity
            const double v_y = control_seq(i, CONTROL_SPACE::Vy);  // y direction velocity
            const double w = control_seq(i, CONTROL_SPACE::Wz);  // angular velocity (yaw rate)

            // Global state update
            const double global_x = global_state_seq->row(i)(STATE_SPACE::x);
            const double global_y = global_state_seq->row(i)(STATE_SPACE::y);
            const double global_yaw = global_state_seq->row(i)(STATE_SPACE::yaw);  // yaw angle

            // Kinematic update using holonomic model
            const double delta_global_x = v_x * cos(global_yaw) * prediction_interval_ - v_y * sin(global_yaw) * prediction_interval_;
            const double delta_global_y = v_x * sin(global_yaw) * prediction_interval_ + v_y * cos(global_yaw) * prediction_interval_;
            const double delta_global_yaw = w * prediction_interval_;  // angular velocity affects yaw

            // Update global state
            global_state_seq->row(i + 1)(STATE_SPACE::x) = global_x + delta_global_x;
            global_state_seq->row(i + 1)(STATE_SPACE::y) = global_y + delta_global_y;
            global_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(global_yaw + delta_global_yaw), cos(global_yaw + delta_global_yaw));

            // Local state update (same logic, different frame)
            const double local_x = local_state_seq->row(i)(STATE_SPACE::x);
            const double local_y = local_state_seq->row(i)(STATE_SPACE::y);
            const double local_yaw = local_state_seq->row(i)(STATE_SPACE::yaw);  // yaw angle

            // Kinematic update using holonomic model
            const double delta_local_x = v_x * cos(local_yaw) * prediction_interval_ - v_y * sin(local_yaw) * prediction_interval_;
            const double delta_local_y = v_x * sin(local_yaw) * prediction_interval_ + v_y * cos(local_yaw) * prediction_interval_;
            const double delta_local_yaw = w * prediction_interval_;  // angular velocity affects yaw

            // Update local state
            local_state_seq->row(i + 1)(STATE_SPACE::x) = local_x + delta_local_x;
            local_state_seq->row(i + 1)(STATE_SPACE::y) = local_y + delta_local_y;
            local_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(local_yaw + delta_local_yaw), cos(local_yaw + delta_local_yaw));
        }
    }


    geometry_msgs::PoseStamped MPCBase::selectTargetReference(const nav_msgs::Path& reference_path, const State& current_state, double lookahead_distance_) const {
    // Step 1: Find the index of the closest point on the reference path.
    int closest_index = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < reference_path.poses.size(); i++) {
        double dx = current_state(STATE_SPACE::x) - reference_path.poses[i].pose.position.x;
        double dy = current_state(STATE_SPACE::y) - reference_path.poses[i].pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_index = i;
        }
    }
    // Step 2: From the closest point, move ahead along the path until the cumulative distance exceeds lookahead_distance.
    double accumulated_distance = 0.0;
    int target_index = closest_index;

    // If the distance to the next waypoint is less than the lookahead_distance, keep iterating
    for (size_t i = closest_index; i < reference_path.poses.size() - 1; i++) {
        double dx = reference_path.poses[i+1].pose.position.x - reference_path.poses[i].pose.position.x;
        double dy = reference_path.poses[i+1].pose.position.y - reference_path.poses[i].pose.position.y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);

        // If the accumulated distance reaches or exceeds the lookahead distance, stop here
        if (accumulated_distance >= lookahead_distance_) {
            target_index = i + 1;
            break;
            }
        }

        // Step 3: Ensure that the target is at least some lookahead_distance away
        if (accumulated_distance < lookahead_distance_) {
            target_index = reference_path.poses.size() - 1;  // Use last pose as target
        }

        // Ensure reference path is not empty and return the target pose
        if (reference_path.poses.empty()) {
            ROS_WARN("Reference path is empty.");
        geometry_msgs::PoseStamped empty_pose;
        return empty_pose;
    }
    return reference_path.poses[target_index];
}


    // calculate state cost using obstacle map and reference map from both global state sequence and local state sequence
    // obstacle cost is calculated from local state sequence
    // reference cost is calculated from global state sequence
    std::pair<double, double> MPCBase::state_cost(const StateSeq& global_state_seq,
                                                  const StateSeq& local_state_seq,
                                                  const grid_map::GridMap& local_obstacle_map,
                                                  const nav_msgs::Path& reference_path) const {
        // calc cost for each state
        double sum_ref_cost = 0.0;
        double sum_collision_cost = 0.0;
        // 경로가 비어있지 않다고 가정 (poses.size() > 0)
        // 만약 pose가 없으면 cost를 어떻게 할지 별도 처리 필요할 수도 있음.
        
        // 새로운 파라미터: lookahead_distance는 클래스 멤버 lookahead_distance_ (YAML로부터 설정됨)
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            const State global_state = global_state_seq.row(i);
            const State local_state  = local_state_seq.row(i);
            
            // (A) Lookahead 방식: 선택한 목표 reference 포인트
            geometry_msgs::PoseStamped target_ref = selectTargetReference(reference_path, global_state, lookahead_distance_);
            double dx = global_state(STATE_SPACE::x) - target_ref.pose.position.x;
            double dy = global_state(STATE_SPACE::y) - target_ref.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            double path_yaw = tf2::getYaw(target_ref.pose.orientation);
            double angle_diff = global_state(STATE_SPACE::yaw) - path_yaw;
            angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
            
            sum_ref_cost += q_dist_ * (dist * dist);
            sum_ref_cost += q_angle_ * (angle_diff * angle_diff);

            // ------------------------------------------------
            // (B) Obstacle collision cost (로컬 상태 기준)
            // ------------------------------------------------
            // collision cost
            // check inside of gridmap
            double collision_cost = 100.0;
            if (local_obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
                collision_cost =
                    local_obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
            }

            sum_collision_cost += collision_cost * collision_weight_;
        }

        // Terminal cost 처리는 기존 방식과 유사하게
        const State global_terminal_state = global_state_seq.row(prediction_step_size_ - 1);
        const State local_terminal_state = local_state_seq.row(prediction_step_size_ - 1);
        double dx = global_terminal_state(STATE_SPACE::x) - reference_path.poses.back().pose.position.x; // 혹은 selectTargetReference 사용
        double dy = global_terminal_state(STATE_SPACE::y) - reference_path.poses.back().pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        sum_ref_cost += q_terminal_dist_ * (dist * dist);

        double yaw = global_terminal_state(STATE_SPACE::yaw);
        double goal_yaw = tf2::getYaw(reference_path.poses.back().pose.orientation);
        double adiff = yaw - goal_yaw;
        adiff = std::atan2(std::sin(adiff), std::cos(adiff));
        sum_ref_cost += q_terminal_angle_ * (adiff * adiff);

        double terminal_collision_cost = 10.0;
        if (local_obstacle_map.isInside(grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)))) {
            terminal_collision_cost = local_obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)));
        }
        sum_collision_cost += terminal_collision_cost * collision_weight_;

        return std::make_pair(sum_ref_cost + sum_collision_cost, sum_collision_cost);
    }



    std::pair<double, double> MPCBase::state_cost(const StateSeq& global_state_seq,
                                                const StateSeq& local_state_seq,
                                                const grid_map::GridMap& local_obstacle_map,
                                                const geometry_msgs::PoseStamped& goal) const {
    double sum_goal_cost = 0.0;
    double sum_collision_cost = 0.0;
    
    // (A) For each prediction step, compute goal-based cost.
    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        const State global_state = global_state_seq.row(i);
        const State local_state  = local_state_seq.row(i);
        
        // Compute distance and angle difference from the goal.
        double dx = global_state(STATE_SPACE::x) - goal.pose.position.x;
        double dy = global_state(STATE_SPACE::y) - goal.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        double goal_yaw = tf2::getYaw(goal.pose.orientation);
        double angle_diff = global_state(STATE_SPACE::yaw) - goal_yaw;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
        
        sum_goal_cost += q_dist_ * (dist * dist);
        sum_goal_cost += q_angle_ * (angle_diff * angle_diff);
        
        // (B) Obstacle collision cost using local state.
        double collision_cost = 100.0;
        if (local_obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
            collision_cost = local_obstacle_map.atPosition(obstacle_layer_name_,
                                    grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
        }
        sum_collision_cost += collision_cost * collision_weight_;

        // collision_cost 로그 출력
        // ROS_INFO_THROTTLE(2.0, "collision_cost: %.2f, sum_collision_cost: %.2f", collision_cost, sum_collision_cost);
    }
    
    // Terminal cost: use the final predicted state versus the goal.
    const State global_terminal_state = global_state_seq.row(prediction_step_size_ - 1);
    const State local_terminal_state = local_state_seq.row(prediction_step_size_ - 1);
    double dx = global_terminal_state(STATE_SPACE::x) - goal.pose.position.x;
    double dy = global_terminal_state(STATE_SPACE::y) - goal.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    sum_goal_cost += q_terminal_dist_ * (dist * dist);
    
    double global_terminal_yaw = global_terminal_state(STATE_SPACE::yaw);
    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    double angle_diff = global_terminal_yaw - goal_yaw;
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
    sum_goal_cost += q_terminal_angle_ * (angle_diff * angle_diff);
    
    double terminal_collision_cost = 100.0;
    if (local_obstacle_map.isInside(grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)))) {
        terminal_collision_cost = local_obstacle_map.atPosition(obstacle_layer_name_,
                                        grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)));
    }
    sum_collision_cost += terminal_collision_cost * collision_weight_;
    
    // collision_cost 로그 출력
    // ROS_INFO_THROTTLE(2.0, "Collision Cost at step %zu: %.2f", prediction_step_size_ - 1, collision_cost);
    // ROS_INFO_THROTTLE(2.0, "terminal_collision_cost: %.2f", terminal_collision_cost);

    return std::make_pair(sum_goal_cost + sum_collision_cost, sum_collision_cost);
}


    std::pair<StateSeq, XYCovMatrices> MPCBase::calc_state_distribution(const StateSeqBatch& state_seq_candidates) const {
        // calc mean state
        StateSeq mean_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
        for (size_t i = 0; i < state_seq_candidates.size(); i++) {
            mean_state_seq += state_seq_candidates[i];
        }
        mean_state_seq /= static_cast<double>(state_seq_candidates.size());

        // calc covariance matrices of x and y
        XYCovMatrices xy_cov_matrices =
            std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(prediction_step_size_, Eigen::MatrixXd::Zero(2, 2));
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < state_seq_candidates.size(); i++) {
            for (size_t j = 0; j < prediction_step_size_; j++) {
                Eigen::VectorXd diff = Eigen::VectorXd::Zero(2);
                diff(0) = state_seq_candidates[i](j, STATE_SPACE::x) - mean_state_seq(j, STATE_SPACE::x);
                diff(1) = state_seq_candidates[i](j, STATE_SPACE::y) - mean_state_seq(j, STATE_SPACE::y);
                xy_cov_matrices[j] += diff * diff.transpose();
            }
        }
        for (size_t j = 0; j < prediction_step_size_; j++) {
            xy_cov_matrices[j] /= static_cast<double>(state_seq_candidates.size());
        }

        return std::make_pair(mean_state_seq, xy_cov_matrices);
    }

}  // namespace cpu
}  // namespace mppi