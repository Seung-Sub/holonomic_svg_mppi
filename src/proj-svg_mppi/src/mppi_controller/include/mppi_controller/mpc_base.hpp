// Kohei Honda, 2023

#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <array>
#include <grid_map_core/GridMap.hpp>
#include <memory>
#include <utility>

#include "mppi_controller/common.hpp"
#include "mppi_controller/prior_samples_with_costs.hpp"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h> 

namespace mppi {
namespace cpu {
    class MPCBase {
    public:
        MPCBase(const Params::Common& params, const size_t& sample_num);
        ~MPCBase(){};

        void set_obstacle_map(const grid_map::GridMap& obstacle_map);

        void set_reference_path(const nav_msgs::Path& reference_path);

        // 새로 추가: 단일 goal 설정
        void set_single_goal(const geometry_msgs::PoseStamped& goal);

        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state);

        std::tuple<StateSeq, double, double> get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const;

        std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples,
                                                                                       const std::vector<double>& weights) const;

        std::pair<StateSeq, XYCovMatrices> get_proposed_distribution() const;

    private:
        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                              const State& global_init_state,
                                                                              const grid_map::GridMap& obstacle_map,
                                                                              const geometry_msgs::PoseStamped& goal,
                                                                              StateSeqBatch* global_state_seq_candidates,
                                                                              StateSeqBatch* local_state_seq_candidates) const;
    
        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                              const State& global_init_state,
                                                                              const grid_map::GridMap& obstacle_map,
                                                                              const nav_msgs::Path& reference_path,
                                                                              StateSeqBatch* global_state_seq_candidates,
                                                                              StateSeqBatch* local_state_seq_candidates) const;

        StateSeq predict_state_seq(const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& obstacle_map) const;
        void predict_state_seq(const ControlSeq& control_seq,
                               const State& global_init_state,
                               StateSeq* global_state_seq,
                               StateSeq* local_state_seq) const;

        geometry_msgs::PoseStamped selectTargetReference(const nav_msgs::Path& reference_path, const State& current_state, double lookahead_distance_) const;

        std::pair<double, double> state_cost(const StateSeq& global_state_seq,
                                             const StateSeq& local_base_state_seq,
                                             const grid_map::GridMap& obstacle_map,
                                             const nav_msgs::Path& reference_path) const;

        std::pair<double, double> state_cost(const StateSeq& global_state_seq,
                                             const StateSeq& local_base_state_seq,
                                             const grid_map::GridMap& obstacle_map,
                                             const geometry_msgs::PoseStamped& goal) const;

        std::pair<StateSeq, XYCovMatrices> calc_state_distribution(const StateSeqBatch& state_seq_candidates) const;

    private:
        // == Constant parameters ==
        const std::string obstacle_layer_name_ = "collision_layer";
        

        const bool is_reference_path_less_mode_;
        const int thread_num_;  //!< @brief number of thread for parallel computation
        const size_t prediction_step_size_;
        const double prediction_interval_;  //!< @brief prediction interval [s]
        const double lookahead_distance_;
        const double q_dist_;
        const double q_angle_;
        
        const double collision_weight_;
        const double q_terminal_dist_;
        const double q_terminal_angle_;
        
        bool has_single_goal_ = false;

        // == Inner-variables ==
        grid_map::GridMap obstacle_map_;
        nav_msgs::Path reference_path_;
        geometry_msgs::PoseStamped single_goal_;
        // To reduce memory allocation and visualize candidates paths
        StateSeqBatch global_state_seq_candidates_ = {};
        StateSeqBatch local_state_seq_candidates_ = {};
    };

}  // namespace cpu
}  // namespace mppi
