#pragma once

#include <string>
#include <nav_msgs/Path.h>

namespace mppi {
namespace STATE_SPACE {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int yaw = 2;

    static constexpr int dim = 3;
};  // namespace STATE_SPACE

namespace CONTROL_SPACE {
    // static constexpr int steer = 0;
    static constexpr int Vx = 0;
    static constexpr int Vy = 1;
    static constexpr int Wz = 2;

    static constexpr int dim = 3;
};  // namespace CONTROL_SPACE

struct Params {
    struct Common {
        int thread_num;
        int prediction_step_size;
        double prediction_interval;
        bool is_reference_path_less_mode;
        double lookahead_distance;
        double max_Vx;
        double min_Vx;
        double max_Vy;
        double min_Vy;
        double max_Wz;
        double min_Wz;
        double q_dist;
        double q_angle;
        double collision_weight;
        double q_terminal_dist;
        double q_terminal_angle;
    };
    Common common;

    struct ForwardMPPI {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double Vx_cov;
        double Vy_cov;
        double Wz_cov;
        
        int sample_num_for_grad_estimation;
        double Vx_cov_for_grad_estimation;
        double Vy_cov_for_grad_estimation;
        double Wz_cov_for_grad_estimation;
        int num_itr_for_grad_estimation;
        double step_size_for_grad_estimation;
    };
    ForwardMPPI forward_mppi;

    struct ReverseMPPI {
        int sample_batch_num;
        double negative_ratio;
        bool is_sample_rejection;
        double sample_inflation_ratio;
        double warm_start_ratio;
        int iteration_num;
        double step_size;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double Vx_cov;
        double Vy_cov;
        double Wz_cov;
    };
    ReverseMPPI reverse_mppi;

    struct SteinVariationalMPC {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double Vx_cov;
        double Vy_cov;
        double Wz_cov;
        int num_svgd_iteration;
        int sample_num_for_grad_estimation;
        double Vx_cov_for_grad_estimation;
        double Vy_cov_for_grad_estimation;
        double Wz_cov_for_grad_estimation;
        double svgd_step_size;
        bool is_max_posterior_estimation;
    };
    SteinVariationalMPC stein_variational_mpc;

    struct SVGuidedMPPI {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double Vx_cov;
        double Vy_cov;
        double Wz_cov;
        int guide_sample_num;
        double grad_lambda;
        int sample_num_for_grad_estimation;
        double Vx_cov_for_grad_estimation;
        double Vy_cov_for_grad_estimation;
        double Wz_cov_for_grad_estimation;
        double svgd_step_size;
        int num_svgd_iteration;
        bool is_use_nominal_solution;
        bool is_covariance_adaptation;
        double gaussian_fitting_lambda;
        double min_Vx_cov;
        double max_Vx_cov;
        double min_Vy_cov;
        double max_Vy_cov;
        double min_Wz_cov;
        double max_Wz_cov;
    };
    SVGuidedMPPI svg_mppi;
};

namespace cpu {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
    using StateSeq = Eigen::MatrixXd;
    using ControlSeq = Eigen::MatrixXd;
    using StateSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ControlSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ControlSeqCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using XYCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
}  // namespace cpu

}  // namespace mppi
