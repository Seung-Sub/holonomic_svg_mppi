#include "mppi_controller/stein_variational_guided_mppi.hpp"
#include <ros/ros.h>
#include <fstream>

namespace mppi {
namespace cpu {
    SVGuidedMPPI::SVGuidedMPPI(const Params::Common& common_params, const Params::SVGuidedMPPI& svg_mppi_params)
        : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
          thread_num_(common_params.thread_num),
          lambda_(svg_mppi_params.lambda),
          alpha_(svg_mppi_params.alpha),
          non_biased_sampling_rate_(svg_mppi_params.non_biased_sampling_rate),
          Vx_cov_(svg_mppi_params.Vx_cov),
          Vy_cov_(svg_mppi_params.Vy_cov),
          Wz_cov_(svg_mppi_params.Wz_cov),
          sample_num_for_grad_estimation_(svg_mppi_params.sample_num_for_grad_estimation),
          grad_lambda_(svg_mppi_params.grad_lambda),
          Vx_cov_for_grad_estimation_(svg_mppi_params.Vx_cov_for_grad_estimation),
          Vy_cov_for_grad_estimation_(svg_mppi_params.Vy_cov_for_grad_estimation),
          Wz_cov_for_grad_estimation_(svg_mppi_params.Wz_cov_for_grad_estimation),
          svgd_step_size_(svg_mppi_params.svgd_step_size),
          num_svgd_iteration_(svg_mppi_params.num_svgd_iteration),
          is_use_nominal_solution_(svg_mppi_params.is_use_nominal_solution),
          is_covariance_adaptation_(svg_mppi_params.is_covariance_adaptation),
          gaussian_fitting_lambda_(svg_mppi_params.gaussian_fitting_lambda),
          min_Vx_cov_(svg_mppi_params.min_Vx_cov),
          max_Vx_cov_(svg_mppi_params.max_Vx_cov),
          min_Vy_cov_(svg_mppi_params.min_Vy_cov),
          max_Vy_cov_(svg_mppi_params.max_Vy_cov),
          min_Wz_cov_(svg_mppi_params.min_Wz_cov),
          max_Wz_cov_(svg_mppi_params.max_Wz_cov) {
        const size_t sample_batch_num = static_cast<size_t>(svg_mppi_params.sample_batch_num);
        const size_t guide_sample_num = static_cast<size_t>(svg_mppi_params.guide_sample_num);
        const size_t sample_num_for_grad_estimation = static_cast<size_t>(svg_mppi_params.sample_num_for_grad_estimation);

        const size_t sample_num_for_cache = std::max(std::max(sample_batch_num, sample_num_for_grad_estimation), guide_sample_num);
        mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_num_for_cache);

        const double max_Vx = common_params.max_Vx;
        const double min_Vx = common_params.min_Vx;
        const double max_Vy = common_params.max_Vy;
        const double min_Vy = common_params.min_Vy;
        const double max_Wz = common_params.max_Wz;
        const double min_Wz = common_params.min_Wz;

        std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_Vx, max_Vy, max_Wz};
        std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_Vx, min_Vy, min_Wz};
        prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);

        guide_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(guide_sample_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);

        prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
        nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();

        // initialize prior distribution
        const ControlSeqCovMatrices control_seq_cov_matrices = guide_samples_ptr_->get_constant_control_seq_cov_matrices({Vx_cov_, Vy_cov_, Wz_cov_});
        guide_samples_ptr_->random_sampling(guide_samples_ptr_->get_zero_control_seq(), control_seq_cov_matrices);

        // initialize grad samplers
        for (size_t i = 0; i < sample_batch_num; i++) {
            grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(sample_num_for_grad_estimation_, prediction_step_size_,
                                                                                    max_control_inputs, min_control_inputs, non_biased_sampling_rate_,
                                                                                    thread_num_, i));
        }
    }


    std::pair<ControlSeq, double> SVGuidedMPPI::solve(const State& initial_state) {
        // For Debug (1)
        // {
        //     const ControlSeqCovMatrices guide_cov = guide_samples_ptr_->get_constant_control_seq_cov_matrices({Vx_cov_, Vy_cov_, Wz_cov_});
        //     guide_samples_ptr_->random_sampling(guide_samples_ptr_->get_zero_control_seq(), guide_cov);
        // }
        
        // 파일 전역 or 클래스 멤버 (예시 편의상 전역)
        static std::ofstream g_cost_log_file;
        static bool g_is_cost_log_file_open = false;

        if (!g_is_cost_log_file_open) {
        g_cost_log_file.open("/tmp/svgd_cost_log.csv");
        // 헤더
        g_cost_log_file << "Iteration,SampleIdx,Cost\n";
        g_is_cost_log_file_open = true;
    }

        // 1) 로그 파일 준비 (static 정적 변수 이용)
        static std::ofstream g_best_particle_log;
        static bool g_best_particle_log_opened = false;

        if (!g_best_particle_log_opened) {
            g_best_particle_log.open("/tmp/svgd_best_particle_log.csv");
            // 헤더: Iteration, Row, Col, Value
            g_best_particle_log << "Iteration,Row,Col,Value\n";
            g_best_particle_log_opened = true;
        }

        static std::ofstream g_adaptive_cov_log;
        static bool g_adaptive_cov_log_opened = false;
        if (!g_adaptive_cov_log_opened) {
            g_adaptive_cov_log.open("/tmp/svgd_adaptive_cov_log.csv");
            // 헤더: Iteration, TimeStep, Dim, CovValue
            g_adaptive_cov_log << "Iteration,TimeStep,Dim,CovValue\n";
            g_adaptive_cov_log_opened = true;
        }

        // SVGD 루프에서 iteration을 세고 싶으면 static 카운터 사용
        static int iter_count = 0;


        // solve()가 여러 번 호출될 때, SVGD 루프(num_svgd_iteration_)을 돌며
        // 그 총합을 세고 싶다면, static 변수 이용
        static int iteration_count = 0; // SVGD 외부 루프 인덱스

        // Transport guide particles by SVGD
        std::vector<double> costs_history;
        std::vector<ControlSeq> control_seq_history;
        auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) { return mpc_base_ptr_->calc_sample_costs(sampler, initial_state).first; };
        for (int i = 0; i < num_svgd_iteration_; i++) {
            // Transport samples by stein variational gradient descent
            const ControlSeqBatch grad_log_posterior = approx_grad_posterior_batch(*guide_samples_ptr_, func_calc_costs);

            

#pragma omp parallel for num_threads(thread_num_)
            for (size_t i = 0; i < guide_samples_ptr_->get_num_samples(); i++) {
                guide_samples_ptr_->noised_control_seq_samples_[i] += svgd_step_size_ * grad_log_posterior[i];
            }

            // store costs and samples for adaptive covariance calculation
            const std::vector<double> costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
            // guide_samples_ptr_->costs_ = costs;
            // const std::vector<double> cost_with_control_term = guide_samples_ptr_->get_costs_with_control_term(gaussian_fitting_lambda, 0,
            // prior_samples_ptr_->get_zero_control_seq());

            // 로그에 기록
            for (size_t s = 0; s < costs.size(); s++) {
                g_cost_log_file << iteration_count << "," << s << "," << costs[s] << "\n";
            }

            iteration_count++;

            // history 업데이트 for adaptive covariance
            costs_history.insert(costs_history.end(), costs.begin(), costs.end());
            control_seq_history.insert(control_seq_history.end(), guide_samples_ptr_->noised_control_seq_samples_.begin(),
                                       guide_samples_ptr_->noised_control_seq_samples_.end());
        }
        // SVGD 종료 후 베스트 파티클 추출
        const auto guide_costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
        const size_t min_idx = std::distance(guide_costs.begin(), std::min_element(guide_costs.begin(), guide_costs.end()));
        const ControlSeq best_particle = guide_samples_ptr_->noised_control_seq_samples_[min_idx];

        // calculate adaptive covariance matrices for prior distribution
        // TODO: Support multiple control input dimensions
        ControlSeqCovMatrices covs = prior_samples_ptr_->get_constant_control_seq_cov_matrices({Vx_cov_, Vy_cov_, Wz_cov_});
        if (is_covariance_adaptation_) {
            // calculate softmax costs
            const std::vector<double> softmax_costs = softmax(costs_history, gaussian_fitting_lambda_, thread_num_);


            // min/max cov arrays for each dimension
            std::array<double, CONTROL_SPACE::dim> min_cov = {min_Vx_cov_, min_Vy_cov_, min_Wz_cov_};
            std::array<double, CONTROL_SPACE::dim> max_cov = {max_Vx_cov_, max_Vy_cov_, max_Wz_cov_};

            // (예시) 차원별 1D fitting
            // time: i in [0..T-1), dimension: d in [0..CONTROL_SPACE::dim)
            // control_seq_history: vector<ControlSeq> (size = costs_history.size()?), each ControlSeq is (T-1 x 3).
            //  => total #samples = control_seq_history.size()
            for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
                // covs[i]를 Identity로 초기화
                covs[i] = Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim);

                for (size_t d = 0; d < CONTROL_SPACE::dim; d++) {
                    // 각 dimension d에 대한 sample & weight
                    std::vector<double> oneD_samples(control_seq_history.size());
                    std::vector<double> q_star(control_seq_history.size());

                    for (size_t idx = 0; idx < control_seq_history.size(); idx++) {
                        // control_seq_history[idx](i, d) => time=i, dimension=d
                        oneD_samples[idx] = control_seq_history[idx](i, d);
                        q_star[idx] = softmax_costs[idx];
                    }

                    // 1D Gaussian fitting
                    const double sigma = gaussian_fitting(oneD_samples, q_star).second;
                    // clamp with per-dim min/max
                    const double sigma_clamped = std::clamp(sigma, min_cov[d], max_cov[d]);

                        // 대각성분에만 sigma_clamped 반영 (나머지 0)
                        covs[i](d, d) = sigma_clamped;
                }
            }
        }

        // iteration_count를 solve()마다 증가시키려면 여기서++ 
        iter_count++; 

        for (size_t row = 0; row < (prediction_step_size_ - 1); row++) {
            for (size_t col = 0; col < CONTROL_SPACE::dim; col++) {
                double val = best_particle(row, col);
                g_best_particle_log 
                    << iter_count << ","
                    << row << ","
                    << col << ","
                    << val 
                    << "\n";
            }
        }

        // (B) covs CSV 저장
        // covs도 (prediction_step_size_-1)개의 행렬, each is CONTROL_SPACE::dim x CONTROL_SPACE::dim
        // 만약 diagonal만 사용할 경우:
        for (size_t t = 0; t < (prediction_step_size_ - 1); t++) {
            for (size_t dim = 0; dim < CONTROL_SPACE::dim; dim++) {
                double cov_val = covs[t](dim, dim); // diagonal
                g_adaptive_cov_log 
                    << iter_count << ","
                    << t << "," 
                    << dim << "," 
                    << cov_val 
                    << "\n";
            }
        }

        // random sampling from prior distribution
        prior_samples_ptr_->random_sampling(prev_control_seq_, covs);

        // Rollout samples and calculate costs
        auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
        prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);

        // calculate weights
        if (is_use_nominal_solution_) {
            // with nominal sequence
            nominal_control_seq_ = best_particle;
        } else {
            // without nominal sequence
            nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
        }
        // mppi 가중치 계산
        const std::vector<double> weights = calc_weights(*prior_samples_ptr_, nominal_control_seq_);
        weights_ = weights;  // for visualization

        // Get control input sequence by weighted average of samples
        ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
        for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
            updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
        }

        const int collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
        const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

        // update previous control sequence for next time step
        prev_control_seq_ = updated_control_seq;

        // For Debug (2)
        // {
        //     guide_samples_ptr_->random_sampling(nominal_control_seq_, covs);
        // }
        

        return std::make_pair(updated_control_seq, collision_rate);
    }

    void SVGuidedMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };
    // void SVGuidedMPPI::set_obstacle_map(const nav_msgs::OccupancyGrid& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };

    void SVGuidedMPPI::set_reference_path(const nav_msgs::Path& reference_path) { mpc_base_ptr_->set_reference_path(reference_path); };

    void SVGuidedMPPI::set_single_goal(const geometry_msgs::PoseStamped& goal) { mpc_base_ptr_->set_single_goal(goal); };

    std::pair<std::vector<StateSeq>, std::vector<double>> SVGuidedMPPI::get_state_seq_candidates(const int& num_samples) const {
        return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
    }

    std::tuple<StateSeq, double, double, double> SVGuidedMPPI::get_predictive_seq(const State& initial_state,
                                                                                  const ControlSeq& control_input_seq) const {
        const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
        double input_error = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            input_error += (control_input_seq.row(i)).norm();
        }
        return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
    }

    ControlSeqCovMatrices SVGuidedMPPI::get_cov_matrices() const { return prior_samples_ptr_->get_cov_matrices(); }

    ControlSeq SVGuidedMPPI::get_control_seq() const { return nominal_control_seq_; }

    std::pair<StateSeq, XYCovMatrices> SVGuidedMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }

    // == private functions ==
    ControlSeq SVGuidedMPPI::approx_grad_log_likelihood(const ControlSeq& mean_seq,
                                                        const ControlSeq& noised_seq,
                                                        const ControlSeqCovMatrices& inv_covs,
                                                        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                                        PriorSamplesWithCosts* sampler) const {
        const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({Vx_cov_for_grad_estimation_, Vy_cov_for_grad_estimation_, Wz_cov_for_grad_estimation_});

        // generate gaussian random samples, center of which is input_seq
        sampler->random_sampling(noised_seq, grad_cov);

        // calculate forward simulation and costs
        sampler->costs_ = calc_costs(*sampler);

        // calculate cost with control term
        // const std::vector<double> costs_with_control_term = sampler->get_costs_with_control_term(grad_lambda_, 0.0,
        // sampler->get_zero_control_seq());
        std::vector<double> exp_costs(sampler->get_num_samples());
        ControlSeq sum_of_grads = mean_seq * 0.0;
        const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < sampler->get_num_samples(); i++) {
            double cost_with_control_term = sampler->costs_[i];
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                // const double control_term = grad_lambda_ * prev_control_seq_.row(j) * inv_covs[j] *
                // sampler->noised_control_seq_samples_[i].row(j).transpose();
                const double diff_control_term = grad_lambda_ * (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)) *
                                                 inv_covs[j] *
                                                 (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)).transpose();
                // cost_with_control_term += control_term + diff_control_term;
                cost_with_control_term += diff_control_term;
            }
            const double exp_cost = std::exp(-cost_with_control_term / grad_lambda_);
            exp_costs[i] = exp_cost;

            ControlSeq grad_log_gaussian = mean_seq * 0.0;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
            }
            sum_of_grads += grad_log_gaussian;
        }
        const double sum_of_costs = std::accumulate(exp_costs.begin(), exp_costs.end(), 0.0);
        return sum_of_grads / (sum_of_costs + 1e-10);
    }


    ControlSeqBatch SVGuidedMPPI::approx_grad_posterior_batch(
        const PriorSamplesWithCosts& samples,
        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {

        // 파일 전역 또는 클래스 멤버로 관리(여기선 간단히 전역 정적 변수처럼 처리 예시)
        static std::ofstream g_gradient_log_file;
        static bool g_is_gradient_log_file_open = false;

        // 로그 파일이 아직 열려있지 않다면, 이 지점에서 엽니다. (한 번만)
        if (!g_is_gradient_log_file_open) {
            g_gradient_log_file.open("/tmp/svgd_gradient_log.csv"); 
            // 헤더 작성 (iteration, sample_idx, row, col, value 등 필요시)
            g_gradient_log_file << "Iteration,SampleIdx,Row,Col,GradValue\n";
            g_is_gradient_log_file_open = true;
        }

        static int iteration_count = 0; // 함수 호출 횟수를 세어 Iteration 지표로 활용

        ControlSeqBatch grad_log_likelihoods = samples.get_zero_control_seq_batch();
        const ControlSeq mean = samples.get_mean();
        // #pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            const ControlSeq grad_log_likelihood = approx_grad_log_likelihood(
                mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, grad_sampler_ptrs_.at(i).get());

            // 여기에서 grad_log_likelihood를 확인하기 위해 로그를 출력
            // ROS_INFO_STREAM("grad_log_likelihood sample " << i << ":\n" << grad_log_likelihood);
            grad_log_likelihoods[i] = grad_log_likelihood;

        // == (A) 여기서 각 row, col에 대해 값을 로그로 저장 ==
        for (size_t row = 0; row < grad_log_likelihood.rows(); row++) {
            for (size_t col = 0; col < grad_log_likelihood.cols(); col++) {
                double val = grad_log_likelihood(row, col);
                g_gradient_log_file 
                    << iteration_count << ","    // Iteration
                    << i << ","          // SampleIdx (guide idx)
                    << row << ","                // Row (time step)
                    << col << ","                // Col (control dimension)
                    << val                       // GradValue
                    << "\n";
            }
        }
        }
        iteration_count++;
        return grad_log_likelihoods;
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
                                                                                       const std::vector<double>& weights) const {
        ControlSeq mean = samples.get_zero_control_seq();
        ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();

        const ControlSeq prior_mean = samples.get_mean();
        const ControlSeqCovMatrices prior_inv_covs = samples.get_inv_cov_matrices();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            mean += weights[i] * samples.noised_control_seq_samples_[i];

            const ControlSeq diff = samples.noised_control_seq_samples_[i] - prior_mean;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                sigma[j] += weights[i] * diff.row(j).transpose() * prior_inv_covs[j] * diff.row(j);
            }
        }

        return std::make_pair(mean, sigma);
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
        ControlSeq mu = samples.get_zero_control_seq();
        ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();

// calculate mean
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            mu += samples.noised_control_seq_samples_[i];
        }
        mu /= static_cast<double>(samples.get_num_samples());

#pragma omp parallel for num_threads(thread_num_)
        // calculate covariance matrices
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
                            (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
            }
        }

        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            sigma[j] /= static_cast<double>(samples.get_num_samples());

            // add small value to avoid singular matrix
            sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
        }

        return std::make_pair(mu, sigma);
    }

    std::vector<double> SVGuidedMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs,
                                                   const ControlSeq& nominal_control_seq) const {
        // calculate costs with control term
        const std::vector<double> costs_with_control_term =
            prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, nominal_control_seq);

        // softmax weights
        return softmax(costs_with_control_term, lambda_, thread_num_);
    }

    // Gao's Algorithm
    // H. GUO, “A Simple Algorithm for Fitting a Gaussian Function,” IEEE Signal Process, Mag.28, No. 5 (2011), 134–137.
    std::pair<double, double> SVGuidedMPPI::gaussian_fitting(const std::vector<double>& x, const std::vector<double>& y) const {
        assert(x.size() == y.size());

        // Should y is larger than 0 for log function
        std::vector<double> y_hat(y.size(), 0.0);
        std::transform(y.begin(), y.end(), y_hat.begin(), [](double y) { return std::max(y, 1e-10); });

        // const double epsilon = 1e-8;
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < x.size(); i++) {
            const double y_hat_2 = y_hat[i] * y_hat[i];
            const double y_hat_log = std::log(y_hat[i]);

            A(0, 0) += y_hat_2;
            A(0, 1) += y_hat_2 * x[i];
            A(0, 2) += y_hat_2 * x[i] * x[i];

            A(1, 0) += y_hat_2 * x[i];
            A(1, 1) += y_hat_2 * x[i] * x[i];
            A(1, 2) += y_hat_2 * x[i] * x[i] * x[i];

            A(2, 0) += y_hat_2 * x[i] * x[i];
            A(2, 1) += y_hat_2 * x[i] * x[i] * x[i];
            A(2, 2) += y_hat_2 * x[i] * x[i] * x[i] * x[i];

            b(0) += y_hat_2 * y_hat_log;
            b(1) += y_hat_2 * x[i] * y_hat_log;
            b(2) += y_hat_2 * x[i] * x[i] * y_hat_log;
        }

        // solve Au = b
        const Eigen::Vector3d u = A.colPivHouseholderQr().solve(b);

        // calculate mean and variance

        // original
        // const double mean = -u(1) / (2.0 * u(2));
        // const double variance = std::sqrt(-1.0 / (2.0 * u(2)));

        // To avoid nan;
        const double eps = 1e-5;
        const double mean = -u(1) / (2.0 * std::min(u(2), -eps));
        // const double variance = std::sqrt(1.0 / (2.0 * std::abs(std::min(u(2), -eps))));
        const double variance = std::sqrt(1.0 / (2.0 * std::abs(u(2))));

        return std::make_pair(mean, variance);
    }

}  // namespace cpu
}  // namespace mppi
