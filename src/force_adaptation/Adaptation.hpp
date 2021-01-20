#ifndef FORCEADAPTATION_ADAPATATION_HPP
#define FORCEADAPTATION_ADAPATATION_HPP

#include <future>
#include <shared_mutex>
#include <thread>
#include <vector>

/* Limbo */
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/model/gp/kernel_loo_opt.hpp>
#include <limbo/opt/adam.hpp>
#include <limbo/tools/math.hpp>
#include <limbo/tools/random_generator.hpp>

namespace force_adaptation {
    using namespace limbo;

    struct Params {
        struct kernel : public limbo::defaults::kernel {
            // BO_PARAM(double, noise, 0.0);
            BO_PARAM(bool, optimize_noise, false);
        };
        struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
        };
        struct opt_rprop : public limbo::defaults::opt_rprop {
        };

        struct opt_adam : public limbo::defaults::opt_adam {
        };
    };

    using Kernel_t = kernel::SquaredExpARD<Params>;
    using Mean_t = mean::NullFunction<Params>;
    using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params, opt::Rprop<Params>>>;

    class Adaptation {
    public:
        Adaptation(const size_t& storage_dim = 100, const double& storage_spacing = 0.001, const double& activation_time = 0, const double& update_freq = 1, const double& optim_freq = 1)
            : _storage_dim(storage_dim), _storage_spacing(storage_spacing), _activation_time(activation_time), _update_freq(update_freq), _optim_freq(optim_freq), _threads(2)
        {
            _active = false;
            _update_time = 0;
            _optim_time = 0;
            _action = Eigen::Vector3d::Zero();
        }

        ~Adaptation() {}

        std::vector<Eigen::VectorXd> samples() const
        {
            return _samples;
        }

        std::vector<Eigen::VectorXd> observations() const
        {
            return _observations;
        }

        Adaptation& store(const Eigen::VectorXd& sample, const Eigen::VectorXd& observation, bool noise = false)
        {
            if (_samples.size() == _storage_dim) {
                nearestNeighborhoods(sample, observation, noise);
            }
            else if (_samples.empty() || (_samples.back() - sample).norm() >= _storage_spacing) {
                std::unique_lock<std::shared_mutex> guard(_shared_mutex);

                _samples.push_back(sample);
                _observations.push_back(limbo::tools::make_vector(observation(2)) + (noise ? limbo::tools::random_vector(1) : limbo::tools::make_vector(0.0)));
            }

            return *(this);
        }

        Adaptation& update()
        {
            _threads[0] = std::thread(&Adaptation::thread_update, this);

            return *(this);
        }

        Adaptation& optimize()
        {
            _threads[0].join();
            _threads[1] = std::thread(&Adaptation::thread_optimize, this);

            return *(this);
        }

        Eigen::Vector3d action(const Eigen::VectorXd& x, const double& time)
        {
            _time = time;

            if (_active) {
                _action(2) = _gpr.mu(x)[0];
            }

            return _action;
        }

    protected:
        bool _active;
        size_t _storage_dim;
        double _storage_spacing, _update_freq, _optim_freq, _activation_time,
            _time, _update_time, _optim_time;

        Eigen::Vector3d _action;

        std::vector<Eigen::VectorXd> _samples;
        std::vector<Eigen::VectorXd> _observations;

        GP_t _gpr;

        std::vector<std::thread> _threads;
        std::shared_mutex _shared_mutex;

        void thread_update()
        {
            if ((_time - _update_time) >= 1 / _update_freq && _time >= _activation_time) {
                std::shared_lock<std::shared_mutex> guard(_shared_mutex);
                _gpr.compute(_samples, _observations, true);
                _update_time = _time;
            }
        }

        void thread_optimize()
        {
            if ((_time - _optim_time) >= 1 / _optim_freq && _time >= _activation_time) {
                _gpr.optimize_hyperparams();
                _optim_time = _time;
                _active = true;
            }
        }

        void movingWindow(const Eigen::VectorXd& sample, const Eigen::VectorXd& observation, bool noise)
        {
            if ((_samples.back() - sample).norm() >= _storage_spacing) {
                std::unique_lock<std::shared_mutex> guard(_shared_mutex);

                _samples.push_back(sample);
                _observations.push_back(limbo::tools::make_vector(observation(2)) + (noise ? limbo::tools::random_vector(1) : limbo::tools::make_vector(0.0)));

                _samples.erase(_samples.begin(), _samples.begin() + 1);
                _observations.erase(_observations.begin(), _observations.begin() + 1);
            }
        }

        void nearestNeighborhoods(const Eigen::VectorXd& sample, const Eigen::VectorXd& observation, bool noise)
        {
            bool add_point = true;
            size_t index_ref;
            double ref = 0, temp;

            for (size_t i = 0; i < _storage_dim; i++) {
                temp = (_samples[i] - sample).norm();

                if (temp < _storage_spacing) {
                    add_point = false;
                    break;
                }

                if (temp > ref) {
                    ref = temp;
                    index_ref = i;
                }
            }

            if (add_point) {
                std::unique_lock<std::shared_mutex> guard(_shared_mutex);

                _samples[index_ref] = sample;
                _observations[index_ref] = limbo::tools::make_vector(observation(2)) + (noise ? limbo::tools::random_vector(1) : limbo::tools::make_vector(0.0));
            }
        }
    };
} // namespace force_adaptation

#endif // FORCEADAPTATION_ADAPATATION_HPP