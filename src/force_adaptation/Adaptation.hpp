#ifndef FORCEADAPTATION_ADAPATATION_HPP
#define FORCEADAPTATION_ADAPATATION_HPP

#include <vector>

/* Force Adaptation */
#include "force_adaptation/CircularDynamics.hpp"
#include "force_adaptation/Control.hpp"
#include "force_adaptation/Particle.hpp"

/* Tools */
#include "force_adaptation/tools/math.hpp"

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
            BO_PARAM(bool, optimize_noise, true);
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
    using GP_t = model::GP<Params, Kernel_t, Mean_t, model::gp::KernelLFOpt<Params, opt::Adam<Params>>>;

    class Adaptation {
    public:
        Adaptation(const size_t& storage_dim, const double& activation_time, const double& update_freq, const double& optim_freq)
            : _storage_dim(storage_dim), _activation_time(activation_time), _update_freq(update_freq), _optim_freq(optim_freq)
        {
            _action = Eigen::Vector3d::Zero();
        }

        ~Adaptation() {}

        std::vector<Eigen::VectorXd> samples()
        {
            return _samples;
        }

        void store(const Eigen::VectorXd& sample, const Eigen::VectorXd& observation)
        {
            _samples.push_back(sample);
            _observations.push_back(limbo::tools::make_vector(observation(2)));

            if (_samples.size() > _storage_dim) {
                _samples.erase(_samples.begin(), _samples.begin() + 1);
                _observations.erase(_observations.begin(), _observations.begin() + 1);
            }

            // size_t index_ref;
            // double ref = 0, temp;

            // for (size_t i = 0; i < dim_gpr; i++) {
            //     temp = (samples[i] - x.head(3)).norm();
            //     if (temp > ref) {
            //         ref = temp;
            //         index_ref = i;
            //     }
            // }

            // samples[index_ref] = x.head(3);
            // f_target = particle.dynamics(time, x, u).tail(3) * particle.mass() - u; // circular_motion.frame() *
            // observations[index_ref] = limbo::tools::make_vector(f_target(2));
        }

        Eigen::Vector3d update(const Eigen::VectorXd& x, float time)
        {
            float upd_time = time * _update_freq, opt_time = time * _optim_freq;

            if (time >= _activation_time) {
                if (ceilf(upd_time) == upd_time) {
                    _gpr.compute(_samples, _observations, true);

                    if (ceilf(opt_time) == opt_time || time == _activation_time)
                        _gpr.optimize_hyperparams();
                }

                _action(2) = _gpr.mu(x)[0];
            }

            return _action;
        }

    protected:
        size_t _storage_dim;
        double _update_freq, _optim_freq, _activation_time;

        Eigen::Vector3d _action;

        std::vector<Eigen::VectorXd> _samples;
        std::vector<Eigen::VectorXd> _observations;

        GP_t _gpr;
    };
} // namespace force_adaptation

#endif // FORCEADAPTATION_ADAPATATION_HPP