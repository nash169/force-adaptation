#include <fstream>
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools/math.hpp>
#include <limbo/tools/random_generator.hpp>

#include <limbo/serialize/text_archive.hpp>

using namespace limbo;

struct Params {
    struct kernel_exp {
        BO_PARAM(double, sigma_sq, 1.0);
        BO_PARAM(double, l, 0.2);
    };
    struct kernel : public defaults::kernel {
        BO_PARAM(bool, optimize_noise, true);
    };
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public defaults::opt_rprop {
    };
};

int main(int argc, char** argv)
{

    // our data (1-D inputs, 1-D outputs)
    std::vector<Eigen::VectorXd> samples;
    std::vector<Eigen::VectorXd> observations;

    size_t N = 8;
    for (size_t i = 0; i < N; i++) {
        Eigen::VectorXd s = tools::random_vector(1).array() * 4.0 - 2.0;
        samples.push_back(s);
        observations.push_back(tools::make_vector(std::cos(s(0))));
    }

    // the type of the GP
    using Kernel_t = kernel::Exp<Params>;
    using Mean_t = mean::Data<Params>;
    using GP_t = model::GP<Params, Kernel_t, Mean_t>;

    // 1-D inputs, 1-D outputs
    GP_t gp(1, 1);

    // compute the GP
    gp.compute(samples, observations);

    // an alternative is to optimize the hyper-parameters
    // in that case, we need a kernel with hyper-parameters that are designed to be optimized
    using Kernel2_t = kernel::SquaredExpARD<Params>;
    using Mean_t = mean::Data<Params>;
    using GP2_t = model::GP<Params, Kernel2_t, Mean_t, model::gp::KernelLFOpt<Params>>;

    GP2_t gp_ard(1, 1);
    // do not forget to call the optimization!
    gp_ard.compute(samples, observations, false);

    std::cout << gp_ard.kernel_function().h_params().transpose() << std::endl;

    gp_ard.optimize_hyperparams();

    std::cout << gp_ard.kernel_function().h_params().transpose() << std::endl;

    return 0;
}