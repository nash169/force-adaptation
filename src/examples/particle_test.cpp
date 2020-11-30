#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <control_lib/Control.hpp>
#include <integrator_lib/Integrate.hpp>
#include <kernel_lib/Kernel.hpp>
#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;
using namespace integrator_lib;
using namespace control_lib;
using namespace kernel_lib;

// class FeedbackControl : public Control {
// public:
//     FeedbackControl() : Control(ControlMode::OPERATIONSPACE) {}

//     ~FeedbackControl() {}

//     void init() override
//     {
//         // Dimension controller
//         _dim = 3;

//         // Controller gains
//         Eigen::MatrixXd dGains(_dim, _dim);
//         _controller.setGains("d", dGains);
//     }

//     Eigen::VectorXd update(const Eigen::VectorXd& state) override
//     {
//         return _controller.update(state);
//     }

// protected:
//     size_t _dim;

//     controllers::Feedback _controller;
// };

struct Params {
    struct kernel_exp : public defaults::kernel_exp {
        PARAM_VECTOR(double, sigma, 1);
    };
};

int main(int argc, char const* argv[])
{
    // GPR
    using Exp = kernels::Exp<Params>;
    Exp kernel;

    using SumExp = kernel_lib::utils::Expansion<Params, Exp>;

    // Particle
    double mass = 1,
           step = 0.001;
    Particle particle(mass);
    // particle.setIntegrator(std::make_unique<integrator::ForwardEuler>(step));

    // Dynamics
    CircularDynamics circular_motion;

    // Control
    Eigen::MatrixXd dGains = Eigen::MatrixXd::Identity(3, 3) * 5;
    controllers::Feedback feedback(ControlSpace::LINEAR, 3, 3, step);
    feedback.setGains("d", dGains);

    // Simulation
    double time = 0, max_time = 30;
    size_t num_steps = std::ceil(max_time / step) + 1, index = 0, dim = 3;

    Eigen::VectorXd x(2 * dim), ref = Eigen::VectorXd::Zero(2 * dim), u = Eigen::VectorXd::Zero(dim), log_t(num_steps);
    Eigen::MatrixXd log_x(num_steps, 2 * dim), log_u(num_steps, dim);
    x << 0, 0, 10, 0, 0, 0;

    particle.setState(x).setInput(u);

    log_t(index) = time;
    log_u.row(index) = u;
    log_x.row(index) = x;

    while (time < max_time && index < num_steps) {
        // Action
        ref.tail(3) = circular_motion.dynamics(time, x.head(3), u);

        feedback.setReference(ref);
        // u = feedback.update(x);
        u = (ref.tail(3) - x.tail(3)) * 10;

        // std::cout << u.transpose() << std::endl;

        // Step
        particle.setInput(u);
        particle.update(time);

        time += step;
        index++;

        log_t(index) = time;
        log_x.row(index) = particle.state();
        log_u.row(index) = u;
    }

    // Write
    utils_cpp::FileManager io_manager;

    io_manager.setFile("rsc/data_particle.csv");
    io_manager.write("time", log_t);
    io_manager.append("state", log_x);
    io_manager.append("action", log_u);
}