#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <control_lib/Control.hpp>
#include <integrator_lib/Integrate.hpp>
#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;
using namespace integrator_lib;
using namespace control_lib;

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

int main(int argc, char const* argv[])
{
    // Particle
    double mass = 1, step = 0.001;
    Particle particle(mass);
    particle.setIntegrator(std::make_unique<integrator::ForwardEuler>(step));

    // Dynamics
    double radius = 1;
    Eigen::Vector2d circle_reference;
    circle_reference << 2, 0;
    Eigen::Vector3d plane_reference;
    plane_reference << 1, 2, -3;
    Eigen::Matrix3d frame;
    frame = Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitZ());
    CircularDynamics circular_motion(radius, circle_reference, plane_reference, frame);

    // Control
    Eigen::MatrixXd dGains = Eigen::MatrixXd::Identity(3,3);
    controllers::Feedback feedback(ControlSpace::LINEAR, 3, 3, step);
    feedback.setGains("d", dGains);

    // Simulation
    double time = 0, max_time = 20;
    size_t num_steps = std::ceil(max_time / step) + 1, index = 0, dim = 3;

    Eigen::VectorXd x(2*dim), ref = Eigen::VectorXd::Zero(2*dim), u = Eigen::VectorXd::Zero(dim), log_t(num_steps);
    Eigen::MatrixXd log_x(num_steps, 2*dim), log_u(num_steps, dim);
    x << 5, 5, 10, 0, 0, 0;

    particle.setState(x).setInput(u);

    log_t(index) = time;
    log_u.row(index) = u;
    log_x.row(index) = x;

    Eigen::VectorXd test1(3), test2(6);
    test1 << 1,2,3;
    test2.tail(3) = test1;
    // while (time < max_time && index < num_steps) {
    //     // Action
    //     ref.segment(2,3) = circular_motion.dynamics(time, x.segment(0,3), u);
    //     feedback.setReference(ref);
    //     u = feedback.update(x);

    //     // Step
    //     particle.setInput(u);
    //     particle.update(time);
    //     // x = particle.state();

    //     time += step;
    //     index++;

    //     // log_t(index) = time;
    //     // log_x.row(index) = x;
    //     // log_u.row(index) = u;
    // }
}