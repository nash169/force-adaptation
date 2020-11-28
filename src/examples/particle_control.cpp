#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <control_lib/Control.hpp>
#include <integrator_lib/Integrate.hpp>
#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;
using namespace integrator_lib;
using namespace control_lib;

class FeedbackControl : public Control {
public:
    FeedbackControl() : Control(ControlMode::OPERATIONSPACE) {}

    ~FeedbackControl() {}

    void init() override
    {
        // Dimension controller
        _dim = 3;

        // Controller gains
        Eigen::MatrixXd dGains(_dim, _dim);
        _controller.setGains("d", dGains);
    }

    Eigen::VectorXd update(const Eigen::VectorXd& state) override
    {
        return _controller.update(state);
    }

protected:
    size_t _dim;

    controllers::Feedback _controller;
};

int main(int argc, char const* argv[])
{
    // Integrator
    double step = 0.001;
    integrator::ForwardEuler integrator(step);

    // Particle
    double mass = 1;
    Particle particle(mass);
    particle.setIntegrator(integrator);

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
    CircularDynamics dynamics(radius, circle_reference, plane_reference, frame);

    // Control
    FeedbackControl feedback;

    // // Simulation
    // double time = 0, max_time = 20;
    // size_t num_steps = std::ceil(max_time / step) + 1, index = 0, dim = 3;

    // Eigen::VectorXd x(dim), u = Eigen::VectorXd::Zero(1), log_t(num_steps), log_u(num_steps);
    // Eigen::MatrixXd log_x(num_steps, dim);
    // x << 5, 5, 10;

    // log_t(index) = time;
    // log_u(index) = u(0);
    // log_x.row(index) = x;

    // while (time < max_time && index < num_steps) {
    //     // Action
    //     u = feedback.control();
    //     x = myInt.integrate(particle, &Particle::dynamics, time, x, u);

    //     time += step;
    //     index++;

    //     log_t(index) = time;
    //     log_x.row(index) = x;
    // }
}