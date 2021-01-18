#include <vector>

#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <control_lib/Control.hpp>
#include <integrator_lib/Integrate.hpp>
#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;
using namespace integrator_lib;
using namespace control_lib;
using namespace limbo;

struct IntegratorParams {
    struct integrator : public integrator_lib::defaults::integrator {
        PARAM_SCALAR(double, step, 0.001);
    };
};

int main(int argc, char const* argv[])
{
    size_t resolution = 100;

    // Circle
    double radius = 0.05;
    Eigen::Vector2d circle_reference;
    circle_reference << 0, 0;
    Eigen::VectorXd angles(resolution);
    angles = Eigen::VectorXd::LinSpaced(resolution, 0, 2 * M_PI);

    // Plane
    double length = 1, width = 0.5;
    Eigen::Vector3d plane_reference;
    plane_reference << 1, 2, -3;
    Eigen::Matrix3d frame;
    frame = Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::MatrixXd X(resolution, resolution), Y(resolution, resolution), plane_points(resolution * resolution, 2);
    X = Eigen::RowVectorXd::LinSpaced(resolution, -length / 2, length / 2).replicate(resolution, 1);
    Y = Eigen::VectorXd::LinSpaced(resolution, -width / 2, width / 2).replicate(1, resolution);
    plane_points.col(0) = X.reshaped();
    plane_points.col(1) = Y.reshaped();

    // Dynamics and embeddings
    CircularDynamics circular_motion(radius); // CircularDynamics circular_motion(radius, circle_reference, plane_reference, frame);
    Eigen::MatrixXd plane_embedding(resolution * resolution, 3), circle_embedding(resolution, 3);
    plane_embedding = circular_motion.planeEmbedding(plane_points);
    circle_embedding = circular_motion.circleEmbedding(angles);

    // Control
    double ctr_freq = 200;
    Eigen::MatrixXd dGains = Eigen::MatrixXd::Identity(3, 3) * 100;
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    controllers::Feedback feedback(ControlSpace::LINEAR, 3, 3, 1 / ctr_freq);
    feedback.setGains("d", dGains);

    // Adaptation
    size_t storage_dim = 100;
    double update_freq = 500, optim_freq = 0.01, activation_time = 2;
    Eigen::VectorXd gpr_target(3), f_adapt = Eigen::VectorXd::Zero(3);
    Adaptation adapt(storage_dim, activation_time, update_freq, optim_freq);

    // Particle
    size_t dim = 3;
    double mass = 1, load = 10;
    Particle<integrator::ForwardEuler<IntegratorParams>> particle(mass, load);
    Eigen::VectorXd x(2 * dim), ref = Eigen::VectorXd::Zero(2 * dim);
    x << 0.1, 0, 0.2, 0, 0, 0;
    particle.setState(x).setInput(u + f_adapt);

    // Simulation
    double time = 0, max_time = 60, step = 0.001;
    size_t num_steps = std::ceil(max_time / step) + 2, index = 0;

    Eigen::VectorXd log_t(num_steps), log_adaptation(num_steps);
    Eigen::MatrixXd log_x(num_steps, 2 * dim), log_force(num_steps, dim), log_u(num_steps, dim);

    log_t(index) = time;
    log_u.row(index) = u;
    log_x.row(index) = x;
    log_force.row(index) = particle.surfaceForce(x.head(3)); // circular_motion.frame() * particle.dynamics(time, x, u).tail(3) * particle.mass();
    log_adaptation(index) = f_adapt(2);

    while (time < max_time && index < num_steps) {
        // Control force for circular motion
        ref.tail(3) = circular_motion.dynamics(time, x.head(3), u);
        feedback.setReference(ref);
        u = feedback.update(x);

        // Adaptation (activate after 100 steps)
        // std::cout << time << std::endl;
        f_adapt = adapt.update(x.head(3), time);
        gpr_target = particle.dynamics(time, x, u + f_adapt).tail(3) * particle.mass() - u;
        adapt.store(x.head(3), gpr_target, false);

        // Step
        particle.setInput(u + f_adapt); // circular_motion.frame() *
        particle.update(time);
        x = particle.state();

        // Increment counters
        time += step;
        index++;

        // Record
        log_t(index) = time;
        log_x.row(index) = x;
        log_u.row(index) = u;
        log_force.row(index) = particle.surfaceForce(x.head(3));
        log_adaptation(index) = f_adapt(2);
    }

    // Write
    utils_cpp::FileManager io_manager;

    io_manager.setFile("rsc/data_learn.csv");
    io_manager.write("time", log_t, "state", log_x, "action", log_u, "force_measured", log_force, "force_adaptation", log_adaptation, "plane", plane_embedding, "circle", circle_embedding);

    Eigen::MatrixXd scatter(storage_dim, 3);

    for (size_t i = 0; i < storage_dim; i++)
        scatter.row(i) = adapt.samples().at(i);

    io_manager.append("scatter", scatter);

    return 0;
}