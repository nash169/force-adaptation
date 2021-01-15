#include <vector>

#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <control_lib/Control.hpp>
#include <integrator_lib/Integrate.hpp>
#include <utils_cpp/UtilsCpp.hpp>

// Limbo
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools/math.hpp>
#include <limbo/tools/random_generator.hpp>

using namespace force_adaptation;
using namespace integrator_lib;
using namespace control_lib;
using namespace limbo;

struct GPRParams {
    struct kernel : public limbo::defaults::kernel {
        BO_PARAM(bool, optimize_noise, false);
    };
    struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public limbo::defaults::opt_rprop {
    };
};

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
    CircularDynamics circular_motion(radius);
    // CircularDynamics circular_motion(radius, circle_reference, plane_reference, frame);
    Eigen::MatrixXd plane_embedding(resolution * resolution, 3), circle_embedding(resolution, 3);
    plane_embedding = circular_motion.planeEmbedding(plane_points);
    circle_embedding = circular_motion.circleEmbedding(angles);

    // Particle
    double mass = 1, step = 0.001;
    Particle<integrator::ForwardEuler<IntegratorParams>> particle(mass);

    // Control
    Eigen::MatrixXd dGains = Eigen::MatrixXd::Identity(3, 3) * 100;
    controllers::Feedback feedback(ControlSpace::LINEAR, 3, 3, step);
    feedback.setGains("d", dGains);

    // GPR
    size_t dim_gpr = 100, update_freq = 100, optim_freq = 10000, activation_time = 1999;
    double force_reference = -10;
    Eigen::VectorXd f_desired(3), f_target(3);
    f_desired << 0, 0, force_reference;
    f_desired = circular_motion.frame() * f_desired;

    std::vector<Eigen::VectorXd> samples(dim_gpr);
    std::vector<Eigen::VectorXd> observations(dim_gpr);

    using Kernel_t = kernel::SquaredExpARD<GPRParams>;
    using Mean_t = mean::NullFunction<GPRParams>;
    using GP_t = model::GP<GPRParams, Kernel_t, Mean_t, model::gp::KernelLFOpt<GPRParams>>;

    GP_t gp_ard(1, 1);

    // Simulation
    double time = 0, max_time = 60;
    size_t num_steps = std::ceil(max_time / step) + 2, index = 0, dim = 3;

    Eigen::Vector3d u = Eigen::Vector3d::Zero(), f_adapt = Eigen::Vector3d::Zero(), f_normal;
    Eigen::VectorXd x(2 * dim), ref(2 * dim), log_t(num_steps), log_adaptation(num_steps);
    Eigen::MatrixXd log_x(num_steps, 2 * dim), log_force(num_steps, dim), log_u(num_steps, dim);
    x << 0.1, 0, 0.2, 0, 0, 0;
    f_normal << 0, 0, 1;

    particle.setState(x).setInput(u);

    log_t(index) = time;
    log_u.row(index) = u;
    log_x.row(index) = x;
    log_force.row(index) = particle.surfaceForce(x.head(3)); // circular_motion.frame() * particle.dynamics(time, x, u).tail(3) * particle.mass();
    log_adaptation(index) = f_adapt(2);

    samples[index] = x.head(3);

    f_target = circular_motion.frame() * particle.dynamics(time, x, u).tail(3) * particle.mass();
    observations[index] = limbo::tools::make_vector(f_target(2));

    while (time < max_time && index < num_steps) {
        // Control force for circular motion
        ref.tail(3) = circular_motion.dynamics(time, x.head(3), u);
        feedback.setReference(ref);
        u = feedback.update(x);

        // Adaptation (activate after 100 steps)
        if (index >= activation_time) {
            if (!((index + 1) % update_freq)) {
                gp_ard.compute(samples, observations, false);

                if (!((index + 1) % optim_freq) || index == activation_time)
                    gp_ard.optimize_hyperparams();
            }

            f_adapt(2) = gp_ard.mu(x.head(3))(0);
        }

        // Step
        particle.setInput(u + circular_motion.frame() * f_adapt + f_desired);
        particle.update(time);
        x = particle.state();

        // Increment counters
        time += step;
        index++;

        // Store & order points
        if (index < dim_gpr) {
            samples[index] = x.head(3);
            f_target = circular_motion.frame() * particle.dynamics(time, x, u).tail(3) * particle.mass(); //circular_motion.frame() * (-f_desired - particle.surfaceForce(x.head(3)));
            observations[index] = limbo::tools::make_vector(f_target(2));
        }
        else {
            size_t index_ref;
            double ref = 0, temp;

            for (size_t i = 0; i < dim_gpr; i++) {
                temp = (samples[i] - x.head(3)).norm();
                if (temp > ref) {
                    ref = temp;
                    index_ref = i;
                }
            }

            samples[index_ref] = x.head(3);
            f_target = circular_motion.frame() * particle.dynamics(time, x, u).tail(3) * particle.mass(); //circular_motion.frame() * (-f_desired - particle.surfaceForce(x.head(3))) + f_normal * gp_ard.mu(x.head(3))(0);
            observations[index_ref] = limbo::tools::make_vector(f_target(2));
        }

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

    Eigen::MatrixXd scatter(samples.size(), 3);
    for (size_t i = 0; i < samples.size(); i++)
        scatter.row(i) = samples[i];

    io_manager.append("scatter", scatter);

    std::cout << gp_ard.kernel_function().h_params().transpose() << std::endl;
    return 0;
}