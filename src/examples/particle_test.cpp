#include <vector>

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

struct KernelParams {
    struct kernel : public kernel_lib::defaults::kernel {
        PARAM_SCALAR(double, sigma_n, 0.0);
        PARAM_SCALAR(double, sigma_f, 1.0);
    };

    struct rbf : public kernel_lib::defaults::rbf {
        PARAM_VECTOR(double, sigma, 1);
    };
};

struct IntegratorParams {
    struct integrator : public integrator_lib::defaults::integrator {
        PARAM_SCALAR(double, step, 0.001);
    };
};

template <typename T>
std::vector<T> slice(std::vector<T> const& v, int m, int n)
{
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n + 1;

    std::vector<T> vec(first, last);
    return vec;
}

Eigen::MatrixXd generate_dataset(size_t dim, const std::vector<Eigen::Vector3d>& input)
{
    Eigen::MatrixXd output(input.size(), 3);

    for (size_t i = 0; i < dim; i++)
        output.row(i) = input.at(i); // input(i);

    return output;
}

Eigen::VectorXd generate_target(size_t dim, const std::vector<double>& input)
{
    Eigen::VectorXd output(input.size());

    for (size_t i = 0; i < dim; i++)
        output(i) = input.at(i); // input(i);

    return output;
}

int main(int argc, char const* argv[])
{
    size_t resolution = 100;

    // Circle
    double radius = 1;
    Eigen::Vector2d circle_reference;
    circle_reference << 2, 0;
    Eigen::VectorXd angles(resolution);
    angles = Eigen::VectorXd::LinSpaced(resolution, 0, 2 * M_PI);

    // Plane
    double length = 10, width = 5;
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
    CircularDynamics circular_motion;
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
    size_t dim_gpr = 100, update_freq = 100;
    double force_reference = 10;

    Eigen::MatrixXd store(dim_gpr, 3);
    Eigen::VectorXd target(dim_gpr), weights(dim_gpr);
    SumRbf psi;

    // Simulation
    double time = 0, max_time = 30;
    size_t num_steps = std::ceil(max_time / step) + 1, index = 0, dim = 3;

    Eigen::Vector3d u = Eigen::Vector3d::Zero(), f_adapt = Eigen::Vector3d::Zero();
    Eigen::VectorXd x(2 * dim), ref = Eigen::VectorXd::Zero(2 * dim), log_t(num_steps);
    Eigen::MatrixXd log_x(num_steps, 2 * dim), log_u(num_steps, dim);
    x << 0, 0, 10, 0, 0, 0;

    particle.setState(x).setInput(u);

    log_t(index) = time;
    log_u.row(index) = u;
    log_x.row(index) = x;

    // store.push_back(x.head(3));
    // target.push_back(force_reference);
    store.row(index) = x.head(3);
    target(index) = force_reference;

    while (time < max_time && index < num_steps) {
        // Control force for circular motion
        ref.tail(3) = circular_motion.dynamics(time, x.head(3), u);
        feedback.setReference(ref);
        u = feedback.update(x);

        // Adaptation (activate after 100 steps)
        if (index >= 99) {
            if (!((index + 1) % update_freq)) {
                weights = psi.kernel()(store, store).colPivHouseholderQr().solve(target);
                psi.setReference(store).setWeights(weights);
            }
            f_adapt(2) = psi(x.head(3).transpose())(0);
        }

        // Step
        particle.setInput(u + frame * f_adapt);
        particle.update(time);
        x = particle.state();

        // Increment counters
        time += step;
        index++;

        // Store & order points
        if (index < dim_gpr) {
            store.row(index) = x.head(3);
            target(index) = frame.transpose().row(2) * particle.surfaceForce(Eigen::VectorXd(x.head(3))) - force_reference;
        }
        else {
            size_t index_ref;
            double ref, temp;

            for (size_t i = 0; i < dim_gpr; i++) {
                temp = (store.row(i) - x.head(3).transpose()).norm();
                if (temp > ref) {
                    ref = temp;
                    index_ref = i;
                }
            }

            store.row(index_ref) = x.head(3);
            target(index_ref) = frame.transpose().row(2) * particle.surfaceForce(Eigen::VectorXd(x.head(3))) - force_reference;
        }

        // Record
        log_t(index) = time;
        log_x.row(index) = x;
        log_u.row(index) = u;
    }

    // Write
    utils_cpp::FileManager io_manager;

    io_manager.setFile("rsc/data_particle.csv");
    io_manager.write("time", log_t, "state", log_x, "action", log_u, "plane", plane_embedding, "circle", circle_embedding);
}