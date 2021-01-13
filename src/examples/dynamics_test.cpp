#include <force_adaptation/ForceAdaptation.hpp>

// Dependencies
#include <integrator_lib/Integrate.hpp>
#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;
using namespace integrator_lib;

struct IntegratorParams {
    struct integrator : public integrator_lib::defaults::integrator {
        PARAM_SCALAR(double, step, 0.001);
    };
};

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
    frame = Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::MatrixXd X(resolution, resolution), Y(resolution, resolution), plane_points(resolution * resolution, 2);
    X = Eigen::RowVectorXd::LinSpaced(resolution, -length / 2, length / 2).replicate(resolution, 1);
    Y = Eigen::VectorXd::LinSpaced(resolution, -width / 2, width / 2).replicate(1, resolution);
    plane_points.col(0) = X.reshaped();
    plane_points.col(1) = Y.reshaped();

    // Particle & Load distribution
    Particle<> my_particle;

    Eigen::MatrixXd surface_force(resolution * resolution, 3);
    Eigen::MatrixXd particle_pos(resolution * resolution, 3);
    particle_pos << plane_points, Eigen::VectorXd::Zero(resolution * resolution);
    surface_force.block(0, 0, surface_force.rows(), 2) = plane_points;
    surface_force.col(2) = my_particle.surfaceForce(particle_pos);

    // Dynamics and embeddings
    CircularDynamics my_ds(radius, circle_reference, plane_reference, frame);

    Eigen::MatrixXd plane_embedding(resolution * resolution, 3), surface_embedding(resolution * resolution, 3), circle_embedding(resolution, 3);
    plane_embedding = my_ds.planeEmbedding(plane_points);
    surface_embedding = my_ds.surfaceEmbedding(plane_points);
    circle_embedding = my_ds.circleEmbedding(angles);

    // // Integration
    // double time = 0, max_time = 20, step = 0.001;
    // size_t num_steps = std::ceil(max_time / step) + 1, index = 0, dim = 3;

    // Eigen::VectorXd x(dim), u = Eigen::VectorXd::Zero(1), log_t(num_steps);
    // Eigen::MatrixXd log_x(num_steps, dim);

    // x << 5, 5, 10;

    // integrator::ForwardEuler<IntegratorParams> myInt;

    // log_t(index) = time;
    // log_x.row(index) = x;

    // while (time < max_time && index < num_steps) {
    //     x = myInt.integrate(my_ds, &CircularDynamics::dynamics, time, x, u);

    //     time += step;
    //     index++;

    //     log_t(index) = time;
    //     log_x.row(index) = x;
    // }

    // Write
    utils_cpp::FileManager io_manager;

    io_manager.setFile("rsc/data_dynamics.csv");
    io_manager.write("plane", plane_embedding);
    io_manager.append("surface", surface_embedding);
    io_manager.append("circle", circle_embedding);
    io_manager.append("force", surface_force);
    // io_manager.append("dynamics", log_x);

    return 0;
}
