#include <iostream>

#include <Eigen/Geometry>

#include <force_adaptation/CircularDynamics.hpp>
#include <force_adaptation/Particle.hpp>

#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;

int main(int argc, char const* argv[])
{
    size_t resolution = 100;
    
    // Circle
    double radius = 1;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();

    Eigen::VectorXd angles(resolution);
    angles = Eigen::VectorXd::LinSpaced(resolution, 0, 2 * M_PI);

    
    // Plane
    double length = 10, width = 5;
    Eigen::Vector3d plane_reference = Eigen::Vector3d::Zero();
    Eigen::Matrix3d frame;
    frame = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(0.5*M_PI,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.33*M_PI, Eigen::Vector3d::UnitZ());

    Eigen::MatrixXd X(resolution, resolution), Y(resolution, resolution), plane_points(resolution * resolution, 2);
    X = Eigen::RowVectorXd::LinSpaced(resolution, -length / 2, length / 2).replicate(resolution, 1);
    Y = Eigen::VectorXd::LinSpaced(resolution, -width / 2, width / 2).replicate(1, resolution);
    plane_points.col(0) = X.reshaped();
    plane_points.col(1) = Y.reshaped();
    
    
    // Particle & Load distribution
    Particle my_particle;

    Eigen::VectorXd surface_force(resolution*resolution, 3);
    Eigen::MatrixXd particle_pos(resolution*resolution, 3);
    particle_pos << plane_points, Eigen::VectorXd::Zero(resolution*resolution);
    surface_force.block(0,0, surface_force.rows(), 2) = plane_points;
    surface_force.col(2) = my_particle.surfaceForce(particle_pos);
    
    
    // Dynamics and embeddings
    CircularDynamics my_ds;

    Eigen::MatrixXd plane_embedding(resolution * resolution, 3), circle_embedding(resolution, 3);
    plane_embedding = my_ds.planeEmbedding(plane_points);
    circle_embedding = my_ds.circleEmbedding(angles);
    
    
    // Write
    utils_cpp::FileManager io_manager;

    io_manager.setFile("rsc/data.csv");
    io_manager.write("plane", plane_embedding);
    io_manager.append("circle", circle_embedding);
    io_manager.append("force", surface_force);

    return 0;
}
