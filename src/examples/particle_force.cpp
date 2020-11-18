#include <iostream>

#include <Eigen/Geometry>

#include <force_adaptation/Dynamics.hpp>

#include <utils_cpp/UtilsCpp.hpp>

using namespace force_adaptation;

int main(int argc, char const* argv[])
{
    utils_cpp::FileManager io_manager;

    size_t resolution = 100;
    Dynamics my_ds;

    // Rotation
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(0.5*M_PI,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.33*M_PI, Eigen::Vector3d::UnitZ());

    // Plane
    double length = 10, width = 5;
    Eigen::Vector3d reference = Eigen::Vector3d::Zero();
    Eigen::MatrixXd base(3, 2);
    base << 1, 0,
        0, 1,
        0, 0;
    Eigen::MatrixXd X(resolution, resolution), Y(resolution, resolution), plane_points(resolution * resolution, 2), plane_embedding(resolution * resolution, 3);
    X = Eigen::RowVectorXd::LinSpaced(resolution, -length / 2, length / 2).replicate(resolution, 1);
    Y = Eigen::VectorXd::LinSpaced(resolution, -width / 2, width / 2).replicate(1, resolution);
    plane_points.col(0) = X.reshaped();
    plane_points.col(1) = Y.reshaped();
    plane_embedding = my_ds.planeEmbedding(plane_points, reference, m);

    // Circle
    double radius = 1;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::VectorXd angles(resolution);
    Eigen::MatrixXd circle_points(resolution, 2), circle_embedding(resolution, 3);
    angles = Eigen::VectorXd::LinSpaced(resolution, 0, 2 * M_PI);
    circle_points = my_ds.circle(angles, radius, center);
    circle_embedding = my_ds.planeEmbedding(circle_points, reference, m);

    // Save matrices
    Eigen::MatrixXd plane(resolution * resolution, 5), circle(resolution, 6);
    plane << plane_points, plane_embedding;
    circle << angles, circle_points, circle_embedding;

    // Write
    io_manager.setFile("rsc/data.csv");
    io_manager.write("plane", plane);
    io_manager.append("circle", circle);

    return 0;
}
