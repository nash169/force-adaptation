#ifndef FORCEADAPTATION_CIRCULARDYNAMICS_HPP
#define FORCEADAPTATION_CIRCULARDYNAMICS_HPP

#include <Eigen/Core>
#include <math.h>

namespace force_adaptation {
    class CircularDynamics {
    public:
        CircularDynamics(const double radius = 1, const Eigen::Vector2d& circle_reference = Eigen::Vector2d::Zero(), const Eigen::Vector3d& plane_reference = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& frame = Eigen::Matrix3d::Identity())
            : _radius(radius), _circle_reference(circle_reference), _plane_reference(plane_reference), _frame(frame) {}

        ~CircularDynamics() {}

        /* Getter method */

        Eigen::Matrix3d frame() const
        {
            return _frame;
        }

        /* Setter method */

        CircularDynamics& setRadius(const double radius)
        {
            _radius = radius;

            return *this;
        }

        CircularDynamics& setCircleReference(const Eigen::Vector2d& circle_reference)
        {
            _circle_reference = circle_reference;

            return *this;
        }

        CircularDynamics& setPlaneReference(const Eigen::Vector3d& plane_reference)
        {
            _plane_reference = plane_reference;

            return *this;
        }

        CircularDynamics& setFrame(const Eigen::Matrix3d& frame)
        {
            _frame = frame;

            return *this;
        }

        CircularDynamics& setStep(const double step)
        {
            _step = step;

            return *this;
        }

        /* Dynamics */

        Eigen::Vector3d dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
        {
            double r = 0;
            Eigen::Vector2d planar_vel;
            Eigen::Vector3d velocity, proj;

            // Eigen::Map<Eigen::MatrixXd> center(_circle_reference.data(), 1, 2);
            Eigen::VectorXd offset(3);
            offset = planeEmbedding(_circle_reference.transpose()).row(0);

            // Projection over the plane reference frame
            proj = _frame.transpose() * (x - offset);

            // Current rotation radius
            r = proj.segment(0, 2).norm();

            // Planar velocity
            planar_vel(0) = _radius - r;
            planar_vel(1) = r * M_PI;

            // Plane distace dependent scaling of planar velocity
            planar_vel *= exp(-proj(2) * 50);

            velocity.segment(0, 2) = rotation2D(atan2(proj(1), proj(0))) * planar_vel;

            // Vertical velocity
            velocity(2) = -proj(2);

            return _frame * velocity;
        }

        /* Embeddings */

        Eigen::MatrixXd circleEmbedding(const Eigen::VectorXd& x) const
        {
            Eigen::MatrixXd circle_points(x.rows(), 2), circle_embedding(x.rows(), 3);

            circle_points = circle(x);
            circle_embedding = planeEmbedding(circle_points);

            return circle_embedding;
        }

        Eigen::MatrixXd planeEmbedding(const Eigen::MatrixXd& x) const
        {
            Eigen::MatrixXd plane(x.rows(), 3);

            for (size_t i = 0; i < x.rows(); i++)
                plane.row(i) = _plane_reference + _frame.col(0) * x(i, 0) + _frame.col(1) * x(i, 1);

            return plane;
        }

        Eigen::MatrixXd surfaceEmbedding(const Eigen::MatrixXd& x) const
        {
            Eigen::MatrixXd surface(x.rows(), 3);

            surface << x, (x.col(1).array() * x.col(0).array().sin() - x.col(0).array() * x.col(1).array().cos()) * 0.5;
            surface = (_frame * surface.transpose()).transpose();
            surface.rowwise() += _plane_reference.transpose();

            return surface;
        }

    protected:
        double _radius, _step;
        Eigen::Vector2d _circle_reference;
        Eigen::Vector3d _plane_reference;
        Eigen::Matrix3d _frame;

        Eigen::MatrixXd circle(const Eigen::VectorXd& angle) const
        {
            Eigen::MatrixXd x(angle.rows(), 2);

            x.col(0) = _circle_reference(0) + _radius * angle.array().cos();
            x.col(1) = _circle_reference(1) + _radius * angle.array().sin();

            return x;
        }

        Eigen::Matrix2d rotation2D(const double angle) const
        {
            Eigen::Matrix2d rotation;

            rotation << cos(angle), -sin(angle),
                sin(angle), cos(angle);

            return rotation;
        }
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_CIRCULARDYNAMICS_HPP
