#ifndef FORCEADAPTATION_CIRCULARDYNAMICS_HPP
#define FORCEADAPTATION_CIRCULARDYNAMICS_HPP

#include <Eigen/Core>
#include <math.h>

namespace force_adaptation {
    class CircularDynamics {
    public:
        CircularDynamics(const double radius = 1, const Eigen::Vector2d& circle_reference = Eigen::Vector2d::Zero(), const Eigen::Vector3d& plane_reference = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& frame = Eigen::Matrix3d::Identity(), const double step = 0.01) : _radius(radius), _circle_reference(circle_reference), _plane_reference(plane_reference), _frame(frame), _step(step) {}

        ~CircularDynamics() {}

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

        Eigen::MatrixXd velocity(const Eigen::MatrixXd& x)
        {
            double r = 0;
            Eigen::Vector3d proj;
            Eigen::Vector2d planar_vel;

            for (size_t i = 0; i < x.rows(); i++){
                proj = _frame.transpose() * (x.row(i).transpose() - _plane_reference);
                r = proj.segment(0,1).norm();
                planar_vel(0) = _radius - r; planar_vel(1) = r*M_PI;
                _velocity.row(i).segment(0,1) = rotation2D(atan2(proj(1), proj(2))).transpose()*planar_vel;
                _velocity(i,2) = proj(2);
            }

            return _velocity;
        }

        Eigen::MatrixXd position(const Eigen::MatrixXd& x)
        {
            _position = _position + _step * _velocity;

            return _position;
        }

        Eigen::MatrixXd circleEmbedding(const Eigen::VectorXd& x)
        {
            Eigen::MatrixXd circle_points(x.rows(), 2), circle_embedding(x.rows(), 3);

            circle_points = circle(x);
            circle_embedding = planeEmbedding(circle_points);

            return circle_embedding;
        }

        Eigen::MatrixXd planeEmbedding(const Eigen::MatrixXd& x)
        {
            Eigen::MatrixXd plane(x.rows(), 3);

            for (size_t i = 0; i < x.rows(); i++)
                plane.row(i) = _plane_reference + _frame.col(0) * x(i, 0) + _frame.col(1) * x(i, 1);

            return plane;
        }

    protected:
        double _radius, _step;
        Eigen::Vector2d _circle_reference;
        Eigen::Vector3d _plane_reference;
        Eigen::Matrix3d _frame;

        Eigen::MatrixXd _position, _velocity;

        Eigen::MatrixXd circle(const Eigen::VectorXd& angle)
        {
            Eigen::MatrixXd x(angle.rows(), 2);

            x.col(0) = _circle_reference(0) + _radius * angle.array().cos();
            x.col(1) = _circle_reference(1) + _radius * angle.array().sin();

            return x;
        }

        Eigen::Matrix2d rotation2D(const double angle)
        {
            Eigen::Matrix2d rotation;
            
            rotation << cos(angle), -sin(angle),
                sin(angle), cos(angle);

            return rotation;
        }
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_CIRCULARDYNAMICS_HPP
