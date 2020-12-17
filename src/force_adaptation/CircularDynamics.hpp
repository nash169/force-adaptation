#ifndef FORCEADAPTATION_CIRCULARDYNAMICS_HPP
#define FORCEADAPTATION_CIRCULARDYNAMICS_HPP

#include <Eigen/Core>
#include <math.h>

namespace force_adaptation {
    class CircularDynamics {
    public:
        CircularDynamics(const double radius = 1, const Eigen::Vector2d& circle_reference = Eigen::Vector2d::Zero(), const Eigen::Vector3d& plane_reference = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& frame = Eigen::Matrix3d::Identity()) : _radius(radius), _circle_reference(circle_reference), _plane_reference(plane_reference), _frame(frame) {}

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

        Eigen::Vector3d dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            // double r = 0;
            // Eigen::Vector2d planar_vel;
            // Eigen::Vector3d velocity, proj;

            // Eigen::Map<Eigen::MatrixXd> center(_circle_reference.data(), 1, 2);
            // Eigen::VectorXd offset(3);
            // offset = planeEmbedding(center).row(0);

            // // Projection over the plane reference frame
            // proj = _frame.transpose() * (x - offset);

            // // Current rotation radius
            // r = proj.segment(0, 2).norm();

            // // Planar velocity
            // planar_vel(0) = _radius - r;
            // planar_vel(1) = r * M_PI;

            // // Plane distace dependent scaling of planar velocity
            // planar_vel *= exp(-proj(2));

            // velocity.segment(0, 2) = rotation2D(atan2(proj(1), proj(0))) * planar_vel;

            // // Vertical velocity
            // velocity(2) = -proj(2);

            // return _frame * velocity;

            Eigen::Vector3d vd_circular = circularDS(x.head(3));

            Eigen::Vector3d vd_contact;
            Eigen::Vector3d n;
            n << 0.0, 0.0, -1.0;

            vd_contact = (Eigen::Matrix3d::Identity() - n * n.transpose()) * vd_circular;
            vd_contact.normalize();

            float angle = std::acos(n.dot(vd_contact));
            float theta = angle * (1 - std::tanh(40 * std::max(0.0, x(2))));
            Eigen::Vector3d k;
            k = n.cross(vd_contact);

            Eigen::Matrix3d R, K;
            R.setIdentity();
            if (k.norm() > 1e-6) {
                k.normalize();
                K << 0.0f, -k(2), k(1),
                    k(2), 0.0, -k(0),
                    -k(1), k(0), 0.0;
                R = Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
            }

            return R * (0.1f * n);
        }

        Eigen::Vector3d circularDS(const Eigen::Vector3d x)
        {
            Eigen::Vector3d center;
            center << _circle_reference(0), _circle_reference(1), 0.0;

            Eigen::Vector3d xp;
            xp = x - center;

            double r;
            r = xp.segment(0, 2).norm();

            double theta;
            theta = std::atan2(xp(1), xp(0));

            double rd_dot = _radius - r;

            Eigen::Vector3d vd;

            double omega = M_PI;

            vd(0) = (rd_dot * cos(theta)) - (omega * sin(theta) * r);
            vd(1) = (rd_dot * sin(theta)) + (omega * cos(theta) * r);

            vd(2) = -xp(2);

            return vd;
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
