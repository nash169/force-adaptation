#ifndef FORCEADAPTATION_DYNAMICS_HPP
#define FORCEADAPTATION_DYNAMICS_HPP

#include <Eigen/Core>

namespace force_adaptation {
    class Dynamics {
    public:
        Dynamics()
        {
        }

        ~Dynamics()
        {
        }

        void circularMotion()
        {
        }

        Eigen::MatrixXd circle(const Eigen::VectorXd& angle, double radius, const Eigen::Vector2d& center)
        {
            Eigen::MatrixXd x(angle.rows(), 2);

            x.col(0) = radius * angle.array().cos();
            x.col(1) = radius * angle.array().sin();

            return x;
        }

        Eigen::MatrixXd planeEmbedding(const Eigen::MatrixXd& x, const Eigen::Vector3d& reference, const Eigen::MatrixXd& base)
        {
            Eigen::MatrixXd plane(x.rows(), 3);

            for (size_t i = 0; i < x.rows(); i++)
                plane.row(i) = reference + base.col(0) * x(i, 0) + base.col(1) * x(i, 1);

            return plane;
        }

    private:
        Eigen::Vector3d _velocity, _position;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_DYNAMICS_HPP
