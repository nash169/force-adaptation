#ifndef FORCEADAPTATION_PARTICLE_HPP
#define FORCEADAPTATION_PARTICLE_HPP

#include <Eigen/Core>

namespace force_adaptation {
    class Particle {
    public:
        Particle(const double mass = 1) : _mass(mass) {}

        ~Particle() {}

        Eigen::VectorXd dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            return (u + surfaceForce(x))/_mass;
        }

        Eigen::VectorXd surfaceForce(const Eigen::MatrixXd& x)
        {
            Eigen::VectorXd surface_force(x.rows()), negative_altitude(x.rows());
            negative_altitude = -x.col(2);

            surface_force = (x.col(1).array()*x.col(0).array().sin() - x.col(0).array()*x.col(1).array().cos())*negative_altitude.array().exp();
            
            return surface_force;
        }

    protected:
        double _mass;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP