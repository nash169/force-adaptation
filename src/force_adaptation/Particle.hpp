#ifndef FORCEADAPTATION_PARTICLE_HPP
#define FORCEADAPTATION_PARTICLE_HPP

#include <Eigen/Core>

namespace force_adaptation {
    class Particle {
    public:
        Particle()
        {
        }

        ~Particle()
        {
        }

        Eigen::MatrixXd surfaceForce(const Eigen::MatrixXd& x, const Eigen::VectorXd& distance)
        {
            double a = 1000, b = 100000, c = 400000;
            Eigen::Vector3d surface_force(x.rows());

            surface_force = (-a - b*x.col(0).array().pow(2) - c*x.col(1).array().pow(2))*distance.array();
            
            return surface_force;
        }

    private:
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP