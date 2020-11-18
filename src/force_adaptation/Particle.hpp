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

        Eigen::VectorXd surfaceForce(const Eigen::MatrixXd& x, const Eigen::VectorXd& distance)
        {
            Eigen::VectorXd surface_force(x.rows());

            surface_force = (x.col(1).array()*x.col(0).array().sin() - x.col(0).array()*x.col(1).array().cos())*distance.array();
            
            return surface_force;
        }

    private:
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP