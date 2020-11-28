#ifndef FORCEADAPTATION_PARTICLE_HPP
#define FORCEADAPTATION_PARTICLE_HPP

#include <Eigen/Core>
#include <integrator_lib/Integrate.hpp>

namespace force_adaptation {
    using namespace integrator_lib;

    class Particle {
    public:
        Particle(const double mass = 1) : _mass(mass) {}

        ~Particle() {}

        void update(const double& time)
        {
            _state = _integrator.integrate(*this, &Particle::dynamics, time, _state, _input);
        }

        Particle& setMass(const double& mass)
        {
            _mass = mass;

            return *(this);
        }

        Particle& setState(const Eigen::VectorXd& state)
        {
            _state = state;

            return *this;
        }

        Particle& setInput(const Eigen::VectorXd& input)
        {
            _input = input;

            return *this;
        }

        Particle& setIntegrator(const AbstractIntegrator& integrator)
        {
            _integrator = integrator;

            return *this;
        }

        Eigen::VectorXd dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            return (u + surfaceForce(x)) / _mass;
        }

        Eigen::VectorXd surfaceForce(const Eigen::MatrixXd& x)
        {
            Eigen::VectorXd surface_force(x.rows()), negative_altitude(x.rows());
            negative_altitude = -x.col(2);

            surface_force = (x.col(1).array() * x.col(0).array().sin() - x.col(0).array() * x.col(1).array().cos()) * negative_altitude.array().exp();

            return surface_force;
        }

    protected:
        double _mass;

        Eigen::VectorXd _state, _input;

        AbstractIntegrator _integrator;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP