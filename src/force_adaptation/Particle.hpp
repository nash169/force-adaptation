#ifndef FORCEADAPTATION_PARTICLE_HPP
#define FORCEADAPTATION_PARTICLE_HPP

#include <Eigen/Core>
#include <integrator_lib/Integrate.hpp>
#include <iostream>
#include <memory>

namespace force_adaptation {
    using namespace integrator_lib;

    struct DefaultParams {
        struct integrator : public integrator_lib::defaults::integrator {
        };
    };

    template <typename Integrator = integrator::ForwardEuler<DefaultParams>>
    class Particle {
    public:
        Particle(const double& mass = 1, const double& load = 0, const Eigen::Vector3d& reference = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& frame = Eigen::Matrix3d::Identity())
            : _mass(mass), _load(load), _frame(frame), _reference(reference)
        {
            _integrator = Integrator();

            _state = Eigen::VectorXd::Zero(6);
            _input = Eigen::VectorXd::Zero(3);
        }

        ~Particle() {}

        /* Getter method */

        double mass() const
        {
            return _mass;
        }

        double load() const
        {
            return _load;
        }

        Eigen::VectorXd state() const
        {
            return _state;
        }

        Eigen::VectorXd input() const
        {
            return _input;
        }

        /* Setter method */

        Particle& setMass(const double& mass)
        {
            _mass = mass;

            return *(this);
        }

        Particle& setLoad(const double& load)
        {
            _load = load;

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

        /* Dynamics and Integration */

        Eigen::VectorXd dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
        {
            Eigen::VectorXd acc(6);
            acc << x.tail(3), (u + surfaceForce(x.head(3)) + contactForce(x.head(3))) / _mass; // (u + surfaceForce(x.head(3)) + contactForce(x.head(3))) / _mass;

            return acc;
        }

        void update(const double& time)
        {
            _state = _integrator.integrate(*this, &Particle::dynamics, time, _state, _input);
        }

        /* Contact force */

        Eigen::Vector3d contactForce(const Eigen::Vector3d& x) const
        {
            Eigen::Vector3d contact_force = Eigen::Vector3d::Zero();

            if (x(2) < 0.01)
                contact_force(2) = -_load;

            return _frame * contact_force;
        }

        /* Surface force */

        // Eigen::Vector3d surfaceForce(const Eigen::VectorXd& x)
        // {
        //     Eigen::Vector3d surface_force = Eigen::Vector3d::Zero(), proj;

        //     proj = _frame.transpose() * (x - _reference);

        //     surface_force(2) = (x(1) * sin(x(0)) - x(0) * cos(x(1))) * exp(-x(2)) * 100 + 10;

        //     return _frame * surface_force;
        // }

        Eigen::Vector3d surfaceForce(const Eigen::VectorXd& x) const
        {
            Eigen::Vector3d surface_force = Eigen::Vector3d::Zero();

            if (x(2) < 0)
                surface_force(2) = -(1000 + 100000 * x(0) * x(0) + 400000 * x(1) * x(1)) * x(2);

            return _frame * surface_force;
        }

        // Eigen::Vector3d distributionForce(const Eigen::MatrixXd& x)
        // {
        //     Eigen::Vector3d surface_force;

        //     if (x(2) < 0.0f)
        //         surface_force(2) = -(1000 + 100000 * x(0) * x(0) + 400000 * x(1) * x(1)) * x(2);

        //     return _frame * surface_force;
        // }

        Eigen::VectorXd distributionForce(const Eigen::MatrixXd& x) const
        {
            Eigen::VectorXd surface_force(x.rows()), negative_altitude(x.rows());
            negative_altitude = -x.col(2);

            surface_force = (x.col(1).array() * x.col(0).array().sin() - x.col(0).array() * x.col(1).array().cos()) * negative_altitude.array().exp();

            return surface_force;
        }

    protected:
        double _mass, _load;

        Eigen::Vector3d _reference;
        Eigen::Matrix3d _frame;

        Eigen::VectorXd _state, _input;

        Integrator _integrator;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP