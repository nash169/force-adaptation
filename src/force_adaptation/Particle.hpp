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
        Particle(const double mass = 1, const Eigen::Vector3d& reference = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& frame = Eigen::Matrix3d::Identity(), const double step = 0.001) : _mass(mass), _frame(frame), _reference(reference)
        {
            _integrator = Integrator();
            _integrator.setStep(step);
            _state = Eigen::VectorXd::Zero(6);
            _input = Eigen::VectorXd::Zero(3);
        }

        ~Particle() {}

        void update(const double& time)
        {
            _state = _integrator.integrate(*this, &Particle::dynamics, time, _state, _input);
        }

        double mass()
        {
            return _mass;
        }

        Eigen::VectorXd state()
        {
            return _state;
        }

        Eigen::VectorXd input()
        {
            return _input;
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

        Eigen::VectorXd velocity(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            return _state.tail(3);
        }

        Eigen::VectorXd dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            Eigen::VectorXd acc(6);
            acc << x.tail(3), (u + surfaceForce(x.head(3))) / _mass;

            return acc;
        }

        // Eigen::Vector3d surfaceForce(const Eigen::VectorXd& x)
        // {
        //     Eigen::Vector3d surface_force = Eigen::Vector3d::Zero(), proj;

        //     proj = _frame.transpose() * (x - _reference);

        //     surface_force(2) = (x(1) * sin(x(0)) - x(0) * cos(x(1))) * exp(-x(2)) * 100 + 10;

        //     return _frame * surface_force;
        // }

        Eigen::Vector3d surfaceForce(const Eigen::VectorXd& x)
        {
            Eigen::Vector3d surface_force;

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

        Eigen::VectorXd distributionForce(const Eigen::MatrixXd& x)
        {
            Eigen::VectorXd surface_force(x.rows()), negative_altitude(x.rows());
            negative_altitude = -x.col(2);

            surface_force = (x.col(1).array() * x.col(0).array().sin() - x.col(0).array() * x.col(1).array().cos()) * negative_altitude.array().exp();

            return surface_force;
        }

    protected:
        double _mass;

        Eigen::Vector3d _reference;
        Eigen::Matrix3d _frame;

        Eigen::VectorXd _state, _input;

        Integrator _integrator;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP